#!/usr/bin/env python3
"""
vtol_agent.py - o VTOL "encontra a pessoa" e manda a posicao ao quadcopter.

Substitui o `fake_target` da missao Icarus: em vez de um publisher falso,
quem publica o alvo e um outro veiculo (o VTOL) que voa ate la.

    VTOL: WAIT_FCU -> GUIDED -> ARM -> TAKEOFF -> SEARCH -> DETECT (publica) -> HOVER -> LAND
    Quad: corre `icarus_tests mission` e so descola quando recebe a mensagem.

Mensagem enviada (igual a do fake_target, ver atlas_mission/fake_target.py):
    /mavros/mission/vtol_person_position   sensor_msgs/NavSatFix   (BEST_EFFORT)
publicada a `rate_hz` (2 Hz) ate `stop_after_s` depois da deteccao.

Corre-se dentro do namespace do VTOL (o mesmo do mavros dele):

    python3 vtol_agent.py --ros-args -r __ns:=/vtol \
        -p takeoff_alt:=20.0 -p person_x:=40.0 -p person_y:=25.0

person_x/person_y estao no frame MUNDO do Gazebo (ENU, metros). A lat/lon e
calculada a partir da posicao GPS + local do proprio VTOL (os SITL partilham
a mesma origem GPS, por isso o quad le exatamente o mesmo sitio).

O VTOL usa Q_GUIDED_MODE=1 (GUIDED comporta-se como VTOL: sobe na vertical e
segue setpoints locais, sem transicao para voo de asa fixa).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

EARTH_R = 6378137.0

PUB_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                     reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE)


class VtolAgent(Node):
    def __init__(self):
        super().__init__('agent')
        d = self.declare_parameter
        self.takeoff_alt = float(d('takeoff_alt', 20.0).value)
        self.search = (float(d('search_x', 20.0).value),
                       float(d('search_y', 10.0).value))
        self.person = (float(d('person_x', 40.0).value),
                       float(d('person_y', 25.0).value))
        self.person_alt = float(d('person_alt', 0.0).value)   # altitude reportada (m)
        self.tol = float(d('tolerance', 1.5).value)
        self.search_timeout = float(d('search_timeout_s', 60.0).value)
        self.rate_hz = float(d('rate_hz', 2.0).value)
        self.stop_after = float(d('stop_after_s', 120.0).value)
        self.land_at_end = bool(d('land_at_end', True).value)
        self.topic = str(d('target_topic', '/mavros/mission/vtol_person_position').value)

        self.state = State()
        self.pose = None
        self.fix = None
        self.phase = 'WAIT_FCU'
        self.last_req = {}
        self.ticks = 0
        self.phase_t = 0.0
        self.detect_t = None
        self.hold = None
        self.sent = 0
        self.person_ll = None

        self.create_subscription(State, 'mavros/state', self._on_state, 10)
        self.create_subscription(PoseStamped, 'mavros/local_position/pose',
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, 'mavros/global_position/global',
                                 self._on_fix, qos_profile_sensor_data)
        self.sp_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        self.tgt_pub = self.create_publisher(NavSatFix, self.topic, PUB_QOS)
        self.status_pub = self.create_publisher(String, 'status', 10)

        self.mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.stream_cli = self.create_client(StreamRate, 'mavros/set_stream_rate')

        self.create_timer(0.05, self._tick)
        self.create_timer(1.0 / self.rate_hz, self._publish_target)
        self.get_logger().info(
            f'vtol pronto | pessoa (mundo)={self.person} | publica em {self.topic}')

    # ---------------------------------------------------------------- util
    def _on_state(self, m): self.state = m
    def _on_pose(self, m): self.pose = m
    def _on_fix(self, m): self.fix = m

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pos(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def _go(self, phase):
        self.get_logger().info(f'[{self.phase}] -> [{phase}]')
        self.phase = phase
        self.phase_t = self._now()

    def _request(self, name, client, req, every):
        t = self._now()
        if t - self.last_req.get(name, -1e9) < every or not client.service_is_ready():
            return
        self.last_req[name] = t
        client.call_async(req)
        self.get_logger().info(f'-> servico {name}')

    def _sp(self, xyz):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        self.sp_pub.publish(m)

    def _person_latlon(self):
        """lat/lon da pessoa: origem do mundo = GPS atual - posicao local."""
        x, y, _ = self._pos()
        lat0 = self.fix.latitude - math.degrees(y / EARTH_R)
        lon0 = self.fix.longitude - math.degrees(
            x / (EARTH_R * math.cos(math.radians(self.fix.latitude))))
        px, py = self.person
        lat = lat0 + math.degrees(py / EARTH_R)
        lon = lon0 + math.degrees(px / (EARTH_R * math.cos(math.radians(lat0))))
        return lat, lon

    # --------------------------------------------------------------- estados
    def _tick(self):
        self.ticks += 1
        if self.ticks % 20 == 0:
            self.status_pub.publish(String(data=self.phase))
            if self.ticks % 100 == 0 and self.pose:
                x, y, z = self._pos()
                self.get_logger().info(
                    f'{self.phase} modo={self.state.mode} armado={self.state.armed} '
                    f'pos=({x:.1f},{y:.1f},{z:.1f}) enviados={self.sent}')
        getattr(self, f'_ph_{self.phase.lower()}')()

    def _ph_wait_fcu(self):
        if self.state.connected and self.pose is not None and self.fix is not None:
            self._go('GUIDED')
        elif self.state.connected:
            self._request('set_stream_rate', self.stream_cli,
                          StreamRate.Request(stream_id=0, message_rate=10, on_off=True), 5.0)

    def _ph_guided(self):
        if self.state.mode == 'GUIDED':
            self._go('ARM')
        else:
            self._request('set_mode', self.mode_cli, SetMode.Request(custom_mode='GUIDED'), 2.0)

    def _ph_arm(self):
        if self.state.armed:
            self._go('TAKEOFF')
        else:
            self._request('arm', self.arm_cli, CommandBool.Request(value=True), 3.0)

    def _ph_takeoff(self):
        if not self.state.armed:
            self._go('ARM')
            return
        self._request('takeoff', self.takeoff_cli,
                      CommandTOL.Request(altitude=self.takeoff_alt), 15.0)
        if self._pos()[2] >= self.takeoff_alt - 1.0:
            self._go('SEARCH')

    def _ph_search(self):
        # voa para o ponto de procura (antes da pessoa); sai por chegada ou timeout
        tgt = (self.search[0], self.search[1], self.takeoff_alt)
        self._sp(tgt)
        if (math.dist(self._pos(), tgt) <= self.tol
                or self._now() - self.phase_t > self.search_timeout):
            self.hold = tgt
            self._go('DETECT')

    def _ph_detect(self):
        x, y, _ = self._pos()
        self.hold = (x, y, self.takeoff_alt)
        lat, lon = self._person_latlon()
        self.person_ll = (lat, lon)      # fixo: nao deriva com a pose do VTOL
        self.detect_t = self._now()
        self.get_logger().warn(f'PESSOA DETETADA em ({lat:.7f}, {lon:.7f}) '
                               f'(mundo {self.person}) - a publicar em {self.topic}')
        self._go('HOVER')

    def _ph_hover(self):
        self._sp(self.hold)
        if self._now() - self.detect_t >= self.stop_after:
            self._go('LAND' if self.land_at_end else 'DONE')

    def _ph_done(self):
        self._sp(self.hold)

    def _ph_land(self):
        if self.state.mode not in ('QLAND', 'LAND'):
            self._request('set_mode', self.mode_cli, SetMode.Request(custom_mode='QLAND'), 2.0)

    # ---------------------------------------------------------- comunicacao
    def _publish_target(self):
        if self.detect_t is None or self.phase not in ('HOVER',) or self.fix is None:
            return
        lat, lon = self.person_ll
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'wgs84'
        m.latitude, m.longitude, m.altitude = lat, lon, self.person_alt
        self.tgt_pub.publish(m)
        self.sent += 1


def main():
    rclpy.init()
    node = VtolAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
