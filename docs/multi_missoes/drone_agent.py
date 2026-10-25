#!/usr/bin/env python3
"""
drone_agent.py - um agente por drone (multi-missão no mesmo mundo Gazebo).

Corre-se UMA instância por drone, cada uma dentro do namespace desse drone
(o mesmo namespace usado no mavros), por exemplo:

    python3 drone_agent.py --ros-args -r __ns:=/drone1 -p ...

Como o namespace é aplicado pelo ROS, todos os tópicos abaixo são relativos
(ficam /drone1/mavros/..., /drone1/cmd/waypoint, /drone1/status, ...).

Máquina de estados:
    WAIT_FCU -> GUIDED -> ARM -> TAKEOFF -> MISSION  (percorre `waypoints`)
                                    ou -> LOITER   (se wait_for_cmd=true,
                                                    ou depois da missão)
    LOITER --(chega cmd/waypoint)--> GOTO_CMD --(chegou)--> LOITER ("ARRIVED")
    ARRIVED --(land_at_end, apos land_delay_s)--> LAND

Comunicação entre veículos (tópicos ROS 2):
    /<drone>/cmd/waypoint  geometry_msgs/PoseStamped  waypoint no frame MUNDO (ENU)
    /<drone>/status        std_msgs/String            fase atual (publicado a 1 Hz)

O drone A pode mandar um waypoint ao drone B (parâmetros peer_ns,
notify_at_wp, notify_wp). O envio repete-se a 1 Hz até o B confirmar
(status GOTO_CMD/ARRIVED), por isso não importa quem arranca primeiro.

Frames: os waypoints (missao e comandos) estao no frame MUNDO do Gazebo (ENU).
Com o mundo gerado pelo `ap-multi` (todos os SITL partem da mesma origem GPS)
o frame local do mavros de cada drone COINCIDE com o mundo, por isso
origin_x/origin_y ficam em 0.0 (medido: o drone2, spawnado em x=3, aparece em
pose_local=(3.0, 0.0)). Se usares SITL com homes diferentes (ex.: --home por
drone), poe em origin_x/y a posicao do home de cada um no mundo:
local = mundo - origem.

O MAVROS so publica local_position/pose se o ArduPilot enviar LOCAL_POSITION_NED;
o agente pede os stream rates (mavros/set_stream_rate) enquanto espera o FCU.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from std_msgs.msg import String


def parse_wps(text):
    """'x,y,z;x,y,z' -> [(x, y, z), ...]"""
    wps = []
    for item in text.split(';'):
        item = item.strip()
        if item:
            x, y, z = (float(v) for v in item.split(','))
            wps.append((x, y, z))
    return wps


class DroneAgent(Node):
    def __init__(self):
        super().__init__('agent')

        # ---- parâmetros -------------------------------------------------
        decl = self.declare_parameter
        self.origin = (decl('origin_x', 0.0).value, decl('origin_y', 0.0).value)
        self.takeoff_alt = decl('takeoff_alt', 5.0).value
        self.tol = decl('tolerance', 0.6).value          # m
        self.hold_s = decl('wp_hold_s', 2.0).value       # s parado em cada WP
        self.wps = parse_wps(decl('waypoints', '').value)
        self.wait_for_cmd = decl('wait_for_cmd', False).value
        self.land_at_end = decl('land_at_end', False).value
        self.land_delay = float(decl('land_delay_s', 5.0).value)  # espera em ARRIVED antes de aterrar
        self.arrived_t = None
        self.peer = str(decl('peer_ns', '').value).strip('/')   # ex.: 'drone2' (aceita '/drone2')
        self.notify_at_wp = decl('notify_at_wp', -1).value  # 1-based
        self.notify_wp = parse_wps(decl('notify_wp', '').value)

        # ---- estado interno ---------------------------------------------
        self.state = State()
        self.pose = None                 # PoseStamped no frame local
        self.phase = 'WAIT_FCU'
        self.hold = (0.0, 0.0, self.takeoff_alt)   # setpoint de loiter (local)
        self.wp_idx = 0
        self.reached_t = None
        self.cmd = None                  # waypoint recebido (frame mundo)
        self.cmd_target = None           # waypoint em execução (frame local)
        self.notify_pending = False
        self.notified = False
        self.last_req = {}
        self.ticks = 0
        self.last_status = ''

        # ---- ROS --------------------------------------------------------
        self.create_subscription(State, 'mavros/state', self._on_state, 10)
        # a pose do mavros é BEST_EFFORT: sensor_data QoS é compatível com tudo
        self.create_subscription(PoseStamped, 'mavros/local_position/pose',
                                 self._on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'cmd/waypoint', self._on_cmd, 10)

        self.sp_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)

        self.peer_pub = None
        if self.peer:
            self.peer_pub = self.create_publisher(
                PoseStamped, f'/{self.peer}/cmd/waypoint', 10)
            self.create_subscription(
                String, f'/{self.peer}/status', self._on_peer_status, 10)

        self.mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.stream_cli = self.create_client(StreamRate, 'mavros/set_stream_rate')

        self.create_timer(0.05, self._tick)        # 20 Hz
        self.get_logger().info(
            f'agente pronto | origem mundo={self.origin} | {len(self.wps)} WPs | '
            f'wait_for_cmd={self.wait_for_cmd} | peer="{self.peer}"')

    # ================= callbacks =========================================
    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg

    def _on_cmd(self, msg):
        p = msg.pose.position
        self.cmd = (p.x, p.y, p.z)
        self.get_logger().info(f'cmd/waypoint recebido (mundo): {self.cmd}')

    def _on_peer_status(self, msg):
        if self.notify_pending and msg.data.split()[0] in ('GOTO_CMD', 'ARRIVED'):
            self.notify_pending = False
            self.get_logger().info(f'{self.peer} confirmou ({msg.data}); '
                                   'a parar de reenviar o waypoint')

    # ================= utilitários =======================================
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _to_local(self, w):
        return (w[0] - self.origin[0], w[1] - self.origin[1], w[2])

    def _pos(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def _dist(self, tgt):
        p = self._pos()
        return math.dist(p, tgt)

    def _publish_sp(self, local_xyz):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = local_xyz
        m.pose.orientation.w = 1.0
        self.sp_pub.publish(m)

    def _request(self, name, client, req, every):
        """Chama um serviço no máximo de `every` em `every` segundos."""
        t = self._now()
        if t - self.last_req.get(name, -1e9) < every:
            return
        if not client.service_is_ready():
            return
        self.last_req[name] = t
        client.call_async(req)
        self.get_logger().info(f'-> serviço {name}')

    def _go(self, phase):
        self.get_logger().info(f'[{self.phase}] -> [{phase}]')
        self.phase = phase
        self.reached_t = None

    def _publish_status(self):
        txt = self.phase
        if self.phase == 'MISSION':
            txt += f' wp={self.wp_idx + 1}/{len(self.wps)}'
        self.status_pub.publish(String(data=txt))

    # ================= máquina de estados ================================
    def _tick(self):
        self.ticks += 1
        if self.ticks % 20 == 0:                   # 1 Hz
            self._publish_status()
            if self.notify_pending and self.peer_pub:
                self._send_to_peer()
            if self.ticks % 100 == 0 and self.pose:
                self.get_logger().info(
                    f'{self.phase} modo={self.state.mode} armado={self.state.armed} '
                    f'pos_local=({self._pos()[0]:.1f},{self._pos()[1]:.1f},'
                    f'{self._pos()[2]:.1f})')

        getattr(self, f'_ph_{self.phase.lower()}')()

    def _ph_wait_fcu(self):
        if self.state.connected and self.pose is not None:
            self._go('GUIDED')
        elif self.state.connected:
            # O MAVROS liga mas, sem pedir stream rates, o ArduPilot nao manda
            # LOCAL_POSITION_NED e nao ha local_position/pose. Pede-se aqui.
            self._request('set_stream_rate', self.stream_cli,
                          StreamRate.Request(stream_id=0, message_rate=10,
                                             on_off=True), 5.0)

    def _ph_guided(self):
        if self.state.mode == 'GUIDED':
            self._go('ARM')
        else:
            self._request('set_mode', self.mode_cli,
                          SetMode.Request(custom_mode='GUIDED'), 2.0)

    def _ph_arm(self):
        if self.state.armed:
            self._go('TAKEOFF')
        else:  # falha enquanto o EKF/GPS não está pronto; repete até conseguir
            self._request('arm', self.arm_cli, CommandBool.Request(value=True), 3.0)

    def _ph_takeoff(self):
        if not self.state.armed:          # desarmou (prearm / timeout) -> recomeça
            self._go('ARM')
            return
        self._request('takeoff', self.takeoff_cli,
                      CommandTOL.Request(altitude=float(self.takeoff_alt)), 15.0)
        if self._pos()[2] >= self.takeoff_alt - 0.5:
            x, y, _ = self._pos()
            self.hold = (x, y, self.takeoff_alt)
            self._go('LOITER' if (self.wait_for_cmd or not self.wps) else 'MISSION')

    def _ph_mission(self):
        tgt = self._to_local(self.wps[self.wp_idx])
        self._publish_sp(tgt)
        if self._dist(tgt) > self.tol:
            return
        t = self._now()
        if self.reached_t is None:
            self.reached_t = t
            self.get_logger().info(f'WP {self.wp_idx + 1} alcançado')
            if (self.wp_idx + 1 == self.notify_at_wp and not self.notified
                    and self.peer_pub and self.notify_wp):
                self.notified = True
                self.notify_pending = True
                self._send_to_peer()
        if t - self.reached_t >= self.hold_s:
            self.hold = tgt
            self.wp_idx += 1
            self.reached_t = None
            if self.wp_idx >= len(self.wps):
                self._go('LAND' if self.land_at_end else 'LOITER')

    def _ph_loiter(self):
        # GUIDED + setpoint constante = posição mantida (o "loiter" deste drone)
        self._publish_sp(self.hold)
        if self.cmd is not None:
            self.cmd_target = self._to_local(self.cmd)
            self.cmd = None
            self._go('GOTO_CMD')

    def _ph_goto_cmd(self):
        self._publish_sp(self.cmd_target)
        if self._dist(self.cmd_target) <= self.tol:
            self.hold = self.cmd_target
            self._go('LOITER')
            self.status_pub.publish(String(data='ARRIVED'))
            self.get_logger().info('ARRIVED no waypoint recebido')
            self.phase = 'ARRIVED'      # mantém "ARRIVED" no status a 1 Hz...
            self.arrived_t = self._now()

    def _ph_arrived(self):
        # ...mas comporta-se exatamente como LOITER...
        self._ph_loiter()
        # ...e, com land_at_end, aterra `land_delay_s` depois de chegar
        if (self.phase == 'ARRIVED' and self.land_at_end
                and self.arrived_t is not None
                and self._now() - self.arrived_t >= self.land_delay):
            self._go('LAND')

    def _ph_land(self):
        if self.state.mode != 'LAND':
            self._request('set_mode', self.mode_cli,
                          SetMode.Request(custom_mode='LAND'), 2.0)

    # ================= comunicação entre veículos ========================
    def _send_to_peer(self):
        x, y, z = self.notify_wp[0]
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        self.peer_pub.publish(m)
        self.get_logger().info(f'-> /{self.peer}/cmd/waypoint ({x}, {y}, {z})')


def main():
    rclpy.init()
    node = DroneAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
