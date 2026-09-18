#!/usr/bin/env python3
"""Publica um alvo falso para testar missoes em SITL.

A missao (ex. icarus_tests `mission`) espera a "posicao da pessoa" em
`/mavros/mission/vtol_person_position` (NavSatFix). Em voo real isso vem do
VTOL/visao; em SITL publicamos aqui um ponto fixo.

Default: ~300 m a norte do home CMAC do gazebo-iris (-35.363262, 149.165237).

    ros2 run atlas_mission fake_target
    ros2 run atlas_mission fake_target --ros-args -p latitude:=-35.3605 -p topic:=/mavros/mission/vtol_person_position
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class FakeTarget(Node):
    def __init__(self):
        super().__init__("fake_target")
        self.declare_parameter("topic", "/mavros/mission/vtol_person_position")
        self.declare_parameter("latitude", -35.3605622)
        self.declare_parameter("longitude", 149.1652375)
        self.declare_parameter("altitude", 603.0)
        self.declare_parameter("rate_hz", 2.0)

        topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(NavSatFix, topic, qos_profile_sensor_data)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"[fake_target] publicando em {topic} "
            f"({self.get_parameter('latitude').value}, "
            f"{self.get_parameter('longitude').value}, "
            f"{self.get_parameter('altitude').value}) @ {rate} Hz"
        )

    def _tick(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.latitude = float(self.get_parameter("latitude").value)
        msg.longitude = float(self.get_parameter("longitude").value)
        msg.altitude = float(self.get_parameter("altitude").value)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
