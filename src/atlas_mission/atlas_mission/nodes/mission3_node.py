#!/usr/bin/env python3
import rclpy
import threading
import math
import time
from atlas_mission.communication import Mav

# Waypoints em linha reta (x, y, z)
CRUISE_ALT = 30.0
WAYPOINTS = [
    ( 50.0,  0.0, CRUISE_ALT),
    (150.0,  0.0, CRUISE_ALT),
    (300.0,  0.0, CRUISE_ALT),
    (450.0,  0.0, CRUISE_ALT),
]
WAYPOINT_TOL = 15.0  # tolerância XY — avião não para no WP

def distance_xy(pose, x, y):
    return math.sqrt((pose.position.x - x)**2 + (pose.position.y - y)**2)

def main():
    rclpy.init()
    mav = Mav(debug=True)
    thread = threading.Thread(target=rclpy.spin, args=(mav,), daemon=True)
    thread.start()

    rate = mav.create_rate(10)
    try:
        # 1. Descolagem
        mav.takeoff(CRUISE_ALT)

        # 2. Voar pelos waypoints em linha reta
        for i, (x, y, z) in enumerate(WAYPOINTS):
            mav.get_logger().info(f"→ WP{i+1}: ({x:.0f}, {y:.0f}, {z:.0f})")
            while rclpy.ok():
                mav.goto(x=x, y=y, z=z, yaw=0.0)
                if distance_xy(mav._pose, x, y) < WAYPOINT_TOL:
                    mav.get_logger().info(f"✓ WP{i+1} atingido.")
                    break
                time.sleep(0.1)

        # 3. RTL — retorno automático e aterragem
        mav.get_logger().info("→ A iniciar RTL...")
        mav.change_mode("AUTO.RTL")

        while rclpy.ok():
            mav.get_logger().info(f"  Altitude: {mav._pose.position.z:.1f}m")
            if mav._pose.position.z < 2.0:
                break
            time.sleep(1.0)

        mav.get_logger().info("✓ Aterragem concluída.")
        mav.disarm()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
