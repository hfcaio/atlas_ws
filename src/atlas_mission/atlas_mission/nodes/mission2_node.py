#!/usr/bin/env python3
import rclpy
import threading
import math
import time
from atlas_mission.communication import Mav

# Ponto fixo para onde o yaw aponta sempre
INTEREST_X = 0.0
INTEREST_Y = 0.0

# Waypoints em círculo (raio=10m, altitude=7m)
RADIUS   = 10.0
ALTITUDE = 7.0
N_WPS    = 6

def generate_circle_waypoints(radius, altitude, n):
    wps = []
    for i in range(n):
        angle = 2 * math.pi * i / n
        wps.append((radius * math.cos(angle), radius * math.sin(angle), altitude))
    return wps

def yaw_to_interest(pose, ix, iy):
    """Calcula o yaw para apontar para (ix, iy) e compensa o offset +pi/2 do goto()."""
    raw = math.atan2(iy - pose.position.y, ix - pose.position.x)
    return raw - math.pi / 2

def distance(pose, x, y, z):
    return math.sqrt((pose.position.x - x)**2 + (pose.position.y - y)**2 + (pose.position.z - z)**2)

def main():
    rclpy.init()
    mav = Mav(debug=True)
    thread = threading.Thread(target=rclpy.spin, args=(mav,), daemon=True)
    thread.start()

    waypoints = generate_circle_waypoints(RADIUS, ALTITUDE, N_WPS)

    try:
        # 1. Descolagem
        mav.takeoff(5.0)

        # 2. Visitar cada waypoint com yaw dinâmico
        for i, (wx, wy, wz) in enumerate(waypoints):
            mav.get_logger().info(f"→ WP{i+1}: ({wx:.1f}, {wy:.1f}, {wz:.1f})")
            while rclpy.ok():
                yaw = yaw_to_interest(mav._pose, INTEREST_X, INTEREST_Y)
                mav.goto(x=wx, y=wy, z=wz, yaw=yaw)
                if distance(mav._pose, wx, wy, wz) < 0.8:
                    break
                time.sleep(0.1)

            # Hover 2s no WP com yaw fixo para o ponto de interesse
            t_end = time.time() + 2.0
            while time.time() < t_end:
                yaw = yaw_to_interest(mav._pose, INTEREST_X, INTEREST_Y)
                mav.goto(x=wx, y=wy, z=wz, yaw=yaw)
                time.sleep(0.1)

        # 3. Regressar à origem
        mav.get_logger().info("→ A regressar à origem...")
        while rclpy.ok():
            yaw = yaw_to_interest(mav._pose, INTEREST_X, INTEREST_Y)
            mav.goto(x=0.0, y=0.0, z=5.0, yaw=yaw)
            if distance(mav._pose, 0.0, 0.0, 5.0) < 0.8:
                break
            time.sleep(0.1)

        # 4. Aterrar
        mav.land()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
