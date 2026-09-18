#!/usr/bin/env python3

import rclpy
import threading
from atlas_mission.communication import Mav

def main():
    rclpy.init()
    mav = Mav(debug=True)
    thread = threading.Thread(target=rclpy.spin, args=(mav,), daemon=True)
    thread.start()

    rate = mav.create_rate(10)
    try:
        while rclpy.ok():
            mav.takeoff(1.0)
            mav.goto(x=1.0, y=0.0, z=1.0, yaw=0.0, send_time=5)
            mav.goto(x=1.0, y=1.0, z=2.0, yaw=0.0, send_time=5)
            mav.goto(x=0.0, y=1.0, z=3.0, yaw=0.0, send_time=5)
            mav.goto(x=0.0, y=0.0, z=4.0, yaw=0.0, send_time=5)
            if mav.land(): break
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
