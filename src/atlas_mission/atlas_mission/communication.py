import rclpy
import numpy as np
from dataclasses import dataclass
from time import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

# CONSTANTS
PI = np.pi
HALF_PI = PI / 2

@dataclass
class Numeric:
  int, float

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list:
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

class Mav(Node):
  def __init__(self, debug: bool = False, indoor: bool = False, QoS: int = 10) -> None:
    
    #INITIALIZING NODE
    super().__init__('mav')
    self._indoor = indoor
    self._debug = debug
    if self._debug:
      self.get_logger().info("Mavros node initialized")
    
    # INITIALIZING SUBSCRIBERS
    self._pose_sub = self.create_subscription(PoseStamped,'/mavros/vision_pose/pose', self.pose_callback, QoS) if self._indoor else self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, QoS)

    # INITIALIZING PUBLISHERS
    self._position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', QoS)
    self._velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', QoS) 
    self._raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', QoS)

    # INITIALIZING SERVICES
    self._arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
    self._set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')
    self._takeoff_srv = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

    # ATRIBUTES
    self._pose = Pose()
    self._mode = int()

  def pose_callback(self, msg: PoseStamped) -> None:
    """
    ROS callback used to get local position PoseStamped messages.
    """
    self._pose = msg.pose
  
  def change_mode(self, mode: str) -> bool:
    """
    Changes vehicle mode to given string. Some modes include:
    STABILIZE = 0,
    GUIDED = 4,
    LAND = 9
    """
    if self._debug: self.get_logger().info(f"[MODE] Changing mode to {mode}")
    self._set_mode_srv.wait_for_service()
    return self._set_mode_srv.call_async(SetMode.Request(custom_mode=mode))

  def arm(self) -> bool:
    """
    Arms the vehicle.
    """
    if self._debug: self.get_logger().info("[ARM] Arming vehicle")
    self._arm_srv.wait_for_service()
    return self._arm_srv.call_async(CommandBool.Request(value=True))

  def takeoff(self, altitude: float) -> bool:
    """
    Sends a takeoff command to the vehicle.
    """
    if self._pose.position.z > altitude - 0.2 and self._pose.position.z < altitude + 0.1:
      if self._debug: self.get_logger().info("Already at desired altitude")
      return True

    if not self.change_mode("4"):
      if self._debug: self.get_logger().error("Could not change mode to GUIDED")
      return False

    if not self.arm():
      if self._debug: self.get_logger().error("Could not arm vehicle")
      return False
    
    if self._debug: self.get_logger().info(f"[TAKEOFF] Taking off to {altitude} meters")
    self._takeoff_srv.wait_for_service()
    if not self._takeoff_srv.call_async(CommandTOL.Request(altitude=altitude)):
      if self._debug: self.get_logger().error("Could not send takeoff command")
      return False
    
    start_time = time()
    self.get_logger().warn(f"Start time: {start_time}")
    while (time() - start_time) < 5:
      if self._pose.position.z > altitude - 0.2:
        if self._debug: self.get_logger().info("Takeoff successful")
        return True

    if self._debug: self.get_logger().error("Takeoff failed")
    return False
  
  def disarm(self) -> bool:
    """
    Disarms the vehicle.
    """
    if self._debug: self.get_logger().info("[DISARM] Disarming vehicle")
    self._arm_srv.wait_for_service()
    return self._arm_srv.call_async(CommandBool.Request(value=False))
  
  def land(self) -> bool:
    """
    Sends a land command to the vehicle.
    """
    return self.change_mode("9") and self.disarm()
  
  def set_velocity(self, vel_x: Numeric = 0, vel_y: Numeric = 0, vel_z: Numeric = 0, ang_x: Numeric = 0, ang_y: Numeric = 0, ang_z: Numeric = 0) -> None:
    """
    Sends a Twist message and publishes it as a setpoint (assuming vehicle is in guided mode).
    """
    vel_msg = Twist()
    vel_msg.linear.x = vel_x
    vel_msg.linear.y = vel_y
    vel_msg.linear.z = vel_z
    vel_msg.angular.x = ang_x
    vel_msg.angular.y = ang_y
    vel_msg.angular.z = ang_z

    if self._debug: self.get_logger().info(f"[SET_VELOCITY] Sending Twist: {vel_msg}")
    self._velocity_pub.publish(vel_msg)

  def set_velocity_relative(self, forward: Numeric = 0, sideways: Numeric = 0, upward: Numeric = 0) -> None:
    """
    Sends a PositionTarget message and publishes it as a setpoint (assuming vehicle is in guided mode).
    Just uses the velocity fields of the message.
    frame: MAV_FRAME_BODY_NED
    """
    msg = PositionTarget()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'base_footprint'
    
    msg.coordinate_frame = PositionTarget.FRAME_BODY_NED

    msg.velocity.x = forward
    msg.velocity.y = sideways
    msg.velocity.z = upward

    msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
    
    if self._debug: self.get_logger().info(f"[SET_VELOCITY_RELATIVE] Sending PositionTarget: {msg}")
    self._raw_pub.publish(msg)

  def rotate_yaw_relative(self, yaw: Numeric,yaw_rate: Numeric = 0.5) -> None:
    """
    Sends a PositionTarget message and publishes it as a setpoint (assuming vehicle is in guided mode).
    Just uses the yaw fields of the message.
    frame: MAV_FRAME_BODY_NED
    """
    msg = PositionTarget()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'base_footprint'
    
    msg.coordinate_frame = PositionTarget.FRAME_BODY_NED

    msg.yaw = yaw
    msg.yaw_rate = yaw_rate

    msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
    
    if self._debug: self.get_logger().info(f"[ROTATE_YAW_RELATIVE] Sending PositionTarget: {msg}")
    self._raw_pub.publish(msg) 

  def rotate(self, yaw: Numeric, send_time: Numeric = None) -> None:
    """
    Same as goto but only changes yaw.
    """
    self.goto(yaw=yaw, send_time=send_time)

  def goto(self, x: Numeric = None, y: Numeric = None, z: Numeric = None, yaw: Numeric = None, send_time = None, min_distance: Numeric = None) -> None:
    """
    Sends a Pose message and publishes it as a setpoint (assuming vehicle is in guided mode). Yaw is offset by pi/2.
    If movement in a specifit axis is not provided, assumes that you want to keep the vehicles current axial position.
    """

    # creating Pose message
    goal_pose = PoseStamped()

    # setting header
    goal_pose.header.stamp = self.get_clock().now().to_msg()
    goal_pose.header.frame_id = 'base_link'
    
    # setting position
    goal_pose.pose.position.x = self._pose.position.x if x is None else x
    goal_pose.pose.position.y = self._pose.position.y if y is None else y
    goal_pose.pose.position.z = self._pose.position.z if z is None else z
    
    # setting orientation
    q = euler_to_quaternion(0, 0, yaw + HALF_PI)
    goal_pose.pose.orientation = self._pose.orientation if yaw is None else Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    # TODO: add min_distance logic
    if min_distance is not None:
      if self._debug: self.get_logger().warn("Min distance not implemented yet")
      return

    # publishing
    if send_time is None:
      if self._debug: self.get_logger().info(f"[GOTO] Sending to Pose: {goal_pose}")
      self._position_pub.publish(goal_pose)
      return 

    # publishing with time
    if self._debug: self.get_logger().info(f"[GOTO] Sending to Pose: {goal_pose} for {send_time} seconds")
    start_time = time()
    while (time() - start_time) < send_time:
      self._position_pub.publish(goal_pose)



