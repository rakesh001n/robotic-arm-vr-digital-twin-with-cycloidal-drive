#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math

class JoystickTeleop(Node):

    def __init__(self):
        super().__init__("joystick_teleop_node")

        self.joint_pub = self.create_publisher(JointState,"/joint_angle", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Only 5 joints now
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5"]
        self.joint_limits = [
            (-math.pi/2, math.pi/2),   # joint1
            (-math.pi/2, math.pi/2),   # joint2
            (-math.pi/2, math.pi/2),   # joint3
            (-math.pi/2, math.pi/2),   # joint4
            (-math.pi, math.pi)        # joint5 (wrist rotation)
        ]

        self.current_positions = [0.0] * len(self.joint_names)
        self.joint_speed = 0.03  # radians per update

        self.get_logger().info("Joystick teleop node started for 5 joints with LT/RT control for joint5")

    def joy_callback(self, joy_msg):
        # Map joystick axes for first 4 joints
        axes_mapping = [
            joy_msg.axes[0],  # joint1
            joy_msg.axes[1],  # joint2
            joy_msg.axes[2],  # joint3
            joy_msg.axes[3],  # joint4
        ]

        for i in range(4):
            delta = axes_mapping[i] * self.joint_speed
            self.current_positions[i] += delta
            min_limit, max_limit = self.joint_limits[i]
            self.current_positions[i] = max(min(self.current_positions[i], max_limit), min_limit)

        # Joint 5 controlled by LT/RT
        lt = joy_msg.axes[4]  # range -1 to 1 (or 0 to 1 depending on controller)
        rt = joy_msg.axes[5]

        # Determine net rotation direction
        delta5 = (rt - lt) * self.joint_speed  # RT positive, LT negative
        self.current_positions[4] += delta5
        min_limit, max_limit = self.joint_limits[4]
        self.current_positions[4] = max(min(self.current_positions[4], max_limit), min_limit)

        # Publish joint positions
        joint_msg = JointState()
        joint_msg.name = self.joint_names
        joint_msg.position = self.current_positions
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()