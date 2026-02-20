#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import math


class JoystickTeleop(Node):

    def __init__(self):
        super().__init__("joystick_teleop_node")

        # Publisher → sends joint angles to Unity
        self.joint_pub = self.create_publisher(JointState,"/joint_angle", 10)

        # Subscriber → reads joystick input
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # ⚠ These names MUST match your URDF exactly
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        # Joint limits (in radians)
        self.joint_limits = [
            (-math.pi/2, math.pi/2),   # Joint 1
            (-math.pi/2, math.pi/2),   # Joint 2
            (-math.pi/2, math.pi/2),   # Joint 3
            (-math.pi/2, math.pi/2),   # Joint 4
            (-math.pi/2, 0),           # Joint 5
            (0, math.pi/2),            # Joint 6
        ]

        self.get_logger().info("Joystick teleop node started")


    def joy_callback(self, joy_msg):
        """
        Convert joystick axes (-1 to 1)
        into joint angles within defined limits.
        """

        # Mapping joystick axes to robot joints
        axes_mapping = [
            joy_msg.axes[0],
            joy_msg.axes[1],
            joy_msg.axes[3],
            joy_msg.axes[4],
            joy_msg.axes[2],
            joy_msg.axes[5]
        ]

        joint_positions = []

        for i in range(len(self.joint_names)):
            min_limit, max_limit = self.joint_limits[i]
            axis_value = axes_mapping[i]

            # Scale joystick value [-1,1] → [min_limit,max_limit]
            scaled_angle = min_limit + (max_limit - min_limit) * ((axis_value + 1) / 2)

            joint_positions.append(scaled_angle)

        # Create JointState message
        joint_msg = JointState()
        joint_msg.name = self.joint_names
        joint_msg.position = joint_positions

        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    node = JoystickTeleop()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
