#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import UInt8, Bool
import math

class Robot_controller(Node):

    def __init__(self):
        super().__init__("robot_controller_node")
        self.mode = 1
        self.estop_state = False
        self.motor_state = False
        self.joint_pub = self.create_publisher(JointState,"/joint_angle", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.mode_sub = self.create_subscription(UInt8, "/mode", self.mode_callback, 10)
        self.estop_sub = self.create_subscription(Bool, "/e_stop", self.estop_callback, 10)
        self.motor_state_sub = self.create_subscription(Bool, "/motor_state", self.motor_state_callback, 10)
        # DEFINE FIRST
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5"]

        self.joint_limits = [
            (-math.pi/2, math.pi/2),
            (-math.pi/2, math.pi/2),
            (-math.pi/2, math.pi/2),
            (-math.pi/2, math.pi/2),
            (-math.pi, math.pi)
        ]

        # THEN USE IT
        self.current_positions = [0.0] * len(self.joint_names)
        self.joint_speed = 0.03

        self.get_logger().info("Robot controller node started")


    def joy_callback(self, joy_msg):
        self.joy_msg = joy_msg

        # STOP immediately if motor disabled or estop active
        if not self.motor_state :
            return

        self.key_mapping = [
            joy_msg.axes[0],
            joy_msg.axes[1],
            joy_msg.axes[2],
            joy_msg.axes[3],
            joy_msg.axes[4],
            joy_msg.axes[5],
            joy_msg.axes[6],
            joy_msg.axes[7],
            joy_msg.buttons[0],
            joy_msg.buttons[1],
            joy_msg.buttons[3],
            joy_msg.buttons[4],
            joy_msg.buttons[6],
            joy_msg.buttons[7],
            joy_msg.buttons[10],
            joy_msg.buttons[11],
            joy_msg.buttons[13],
            joy_msg.buttons[14],
        ]

        if self.mode == 0:
            self.linear_mode()
        else:
            self.joint_mode()

        
    def mode_callback(self, mode_msg):
        self.mode = mode_msg.data
        if self.mode == 0:
            self.get_logger().info("Linear mode activated")
            if self.motor_state:
                self.linear_mode()
        else:
            self.get_logger().info("Joint mode activated")
            if self.motor_state:
                self.joint_mode()
            
    
    def estop_callback(self, estop_msg):
        self.estop_state = estop_msg.data
        if self.estop_state:
            self.get_logger().warn("ESTOP ENABLED AND MOTOR DISABLED")
        else:
            self.get_logger().info("ESTOP CLEARED")

    def motor_state_callback(self, motor_state_msg):
        self.motor_state = motor_state_msg.data
        if self.motor_state:
            self.get_logger().info("MOTOR IS ENABLED")

        else:

            self.get_logger().warn("MOTOR IS DISABLED")


    def joint_mode(self):
        for i in range(4):
            delta = self.key_mapping[i] * self.joint_speed
            self.current_positions[i] += delta
            min_limit, max_limit = self.joint_limits[i]
            self.current_positions[i] = max(min(self.current_positions[i], max_limit), min_limit)

        # Joint 5 controlled by LT/RT
        lt = self.joy_msg.axes[4]  # range -1 to 1 (or 0 to 1 depending on controller)
        rt = self.joy_msg.axes[5]

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

    def linear_mode(self):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = Robot_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()