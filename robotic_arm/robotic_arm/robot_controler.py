#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import UInt8, Bool
import math
import numpy as np

from robotic_arm.kinematics import Kinematics  # ✅ correct import


class Robot_controller(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.mode = 1
        self.estop_state = False
        self.motor_state = False

        self.joint_pub = self.create_publisher(JointState, "/joint_angle", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.mode_sub = self.create_subscription(UInt8, "/mode", self.mode_callback, 10)
        self.estop_sub = self.create_subscription(Bool, "/e_stop", self.estop_callback, 10)
        self.motor_state_sub = self.create_subscription(Bool, "/motor_state", self.motor_state_callback, 10)

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]

        self.joint_limits = [
            (-2*math.pi, 2*math.pi),
            (-math.pi/2, math.pi/2),
            (-math.pi/2, math.pi/2),
            (-math.pi/2, math.pi/2),
            (-math.pi, math.pi)
        ]

        self.current_positions = [0.0] * len(self.joint_names)
        self.joint_speed = 0.03

        # ✅ Create kinematics object once
        self.kin = Kinematics()

        self.get_logger().info("Robot controller node started")

    # =========================
    # JOYSTICK CALLBACK
    # =========================
    def joy_callback(self, joy_msg):
        self.joy_msg = joy_msg

        if self.estop_state:
            self.get_logger().error("ESTOP ENABLED")
            return


        if not self.motor_state:
            return

        self.key_mapping = [
            joy_msg.axes[0],
            joy_msg.axes[1],
            joy_msg.axes[2],
            joy_msg.axes[3],
            joy_msg.axes[4],
            joy_msg.axes[5],
            joy_msg.axes[6],  # D-pad X
            joy_msg.axes[7],  # D-pad Y
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

    # =========================
    # MODE CALLBACK
    # =========================
    def mode_callback(self, mode_msg):
        self.mode = mode_msg.data

        if self.mode == 0:
            self.get_logger().info("Linear mode activated")
        else:
            self.get_logger().info("Joint mode activated")

    def estop_callback(self, estop_msg):
        self.estop_state = estop_msg.data

    def motor_state_callback(self, motor_state_msg):
        self.motor_state = motor_state_msg.data

    # =========================
    # JOINT MODE
    # =========================
    def joint_mode(self):
        for i in range(4):
            delta = self.key_mapping[i] * self.joint_speed
            self.current_positions[i] += delta

            min_limit, max_limit = self.joint_limits[i]
            self.current_positions[i] = max(min(self.current_positions[i], max_limit), min_limit)

        lt = self.joy_msg.axes[4]
        rt = self.joy_msg.axes[5]
        self.current_positions[4] += (rt - lt) * self.joint_speed

        min_limit, max_limit = self.joint_limits[4]
        self.current_positions[4] = max(min(self.current_positions[4], max_limit), min_limit)

        self.publish_joints()

    # =========================
    # LINEAR MODE (UPDATED)
    # =========================
    def linear_mode(self):

        step_r = 3.0      # radial movement (mm)
        step_theta = 0.05 # base rotation (rad)
        step_z = 3.0

        # -------------------------
        # Current joint state
        # -------------------------
        q_current = np.array(self.current_positions)

        # -------------------------
        # FK → current position
        # -------------------------
        current_pos, _ = self.kin.forward_kinematics(q_current)

        x, y, z = current_pos

        # -------------------------
        # Convert to polar
        # -------------------------
        r = math.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)

        # -------------------------
        # D-pad control (polar)
        # -------------------------
        d_theta = self.key_mapping[6] * step_theta   # left/right → rotate base
        d_r = self.key_mapping[7] * step_r           # up/down → extend/retract

        # -------------------------
        # Trigger → Z
        # -------------------------
        lt = self.joy_msg.axes[4]
        rt = self.joy_msg.axes[5]

        # Normalize triggers (for -1 to 1 controllers)
        lt_val = (1 - lt) / 2
        rt_val = (1 - rt) / 2

        dz = (lt_val - rt_val) * step_z

        # -------------------------
        # Apply movement
        # -------------------------
        theta += d_theta
        r += d_r
        z += dz

        # Prevent negative radius
        r = max(10.0, r)

        # -------------------------
        # Convert back to XYZ
        # -------------------------
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        target_pos = np.array([x, y, z])

        # -------------------------
        # Ignore no movement
        # -------------------------
        if np.allclose(target_pos, current_pos, atol=1e-3):
            return

        # -------------------------
        # IK
        # -------------------------
        q_new = self.kin.inverse_kinematics(target_pos, q_current)

        # -------------------------
        # Detect unreachable
        # -------------------------
        if np.allclose(q_new, q_current, atol=1e-6):
            self.get_logger().error(f"Target out of reach: {target_pos}")
            return

        # -------------------------
        # Wrist roll (optional)
        # -------------------------
        q_new[4] += (rt - lt) * self.joint_speed

        # -------------------------
        # Apply limits
        # -------------------------
        for i in range(len(q_new)):
            min_l, max_l = self.joint_limits[i]
            q_new[i] = max(min(q_new[i], max_l), min_l)

        self.current_positions = q_new.tolist()

        # -------------------------
        # Publish
        # -------------------------
        self.publish_joints()

    def publish_joints(self):
        joint_msg = JointState()
        joint_msg.name = self.joint_names
        joint_msg.position = self.current_positions
        self.joint_pub.publish(joint_msg)


# =========================
# MAIN
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = Robot_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()