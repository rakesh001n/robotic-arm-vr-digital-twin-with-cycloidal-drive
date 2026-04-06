#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import serial
import serial.tools.list_ports
import time
import math


class JointToESP32(Node):

    def __init__(self):
        super().__init__('joint_to_esp32')

        self.ser = None

        # 🔥 RATE CONTROL
        self.last_send_time = 0
        self.send_interval = 0.65   # 100 ms

        # 🔥 CHANGE FILTER
        self.last_sent_angles = None

        # Connect to ESP32
        self.connect_serial()

        # Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_angle',
            self.joint_callback,
            10
        )

        self.get_logger().info("Listening to /joint_angle (Serial)...")

    # 🔍 Auto-detect ESP32
    def find_esp32(self):
        ports = serial.tools.list_ports.comports()

        for port in ports:
            desc = port.description.lower()

            if "cp210" in desc or "ch340" in desc or "usb serial" in desc:
                self.get_logger().info(f"ESP32 detected on {port.device}")
                return port.device

        return None

    def connect_serial(self):
        port = self.find_esp32()

        if port is None:
            self.get_logger().warn("ESP32 not found")
            return False

        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to ESP32 on {port}")
            return True

        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None
            return False

    # 📡 Callback
    def joint_callback(self, msg):

        # 🔁 Reconnect if needed
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Reconnecting serial...")
            if not self.connect_serial():
                return

        # 🔥 RATE LIMIT
        now = time.time()
        if now - self.last_send_time < self.send_interval:
            return

        try:
            angles = msg.position

            # rad → deg
            angles_deg = [
                max(0, min(180, int(a * 180.0 / math.pi)))
                for a in angles
            ]

            # 🔥 CHANGE FILTER
            if self.last_sent_angles == angles_deg:
                return

            self.last_sent_angles = angles_deg

            # SAME FORMAT
            data_str = "<" + ",".join(map(str, angles_deg)) + ">\n"

            self.ser.write(data_str.encode())

            self.last_send_time = now

            self.get_logger().info(f"TX: {data_str.strip()}")

        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")

            try:
                self.ser.close()
            except:
                pass

            self.ser = None


def main(args=None):
    rclpy.init(args=args)

    node = JointToESP32()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()