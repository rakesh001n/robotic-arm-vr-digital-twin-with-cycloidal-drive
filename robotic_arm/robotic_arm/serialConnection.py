#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import serial.tools.list_ports
import glob
import os
import time
import math

class JointToESP32(Node):
    def __init__(self):
        super().__init__('joint_to_esp32')
        self.ser = None
        self.last_send_time   = 0
        self.send_interval    = 0.1
        self.last_sent_angles = None

        self.declare_parameter('port', '/dev/ttyACM0')  # your ESP32 port
        self.declare_parameter('baud', 115200)

        self.connect_serial()

        self.subscription = self.create_subscription(
            JointState,
            '/joint_angle',
            self.joint_callback,
            10
        )

        # Read ESP32 responses every 50ms
        self.create_timer(0.05, self.read_serial_response)

        self.get_logger().info("Ready. Listening to /joint_angle ...")

    # =====================
    # FIND PORT
    # =====================
    def find_esp32(self):
        param_port = self.get_parameter('port').get_parameter_value().string_value
        if param_port and os.path.exists(param_port):
            self.get_logger().info(f"Using param port: {param_port}")
            return param_port

        ports = serial.tools.list_ports.comports()
        self.get_logger().info(f"All detected ports ({len(ports)}):")
        for p in ports:
            self.get_logger().info(f"   {p.device} | {p.description} | {p.hwid}")

        KNOWN_VIDS = ["10C4", "1A86", "0403", "2341", "303A"]
        KNOWN_DESCS = [
            "cp210", "cp2102", "cp2104",
            "ch340", "ch341",
            "ftdi",  "ft232",
            "usb serial", "uart",
            "esp32", "espressif", "arduino"
        ]

        for p in ports:
            for vid in KNOWN_VIDS:
                if vid in p.hwid.upper():
                    self.get_logger().info(f"VID match {vid} → {p.device}")
                    return p.device

        for p in ports:
            for kw in KNOWN_DESCS:
                if kw in p.description.lower():
                    self.get_logger().info(f"Desc match '{kw}' → {p.device}")
                    return p.device

        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        self.get_logger().info(f"Trying fallback ports: {candidates}")
        for fp in candidates:
            try:
                t = serial.Serial(fp, 115200, timeout=0.3)
                t.close()
                self.get_logger().info(f"Fallback OK: {fp}")
                return fp
            except Exception as e:
                self.get_logger().warn(f"  {fp} failed: {e}")

        return None

    # =====================
    # CONNECT
    # =====================
    def connect_serial(self):
        port = self.find_esp32()
        if port is None:
            self.get_logger().error("No port found!")
            self.get_logger().error("  1. Check USB cable")
            self.get_logger().error("  2. Close Arduino Serial Monitor")
            self.get_logger().error("  3. Run: sudo chmod 666 /dev/ttyACM0")
            self.get_logger().error("  4. Or: ros2 run <pkg> <node> --ros-args -p port:=/dev/ttyACM0")
            return False

        try:
            baud = self.get_parameter('baud').get_parameter_value().integer_value

            self.ser = serial.Serial(
                port     = port,
                baudrate = baud,
                timeout  = 1,
                dsrdtr   = False,   # prevent ESP32 reset on connect
                rtscts   = False,
            )
            self.ser.dtr = False
            self.ser.rts = False

            self.get_logger().info(f"Port opened: {port}")
            self.get_logger().info("Waiting for ESP32 to boot...")
            time.sleep(2.5)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Test ping
            self.get_logger().info("Sending test ping...")
            self.ser.write(b'<0,90,90,90,90,0>\n')
            time.sleep(0.5)

            response = self.ser.read_all().decode(errors='ignore').strip()
            if response:
                self.get_logger().info(f"ESP32 response: {response}")
            else:
                self.get_logger().warn("No response to ping — will continue anyway")

            self.get_logger().info(f"✅ Connected: {port} @ {baud}")
            return True

        except serial.SerialException as e:
            self.get_logger().error(f"SerialException: {e}")
            if "Permission denied" in str(e):
                self.get_logger().error(f"  Fix: sudo chmod 666 {port}")
                self.get_logger().error(f"  Or:  sudo usermod -a -G dialout $USER")
            self.ser = None
            return False

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            self.ser = None
            return False

    # =====================
    # READ ESP32 RESPONSES
    # =====================
    def read_serial_response(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    self.get_logger().info(f"ESP32: {line}")
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    # =====================
    # JOINT CALLBACK
    # =====================
    def joint_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial dropped — reconnecting...")
            if not self.connect_serial():
                return

        now = time.time()
        if now - self.last_send_time < self.send_interval:
            return

        try:
            angles = msg.position

            NUM_JOINTS = 5  # change to 6 when gripper is added
            if len(angles) < NUM_JOINTS:
                self.get_logger().warn(
                    f"Expected {NUM_JOINTS} joints, got {len(angles)}"
                )
                return

            # rad → deg, clamped 0–180
            angles_deg = [
                max(0, min(180, int(a * 180.0 / math.pi)))
                for a in angles[:NUM_JOINTS]
            ]

            if self.last_sent_angles == angles_deg:
                return
            self.last_sent_angles = angles_deg

            # ✅ Ghost gripper = 0 (remove +[0] when real gripper added)
            angles_deg_padded = angles_deg + [0]
            data_str = "<" + ",".join(map(str, angles_deg_padded)) + ">\n"

            self.ser.write(data_str.encode())
            self.last_send_time = now
            self.get_logger().info(f"TX → {data_str.strip()}")

        except serial.SerialException as e:
            self.get_logger().error(f"Write failed: {e}")
            try:
                self.ser.close()
            except:
                pass
            self.ser = None

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JointToESP32()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()