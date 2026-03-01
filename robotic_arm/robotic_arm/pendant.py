#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, Bool


class Pendant(Node):

    def __init__(self):
        super().__init__("pendant_node")
        self.get_logger().info("Pendant node started")
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.mode_pub = self.create_publisher(UInt8, "/mode", 10)
        self.estop_state_pub = self.create_publisher(Bool, "/e_stop", 10)
        self.motor_state_pub = self.create_publisher(Bool, "/motor_state", 10)
        self.prev_buttons = [0] * 18
        self.motor_state = False   #motor state
        self.estop_state = False  #emergency stop state
        self.mode = 1 # 0 -> linear
                      # 1 -> Joint control
        


    def joy_callback(self, joy_msg):
        self.joy_msg = joy_msg
        self.key_mapping = [
            joy_msg.axes[0],  #LHJ -Left Horizontal joystick ----- 0
            joy_msg.axes[1],  #LVJ -Left Vertical joystick  ------ 1
            joy_msg.axes[2],  #RHJ -Right Horizontal joystick ---- 2
            joy_msg.axes[3],  #RVJ -Right Vertical joystick ------ 3
            joy_msg.axes[4],  #LT -Left Trigger ------------------ 4
            joy_msg.axes[5],  #RT -Right Trigger ----------------- 5
            joy_msg.axes[6],  #Dpad - Horizontal ----------------- 6
            joy_msg.axes[7],  #Dpad - Vertical ------------------- 7
            joy_msg.buttons[0],  #A ------------------------------ 8
            joy_msg.buttons[1],  #B ------------------------------ 9
            joy_msg.buttons[3],  #X ------------------------------ 10
            joy_msg.buttons[4],  #Y ------------------------------ 11
            joy_msg.buttons[6],  #LB ----------------------------- 12
            joy_msg.buttons[7],  #RB ----------------------------- 13
            joy_msg.buttons[10], #Select ------------------------- 14
            joy_msg.buttons[11], #Start -------------------------- 15
            joy_msg.buttons[13], #Left Joystick Button ----------- 16
            joy_msg.buttons[14], #Right Joystick Button ---------- 17
        ]

        self.mode_callback()
        self.estop_callback()
        self.motor_state_callback()
        # Update previous state memory
        self.prev_buttons = self.key_mapping.copy()

    
    def mode_callback(self):

        # Rising edge detection
        x_pressed = self.key_mapping[10] == 1 and self.prev_buttons[10] == 0
        y_pressed = self.key_mapping[11] == 1 and self.prev_buttons[11] == 0
        self.get_logger().debug(self.key_mapping[10])
        self.get_logger().debug(self.key_mapping[11])
    
        if x_pressed:
            self.mode = 0  # Linear Mode

        elif y_pressed:
            self.mode = 1  # Joint Mode



        # Publish as UInt8 message
        mode_msg = UInt8()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        
        

    def estop_callback(self):

        # Detect rising edges
        estop_pressed = self.key_mapping[9] == 1 and self.prev_buttons[9] == 0
        motor_pressed = self.key_mapping[12] == 1 and self.prev_buttons[12] == 0

        # If E-STOP button pressed → latch TRUE
        if estop_pressed:
            self.estop_state = True
            self.motor_state = False
            self.get_logger().warn("ESTOP ENABLED AND MOTOR DISABLED")

        # If motor button pressed AND estop currently active → clear estop
        elif motor_pressed and self.estop_state:
            self.estop_state = False
            self.get_logger().info("ESTOP CLEARED")
        else:
            self.estop_state = self.estop_state


        # Publish state
        estop_msg = Bool()
        estop_msg.data = self.estop_state
        self.estop_state_pub.publish(estop_msg)

        

    def motor_state_callback(self):
        motor_state_pressed = self.key_mapping[12] == 1 and self.prev_buttons[12] == 0
        if motor_state_pressed and not self.estop_state:
            self.motor_state = not self.motor_state

        if self.motor_state:
            self.get_logger().info("MOTOR IS ENABLED")
        else:
            self.get_logger().warn("MOTOR IS DISABLED")
        
        motor_state_msg = Bool()
        motor_state_msg.data = self.motor_state
        self.motor_state_pub.publish(motor_state_msg)

        
        


        
    

def main(args=None):
    rclpy.init(args=args)
    node = Pendant()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()