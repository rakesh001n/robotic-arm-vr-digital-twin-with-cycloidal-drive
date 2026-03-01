#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np


class Kinematics():
    
    def __init__(self):
        super().__init__("Kinematics Loading")

    def forward_kinematics(self):
        pass

    def inverse_kinematics(self):
        pass
    
    def dh_matrix(self):
        pass


def main():
    rclpy.init()
    node = Kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()