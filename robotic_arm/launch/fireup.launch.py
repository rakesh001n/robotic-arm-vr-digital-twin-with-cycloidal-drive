from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Robot Controller
        Node(
            package='robotic_arm',
            executable='robot_controller_node',
            name='robot_controller',
            output='screen'
        ),

        # Pendant (joystick control)
        Node(
            package='robotic_arm',
            executable='pendant_node',
            name='pendant',
            output='screen'
        ),

        #  Kinematics
        #Node(
        #    package='robotic_arm',
        #    executable='kinematics_node',
        #    name='kinematics',
        #    output='screen'
        #),

        # Serial Communication (ESP32)
        Node(
            package='robotic_arm',
            executable='serialComms_node',
            name='serial_comms',
            output='screen'
        ),

        #  ROS TCP Endpoint (Unity / external comms)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='tcp_endpoint',
            output='screen',
            parameters=[{
                'ROS_IP': '127.0.0.1',   # change if needed
                'ROS_TCP_PORT': 10000
            }]
        )

    ])