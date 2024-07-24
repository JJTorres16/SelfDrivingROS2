from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Joystick driver
    # Read the joystick command and publish them in ROS2
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml")
        ]
    )

    # Read from the ROS2 topic in wich the driver is publishsing the commands
    # and publish them at the cmd_vel messages
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_teleop.yaml")
        ]
    )


    return LaunchDescription([
        joy_node,
        joy_teleop
    ])