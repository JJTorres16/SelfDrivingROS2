from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_pyhton",
        default_value="True"
    )

    use_python = LaunchConfiguration("use_python")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.103", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", 
            "--frame_id", "base_frootpin_efk",
            "--child_frame_id", "imu_link_efk"
        ]
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[

        ]
    )

    return LaunchDescription([

    ])