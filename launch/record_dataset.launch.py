from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_bag_path = os.path.join(
        os.getcwd(), 'bags', datetime.now().strftime('imu_dataset_%Y%m%d_%H%M%S'))

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Output path for the recorded rosbag2 dataset.')

    return LaunchDescription([
        bag_path_arg,
        Node(
            package='imu_svd_calib',
            executable='imu_simulator_node',
            name='simulator',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', LaunchConfiguration('bag_path'),
                '--topics', '/imu/data',
            ],
            output='screen'
        ),
    ])
