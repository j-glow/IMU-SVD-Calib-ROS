from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to a rosbag2 dataset previously recorded with record_dataset.launch.py.')

    return LaunchDescription([
        bag_path_arg,
        Node(
            package='imu_svd_calib',
            executable='imu_calibration_node',
            name='calibrator',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
            output='screen'
        ),
    ])
