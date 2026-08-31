import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('imu_svd_calib'),
        'rviz',
        'calibration.rviz')

    return LaunchDescription([
        Node(
            package='imu_svd_calib',
            executable='imu_simulator_node',
            name='simulator',
            output='screen'
        ),
        Node(
            package='imu_svd_calib',
            executable='imu_calibration_node',
            name='calibrator',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])
