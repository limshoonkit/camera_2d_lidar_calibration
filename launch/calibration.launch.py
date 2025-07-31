from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_2d_lidar_calibration',
            executable='camera_lidar_calibration.py',
            name='camera_lidar_calibration',
            output='screen',
            parameters=[
                {'config_file': 'config/config.yaml'},
                {'data_file': 'data/data.txt'},
                {'result_file': 'data/calibration_result.txt'}
            ]
        )
    ])
