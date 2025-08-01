import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the path to the config file
    calib_dir = get_package_share_directory('camera_2d_lidar_calibration')
    config_file = os.path.join(calib_dir, 'config', 'config.yaml')
    rviz_file = os.path.join(calib_dir, 'rviz', 'show_ros2.rviz')
    return LaunchDescription([
        Node(
            package='camera_2d_lidar_calibration',
            executable='collect_camera_lidar_data.py',
            name='collect_camera_lidar_data',
            output='screen',
            parameters=[config_file]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen'
        )
    ])