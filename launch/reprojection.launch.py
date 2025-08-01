import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the path to the config file
    calib_dir = get_package_share_directory('camera_2d_lidar_calibration')
    config_file = os.path.join(calib_dir, 'config', 'config.yaml')
    rviz_file = os.path.join(calib_dir, 'rviz', 'reprojection.rviz')

    current_directory = os.getcwd()
    print(f'Current directory: {current_directory}')

    result_file = os.path.join(current_directory, 'data', 'calibration_result.txt')

    return LaunchDescription([
        Node(
            package='camera_2d_lidar_calibration',
            executable='reprojection.py',
            name='reprojection',
            output='screen',
            parameters=[
                config_file,
                {'scan_topic': '/scan'},
                {'calib_file': result_file},
                {'laser_point_radius': 3},
                {'time_diff': 1.0}
            ]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen'
        )
    ])
