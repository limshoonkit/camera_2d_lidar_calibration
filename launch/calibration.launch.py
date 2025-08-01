import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    package_dir = get_package_share_directory('camera_2d_lidar_calibration')
    
    # Full paths to config and data files
    config_file = os.path.join(package_dir, 'config', 'config.yaml')

    current_directory = os.getcwd()
    print(f'Current directory: {current_directory}')

    data_file = os.path.join(current_directory, 'data', 'data.txt')
    result_file = os.path.join(current_directory, 'data', 'calibration_result.txt')

    return LaunchDescription([
        Node(
            package='camera_2d_lidar_calibration',
            executable='camera_lidar_calibration.py',
            name='camera_lidar_calibration',
            output='screen',
            parameters=[
                config_file,  # This loads the YAML parameters
                {
                    'data_file': data_file,
                    'result_file': result_file,
                }
            ]
        )
    ])