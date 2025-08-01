from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_2d_lidar_calibration',
            executable='collect_camera_lidar_data.py',
            name='collect_camera_lidar_data',
            output='screen',
            parameters=[
                {'image_topic': '/zed_node/left_raw/image_raw_color'},
                {'config_file': 'config/config.yaml'},
                {'output_file': 'data/data.txt'}
            ]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', 'rviz/show.rviz'],
            output='screen'
        )
    ])
