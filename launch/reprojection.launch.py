from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_2d_lidar_calibration',
            executable='reprojection.py',
            name='reprojection',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'image_topic': '/camera/image_raw'},
                {'calib_file': 'data/calibration_result.txt'},
                {'config_file': 'config/config.yaml'},
                {'laser_point_radius': 3},
                {'time_diff': 1.0}
            ]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', 'rviz/reprojection.rviz'],
            output='screen'
        )
    ])
