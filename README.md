# ROS2 camera_2d_lidar_calibration
port from https://github.com/ehong-tl/camera_2d_lidar_calibration

ROS2 camera 2D Lidar extrinsic calibration tool (Tested on ROS Humble)<br/>
Dependency: NumPy, Matplotlib, OpenCV (>= 3.3), pyquaternion, PyYaml

### tldr
```
colcon build
source install/setup.bash
```

1. Select points to match
```
ros2 launch camera_2d_lidar_calibration collect_camera_lidar_data.launch.py
```

2. 
```
ros2 launch camera_2d_lidar_calibration calibration.launch.py
```

3. 
```
ros2 launch camera_2d_lidar_calibration reprojection.launch.py
```