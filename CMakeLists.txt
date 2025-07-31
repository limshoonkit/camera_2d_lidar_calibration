cmake_minimum_required(VERSION 3.5)
project(camera_2d_lidar_calibration)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

install(
  PROGRAMS
    src/camera_lidar_calibration.py
    src/collect_camera_lidar_data.py
    src/reprojection.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
