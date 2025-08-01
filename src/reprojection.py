#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs_py.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber

def get_z(T_cam_world, T_world_pc, K):
    R = T_cam_world[:3,:3]
    t = T_cam_world[:3,3]
    proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
    xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
    xy_hom = np.dot(proj_mat, xyz_hom.T).T
    z = xy_hom[:, -1]
    z = np.asarray(z).squeeze()
    return z

def extract(point):
    return [point[0], point[1], point[2]]

class ReprojectionNode(Node):
    def __init__(self):
        super().__init__('reprojection')
        
        # Declare basic parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/zed_node/left_raw/image_raw_color')
        self.declare_parameter('calib_file', 'data/calibration_result.txt')
        self.declare_parameter('laser_point_radius', 3)
        self.declare_parameter('time_diff', 1.0)
        
        # Declare camera parameters that can be loaded from YAML
        self.declare_parameter('lens', 'pinhole')
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)  # Note: this matches your YAML key p1/k3
        self.declare_parameter('p2', 0.0)  # Note: this matches your YAML key p2/k4
        
        # Get parameters
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        calib_file = self.get_parameter('calib_file').get_parameter_value().string_value
        self.laser_point_radius = self.get_parameter('laser_point_radius').get_parameter_value().integer_value
        time_diff = self.get_parameter('time_diff').get_parameter_value().double_value
        
        # Get camera parameters (these will be loaded from YAML via launch file)
        self.lens = self.get_parameter('lens').get_parameter_value().string_value
        fx = self.get_parameter('fx').get_parameter_value().double_value
        fy = self.get_parameter('fy').get_parameter_value().double_value
        cx = self.get_parameter('cx').get_parameter_value().double_value
        cy = self.get_parameter('cy').get_parameter_value().double_value
        k1 = self.get_parameter('k1').get_parameter_value().double_value
        k2 = self.get_parameter('k2').get_parameter_value().double_value
        p1 = self.get_parameter('p1').get_parameter_value().double_value
        p2 = self.get_parameter('p2').get_parameter_value().double_value
        
        self.bridge = CvBridge()
        self.lp = lg.LaserProjection()
        
        # Load calibration from file
        try:
            with open(calib_file, 'r') as f:
                data = f.read().split()
                qx = float(data[0])
                qy = float(data[1])
                qz = float(data[2])
                qw = float(data[3])
                tx = float(data[4])
                ty = float(data[5])
                tz = float(data[6])
        except FileNotFoundError:
            self.get_logger().error(f"Calibration file not found: {calib_file}")
            raise
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Error parsing calibration file: {e}")
            raise
        
        self.q = Quaternion(qw, qx, qy, qz).transformation_matrix
        self.q[0, 3] = tx
        self.q[1, 3] = ty
        self.q[2, 3] = tz
        
        self.get_logger().info("Extrinsic parameter - camera to laser")
        self.get_logger().info(f"\n{self.q}")
        
        self.tvec = self.q[:3, 3]
        rot_mat = self.q[:3, :3]
        self.rvec, _ = cv2.Rodrigues(rot_mat)
        
        # Set up camera intrinsics from parameters
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.D = np.array([k1, k2, p1, p2])
        
        self.get_logger().info("Camera parameters")
        self.get_logger().info(f"Lens = {self.lens}")
        self.get_logger().info(f"K =\n{self.K}")
        self.get_logger().info(f"D = {self.D}")
        
        # Validate camera parameters
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn("Camera focal lengths are zero - check your parameter file!")
        
        self.pub = self.create_publisher(Image, '/reprojection', 1)
        self.scan_sub = Subscriber(self, LaserScan, scan_topic)
        self.image_sub = Subscriber(self, Image, image_topic)
        self.ts = ApproximateTimeSynchronizer([self.scan_sub, self.image_sub], 10, time_diff)
        self.ts.registerCallback(self.callback)

    def callback(self, scan, image):
        self.get_logger().info(f"image timestamp: {image.header.stamp.sec}.{image.header.stamp.nanosec} ns")
        self.get_logger().info(f"scan timestamp: {scan.header.stamp.sec}.{scan.header.stamp.nanosec} ns")
        diff = abs((image.header.stamp.sec*1e9 + image.header.stamp.nanosec) - (scan.header.stamp.sec*1e9 + scan.header.stamp.nanosec))
        self.get_logger().info(f"diff: {diff} ns")
        
        img = self.bridge.imgmsg_to_cv2(image)
        cloud = self.lp.projectLaser(scan)
        points = pc2.read_points(cloud)
        objPoints = np.array(list(map(extract, points)))
        
        Z = get_z(self.q, objPoints, self.K)
        objPoints = objPoints[Z > 0]
        
        if len(objPoints) == 0:
            self.get_logger().warn("No valid points after Z filtering")
            self.pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='rgb8'))
            return
        
        if self.lens == 'pinhole':
            img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        elif self.lens == 'fisheye':
            objPoints = np.reshape(objPoints, (1, objPoints.shape[0], objPoints.shape[1]))
            img_points, _ = cv2.fisheye.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        
        img_points = np.squeeze(img_points)
        
        # Handle case where img_points is a single point (becomes 1D array after squeeze)
        if img_points.ndim == 1:
            img_points = img_points.reshape(1, -1)
        
        # Get image dimensions for bounds checking
        img_height, img_width = img.shape[:2]
        
        for i in range(len(img_points)):
            try:
                # Extract x, y coordinates
                x = img_points[i][0]
                y = img_points[i][1]
                
                # Check for NaN or infinity values
                if not (np.isfinite(x) and np.isfinite(y)):
                    continue
                
                # Convert to integers
                x_int = int(round(float(x)))
                y_int = int(round(float(y)))
                
                # Check if point is within image bounds
                if 0 <= x_int < img_width and 0 <= y_int < img_height:
                    cv2.circle(img, (x_int, y_int), self.laser_point_radius, (0, 255, 0), 1)
                
            except (OverflowError, ValueError, TypeError) as e:
                self.get_logger().debug(f"Skipping point {i}: {e}")
                continue
        
        self.pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='8UC4'))

def main(args=None):
    rclpy.init(args=args)
    node = ReprojectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
