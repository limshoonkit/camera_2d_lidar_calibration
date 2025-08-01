#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pyquaternion import Quaternion
import math
import os

class CameraLidarCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_calibration')
        
        # Declare parameters
        self.declare_parameter('data_file', 'data/data.txt')
        self.declare_parameter('result_file', 'data/calibration_result.txt')
        
        # Declare camera parameters that can be loaded from YAML
        self.declare_parameter('lens', 'pinhole')
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        
        # Get parameters
        data_file = self.get_parameter('data_file').get_parameter_value().string_value
        result_file = self.get_parameter('result_file').get_parameter_value().string_value
        
        # Get camera parameters
        self.lens = self.get_parameter('lens').get_parameter_value().string_value
        fx = self.get_parameter('fx').get_parameter_value().double_value
        fy = self.get_parameter('fy').get_parameter_value().double_value
        cx = self.get_parameter('cx').get_parameter_value().double_value
        cy = self.get_parameter('cy').get_parameter_value().double_value
        k1 = self.get_parameter('k1').get_parameter_value().double_value
        k2 = self.get_parameter('k2').get_parameter_value().double_value
        p1 = self.get_parameter('p1').get_parameter_value().double_value
        p2 = self.get_parameter('p2').get_parameter_value().double_value
        
        # Validate camera parameters
        if fx == 0.0 or fy == 0.0:
            self.get_logger().error("Camera focal lengths are zero - check your parameter file!")
            return
        
        # Set up camera matrices
        self.K = np.array([[fx, 0.0, cx],
                          [0.0, fy, cy],
                          [0.0, 0.0, 1.0]])
        self.D = np.array([k1, k2, p1, p2])
        
        self.get_logger().info("Camera parameters")
        self.get_logger().info(f"Lens = {self.lens}")
        self.get_logger().info(f"K =\n{self.K}")
        self.get_logger().info(f"D = {self.D}")
        
        # Perform calibration
        self.perform_calibration(data_file, result_file)
    
    def rmse(self, objp, imgp, rvec, tvec):
        """Calculate RMSE between projected and actual image points"""
        if self.lens == 'pinhole':
            predicted, _ = cv2.projectPoints(objp, rvec, tvec, self.K, self.D)
            predicted = cv2.undistortPoints(predicted, self.K, self.D, P=self.K)
        elif self.lens == 'fisheye':
            predicted, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, self.K, self.D)
            predicted = cv2.fisheye.undistortPoints(predicted, self.K, self.D, P=self.K)
        
        predicted = predicted.squeeze()
        imgp = imgp.squeeze()
        
        pix_serr = []
        for i in range(len(predicted)):
            xp = predicted[i, 0]
            yp = predicted[i, 1]
            xo = imgp[i, 0]
            yo = imgp[i, 1]
            pix_serr.append((xp - xo)**2 + (yp - yo)**2)
        
        ssum = sum(pix_serr)
        return math.sqrt(ssum / len(pix_serr))
    
    def perform_calibration(self, data_file, result_file):
        """Perform the camera-lidar calibration"""
        try:
            # Load correspondence data
            self.get_logger().info(f"Loading data from: {data_file}")
            with open(data_file, 'r') as f:
                imgp = []
                objp = []
                for line_num, line in enumerate(f, 1):
                    try:
                        data = line.strip().split()
                        if len(data) != 4:
                            self.get_logger().warn(f"Line {line_num}: Expected 4 values, got {len(data)}")
                            continue
                        objp.append([float(data[0]), float(data[1]), 0.0])
                        imgp.append([float(data[2]), float(data[3])])
                    except ValueError as e:
                        self.get_logger().error(f"Line {line_num}: Error parsing data - {e}")
                        continue
            
            if len(objp) == 0:
                self.get_logger().error("No valid data points found!")
                return
            
            self.get_logger().info(f"Loaded {len(objp)} correspondence points")
            
            # Convert to numpy arrays
            imgp = np.array([imgp], dtype=np.float32)
            objp = np.array([objp], dtype=np.float32)
            
            # Solve PnP problem without distortion first
            D_0 = np.array([0.0, 0.0, 0.0, 0.0])
            retval, rvec, tvec = cv2.solvePnP(objp, imgp, self.K, D_0, flags=cv2.SOLVEPNP_ITERATIVE)
            
            if not retval:
                self.get_logger().error("PnP solution failed!")
                return
            
            # Convert rotation vector to matrix and quaternion
            rmat, jac = cv2.Rodrigues(rvec)
            q = Quaternion(matrix=rmat)
            
            self.get_logger().info("Transform from camera to laser")
            self.get_logger().info(f"T = \n{tvec.flatten()}")
            self.get_logger().info(f"R = \n{rmat}")
            self.get_logger().info(f"Quaternion = {q}")
            
            # Calculate RMSE
            rmse_value = self.rmse(objp, imgp, rvec, tvec)
            self.get_logger().info(f"RMSE in pixel = {rmse_value:.6f}")
            
            # Save results
            self.get_logger().info(f"Saving results to: {result_file}")
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(result_file), exist_ok=True)
            
            with open(result_file, 'w') as f:
                f.write(f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f} {tvec[0][0]:.6f} {tvec[1][0]:.6f} {tvec[2][0]:.6f}")
            
            self.get_logger().info("Calibration completed successfully!")
            self.get_logger().info("Result output format: qx qy qz qw tx ty tz")
            
            # Log calibration quality
            if rmse_value < 1.0:
                self.get_logger().info("Calibration quality: EXCELLENT (RMSE < 1.0 pixel)")
            elif rmse_value < 2.0:
                self.get_logger().info("Calibration quality: GOOD (RMSE < 2.0 pixels)")
            elif rmse_value < 5.0:
                self.get_logger().warn("Calibration quality: FAIR (RMSE < 5.0 pixels)")
            else:
                self.get_logger().warn("Calibration quality: POOR (RMSE >= 5.0 pixels)")
            
        except FileNotFoundError:
            self.get_logger().error(f"Data file not found: {data_file}")
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarCalibrationNode()
    
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()