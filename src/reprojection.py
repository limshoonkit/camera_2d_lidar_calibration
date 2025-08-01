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
import sensor_msgs.point_cloud2 as pc2
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
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/zed_node/left_raw/image_raw_color')
        self.declare_parameter('calib_file', 'data/calibration_result.txt')
        self.declare_parameter('config_file', 'config/config.yaml')
        self.declare_parameter('laser_point_radius', 3)
        self.declare_parameter('time_diff', 1.0)
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        calib_file = self.get_parameter('calib_file').get_parameter_value().string_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.laser_point_radius = self.get_parameter('laser_point_radius').get_parameter_value().integer_value
        time_diff = self.get_parameter('time_diff').get_parameter_value().double_value
        self.bridge = CvBridge()
        self.lp = lg.LaserProjection()
        # Load calibration
        with open(calib_file, 'r') as f:
            data = f.read().split()
            qx = float(data[0])
            qy = float(data[1])
            qz = float(data[2])
            qw = float(data[3])
            tx = float(data[4])
            ty = float(data[5])
            tz = float(data[6])
        self.q = Quaternion(qw,qx,qy,qz).transformation_matrix
        self.q[0,3] = tx
        self.q[1,3] = ty
        self.q[2,3] = tz
        print("Extrinsic parameter - camera to laser")
        print(self.q)
        self.tvec = self.q[:3,3]
        rot_mat = self.q[:3,:3]
        self.rvec, _ = cv2.Rodrigues(rot_mat)
        # Load camera config
        with open(config_file, 'r') as f:
            f.readline()
            config = yaml.load(f, Loader=yaml.SafeLoader)
            self.lens = config['lens']
            fx = float(config['fx'])
            fy = float(config['fy'])
            cx = float(config['cx'])
            cy = float(config['cy'])
            k1 = float(config['k1'])
            k2 = float(config['k2'])
            p1 = float(config['p1/k3'])
            p2 = float(config['p2/k4'])
        self.K = np.matrix([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.D = np.array([k1, k2, p1, p2])
        print("Camera parameters")
        print("Lens = %s" % self.lens)
        print("K =")
        print(self.K)
        print("D =")
        print(self.D)
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
        if self.lens == 'pinhole':
            img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        elif self.lens == 'fisheye':
            objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
            img_points, _ = cv2.fisheye.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        img_points = np.squeeze(img_points)
        for i in range(len(img_points)):
            try:
                cv2.circle(img, (int(round(img_points[i][0])),int(round(img_points[i][1]))), self.laser_point_radius, (0,255,0), 1)
            except OverflowError:
                continue
        self.pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='rgb8'))

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
