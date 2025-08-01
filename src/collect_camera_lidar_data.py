#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import cv2

u = None
v = None

x = None
y = None

alpha = 1.0
beta = 0

clicked = False
fig = None

sub_img = np.array([])

class Matplot:

    def __init__(self, output_file):
        global alpha, beta
        alpha = 1.0
        beta = 0
        self.output_file = output_file
        self.create_figure()
            
    def create_figure(self):
        global fig
        if sub_img.size == 0:
            print('No image obtained.')
            return
        self.img = sub_img
        self.img_ori = sub_img
        self.fig = plt.figure()
        self.fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
        ax = self.fig.add_subplot(111)
        self.image = ax.imshow(self.img)
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.fig.canvas.mpl_connect('key_press_event', self.onkey)
        fig = self.fig
        plt.show()
        
    def onclick(self, event):
        global u, v
        if event.xdata != None and event.ydata != None and event.button == 3:
            u = int(event.xdata)
            v = int(event.ydata)
            img_temp = np.copy(self.img)
            cv2.circle(img_temp, (u, v), 1, (255,0,0), 1)
            self.image.set_data(img_temp)
            plt.draw()

    def onkey(self, event):
        global u, v, clicked, alpha, beta
        if event.key == 'enter':
            if u == None and v == None:
                print('No point selected. Try again.')
            else:
                print('Image coordinate: (%d, %d), Laser coordinate: (%f, %f)' % (u, v, x, y))
                plt.close()
                with open(self.output_file, 'a') as f:
                    f.write('%f %f %d %d\n' % (x, y, u, v))
                u, v = None, None
        elif event.key in ['up','down','pageup','pagedown']:
            if event.key == 'up':
                alpha += 0.1
            if event.key == 'down':
                alpha -= 0.1
            if event.key == 'pageup':
                beta += 10
            if event.key == 'pagedown':
                beta -= 10
            self.img = cv2.addWeighted(self.img_ori, alpha, self.img_ori, 0, beta)
            self.image.set_data(self.img)
            self.fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
            plt.draw()

def point_cb(msg):
    global x, y, clicked, fig
    x = msg.pose.position.x
    y = msg.pose.position.y
    if clicked and plt.get_fignums():
        fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
        plt.draw()
    else:
        pass
    clicked = True

def image_cb(msg):
    global sub_img
    sub_img_distorted = bridge.imgmsg_to_cv2(msg)
    sub_img_distorted = cv2.cvtColor(sub_img_distorted, cv2.COLOR_BGR2RGB)
    if lens == 'pinhole':
        sub_img = cv2.undistort(sub_img_distorted, K, D, newCameraMatrix = K)
    elif lens == 'fisheye':
        sub_img = cv2.fisheye.undistortImage(sub_img_distorted, K, D, Knew = K)

def Shutdown():
    plt.close()

class CollectCameraLidarDataNode(Node):
    def __init__(self):
        super().__init__('collect_camera_lidar_data')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_file', 'data/data.txt')
        
        # Declare and get parameters for camera calibration
        self.declare_parameter('lens', 'pinhole')
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        global bridge
        bridge = CvBridge()
        global lens, K, D
        
        lens = self.get_parameter('lens').get_parameter_value().string_value
        fx = self.get_parameter('fx').get_parameter_value().double_value
        fy = self.get_parameter('fy').get_parameter_value().double_value
        cx = self.get_parameter('cx').get_parameter_value().double_value
        cy = self.get_parameter('cy').get_parameter_value().double_value
        k1 = self.get_parameter('k1').get_parameter_value().double_value
        k2 = self.get_parameter('k2').get_parameter_value().double_value
        p1 = self.get_parameter('p1').get_parameter_value().double_value
        p2 = self.get_parameter('p2').get_parameter_value().double_value
        
        # Re-initialize the K and D matrices with the retrieved parameters
        if lens not in ['pinhole', 'fisheye']:
            self.get_logger().info('Invalid lens, using pinhole as default.')
            lens = 'pinhole'
        
        K = np.matrix([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        D = np.array([k1, k2, p1, p2])
        
        self.get_logger().info("Camera parameters")
        self.get_logger().info("Lens = %s" % lens)
        self.get_logger().info("K =\n%s" % K)
        self.get_logger().info("D = %s" % D)

        self.sub1 = self.create_subscription(PoseStamped, '/goal_pose', point_cb, 10)
        self.sub2 = self.create_subscription(Image, image_topic, image_cb, 10)
        
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        global clicked
        if clicked:
            Matplot(self.output_file)
            clicked = False

def main(args=None):
    rclpy.init(args=args)
    node = CollectCameraLidarDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        Shutdown()

if __name__ == '__main__':
    main()
