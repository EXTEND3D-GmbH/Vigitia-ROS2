__author__ = "Chris Brammer"
__copyright__ = "Copyright (C) 2022 EXTEND3D and/or its subsidiary(-ies)"
__credits__ = ["Chris Brammer"]
__license__ = "LGPL"
__version__ = "0.1.0"

import sys
import numpy as np
import cv2
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image

class ProjectorDriver(Node):
    

    def __init__(self):
        super().__init__('projector_driver')
        self.subscription = self.create_subscription(
            Image,
            '/projector',
            self.listener_callback,
            10)
        self.title = self.get_name() + " " + self.subscription.topic_name

        dx_descriptor = ParameterDescriptor(description="x-Offset in operating system screen space. Choose any projector pixel's X")
        dy_descriptor = ParameterDescriptor(description="y-Offset in operating system screen space. Choose any projector pixel's X")
        dx: Parameter = self.declare_parameter("window_origin.x", 0, dx_descriptor)
        dy: Parameter = self.declare_parameter("window_origin.y", 0, dy_descriptor)

        cv2.namedWindow(self.title, cv2.WND_PROP_FULLSCREEN)
        cv2.moveWindow(self.title, dx.value, dy.value)
        cv2.setWindowProperty(self.title , cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        self.create_timer(0.1, self.timer_callback)
        
        
    def timer_callback(self):
        cv2.waitKey(1)
        

    def listener_callback(self, data):
        self.get_logger().info("Image Callback")
        img = self.imgmsg_to_cv2(data)
        cv2.imshow(self.title ,img)

    def parameter_callback(self, params):
        self.get_logger().warn("Parameters are ignored after contructor!")

    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8")  # Hardcode to 8 bits
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        # Since OpenCV works with bgr natively, we don't need to reorder the channels.
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 1), dtype=dtype, buffer=img_msg.data)
        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        return image_opencv

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    projector = ProjectorDriver()

    # Spin the node so the callback function is called.
    rclpy.spin(projector)

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()