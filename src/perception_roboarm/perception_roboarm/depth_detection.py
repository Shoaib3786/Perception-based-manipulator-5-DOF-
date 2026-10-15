import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # use according to the ros2_gz_bridge config file you created
from sensor_msgs.msg import Image

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        self.depth_subscriber = self.create_subscription(
                    Image,
                    self.depth_topic,
                    self.image_callback,
                    qos_profile_sensor_data

                )