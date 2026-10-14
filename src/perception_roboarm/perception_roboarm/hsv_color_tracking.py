import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # use msg type the topic uses when gazebo sends on the bridge to ros2
from rclpy.qos import qos_profile_sensor_data # because on the ros_bridge_config.yaml we have qos_profile as SENSOR_DATA
from cv_bridge import CvBridge, CvBridgeError # to get ros2 image and convert into openCV acceptable
import cv2
import numpy as np

class DisplayNode(Node):
    """A ROS2 Node that prints to the console periodically."""

    def __init__(self):
        super().__init__('hsv_tracking_node')
        self.bridge = CvBridge()
        self.display_subscriber = self.create_subscription(
            msg_type=Image,
            topic='/camera/color/image_raw', # topic name got from ros_gz_bridge_config.yaml file which we made in roboarm_urdf_description package
            callback=self.timer_callback,
            qos_profile=qos_profile_sensor_data)

    def timer_callback(self, msg):
        """Method that is periodically called by the timer."""
        try:
            """Real-time tracker values get it"""
            l_h = cv2.getTrackbarPos("L - H", "Trackbars")
            l_s = cv2.getTrackbarPos("L - S", "Trackbars")
            l_v = cv2.getTrackbarPos("L - V", "Trackbars")
            u_h = cv2.getTrackbarPos("U - H", "Trackbars")
            u_s = cv2.getTrackbarPos("U - S", "Trackbars")
            u_v = cv2.getTrackbarPos("U - V", "Trackbars")


            # Convert ROS Image msg → OpenCV (NumPy) array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_range = np.array([l_h, l_s, l_v])
            upper_range = np.array([u_h, u_s, u_v])
               
            mask = cv2.inRange(hsv_image, lower_range, upper_range)
           
            # Remove small isolated detections
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
           
  
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Example: display
        cv2.imshow('sensor_feed', cv_image) # getting feed from ros2 topic converted to cv2
        cv2.imshow('hsv_mask', mask)
        cv2.imshow('result', result)
        cv2.waitKey(1)

def nothing():
    pass    

def main(args=None):
    
    """create Real-time tracker"""
    cv2.namedWindow("Trackbars") # window name of the tracker
    cv2.createTrackbar("L - H", "Trackbars", 0, 360, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 360, 360, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

    try:
        rclpy.init(args=args)
        display_node = DisplayNode()
        rclpy.spin(display_node)
        display_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)