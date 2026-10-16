import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # use according to the ros2_gz_bridge config file you created
from sensor_msgs.msg import Image, CameraInfo
from perception_interfaces.msg import Segment2D, Segmentation2D
# from message_filters import Subscriber, ApproximateTimeSynchronizer 
from message_filters import Subscriber, TimeSynchronizer # sync-callback for (detection_result + depth_image)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class DepthDetectionNode(Node):
    def __init__(self):
        super().__init__('depth_detection_node')

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detection_result_topic', '/perception/segmentation_2d')
        self.declare_parameter('caliberation_topic', '/camera/color/camera_info')
        self.depth_topic = self.get_parameter('depth_topic').value
        self.detection_result_topic = self.get_parameter('detection_result_topic').value
        self.caliberation_topic = self.get_parameter('caliberation_topic').value

        self.bridge = CvBridge()

        # None means calibration has not arrived yet.
        self.camera_info = None
        self.fx = self.fy = self.cx = self.cy = None

        self.object_clouds = []
        self.points_header = None


        """
        For synchronized subscribers -- different way to write subscriber
        The difference is:
        # Ordinary subscription: self is already supplied through self.method().
        self.create_subscription(
            MessageType, topic, callback, qos_profile
        )

        # Filter subscriber: explicitly pass the node to its constructor.
        Subscriber(
            self, MessageType, topic, qos_profile=qos_profile_sensor_data
        )
        """
        self.depth_subscriber = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data
        )

        self.detection_result_subscriber = Subscriber(
            self,
            Segmentation2D,
            self.detection_result_topic,
            qos_profile=qos_profile_sensor_data
        )

        self.caliberation_subscriber = self.create_subscription(
            CameraInfo,
            self.caliberation_topic,
            self.calibration_callback,
            qos_profile_sensor_data
        )

        self.cloud_publishers = {
            color: self.create_publisher(
                PointCloud2,
                f'/perception/{color}/points',
                qos_profile_sensor_data
            )
            for color in ['red', 'pink', 'green']
        }

        """
        Cann't use same call back for all subscriber, Hence:
        1. One callback for camera caliberation
        2. Another callback for both detection_result and depth_image it should be 
        synchronized callback so that the detection_result timestamp must match with
        depth_image timestamp.
        """
        self.synchronizer = TimeSynchronizer(
            [self.depth_subscriber,self.detection_result_subscriber],
            queue_size=10,
            
        )

        self.synchronizer.registerCallback(
            self.synchronized_callback
        )

    def publish_object_clouds(self, object_clouds, header):
        """Publish all detected points grouped by color."""

        for color, publisher in self.cloud_publishers.items():
            arrays = [
                obj['points_xyz']
                for obj in object_clouds
                if obj['color'] == color
            ]

            if arrays:
                points = np.concatenate(arrays, axis=0)
                points = points.astype(np.float32, copy=False)
            else:
                # Publish an empty cloud when this color disappears.
                points = np.empty((0, 3), dtype=np.float32)

            cloud_msg = point_cloud2.create_cloud_xyz32(
                header,
                points
            )

            publisher.publish(cloud_msg)


    def extract_segmentation(self, depth_msg, segmentation_msg):
        object_clouds = []
        # Convert the depth message without changing its numeric encoding.
        depth = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding='passthrough'
        )

        if depth_msg.encoding == '32FC1':
            depth_m = depth.astype(np.float32, copy=False)
        elif depth_msg.encoding == '16UC1':
            depth_m = depth.astype(np.float32) * 0.001
        else:
            raise ValueError(
                f"Unsupported depth encoding: {depth_msg.encoding}"
            )

        for detection in segmentation_msg.detections:
            object_mask = self.bridge.imgmsg_to_cv2(
                detection.mask,
                desired_encoding='mono8'
            )

            if object_mask.shape != depth_m.shape:
                raise ValueError("Mask and aligned depth dimensions differ")

            # Coordinates of this object's pixels.
            v, u = np.nonzero(object_mask)

            # Depth at those exact pixel locations.
            z = depth_m[v, u]

            # Discard unavailable/invalid depth measurements.
            valid = np.isfinite(z) & (z > 0)

            u = u[valid]
            v = v[valid]
            z = z[valid]

            if z.size == 0:
                continue

            # Intrinsics must describe this aligned image geometry.
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            points_xyz = np.column_stack((x, y, z))

            object_clouds.append({
                'name': detection.name,
                'color': detection.color,
                'points_xyz': points_xyz,
            })

        return object_clouds

    def calibration_callback(self, msg):
        """Store calibration for use when an image pair arrives."""
        fx = float(msg.k[0])
        fy = float(msg.k[4])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(
                'Received invalid or uncalibrated CameraInfo.'
            )
            return

        self.fx = fx
        self.fy = fy
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        self.camera_info = msg

    def synchronized_callback(self, depth_msg, segmentation_msg):
        """Called when the synchronizer finds a matching pair."""
        if self.camera_info is None:
            self.get_logger().warning(
                'Waiting for camera calibration.',
                throttle_duration_sec=5.0
            )
            return

        try:
            self.object_clouds = self.extract_segmentation(
                depth_msg,
                segmentation_msg
            )
        except (CvBridgeError, ValueError) as exc:
            self.get_logger().error(
                f'Depth processing failed: {exc}'
            )
            return

        # Assumes depth is registered to the color-image geometry.
        self.points_header = segmentation_msg.header

        self.publish_object_clouds(
            self.object_clouds,
            self.points_header
        )

        self.get_logger().info(
            f'Computed point clouds for '
            f'{len(self.object_clouds)} detected regions.',
            throttle_duration_sec=1.0
        )



def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DepthDetectionNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()