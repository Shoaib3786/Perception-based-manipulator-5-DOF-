"""Save each received table- and claw-camera frame as a uniquely numbered JPEG."""
import fcntl
from pathlib import Path
import re

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CAMERA_TOPICS = {
    **{f'camera{i}': f'/camera{i}/image_raw' for i in range(1, 5)},
    'camera5': '/camera/color/image_raw',
}


def next_index(directory, camera):
    """Resume after the highest existing number, including non-padded filenames."""
    pattern = re.compile(rf'{re.escape(camera)}_(\d+)\.jpg')
    return 1 + max((int(match.group(1)) for p in directory.iterdir()
                    if (match := pattern.fullmatch(p.name))), default=0)


def save_jpeg(directory, camera, index, frame, quality=95):
    """Create a JPEG exclusively: never overwrite an earlier capture."""
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('JPEG encoding failed')
    path = directory / f'{camera}_{index:06d}.jpg'
    with path.open('xb') as stream:
        try:
            stream.write(encoded.tobytes())
        except BaseException:
            path.unlink(missing_ok=True)
            raise
    return path



class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        default = Path.home() / 'ros2_projects' / 'roboArm_rosws' / 'dataset'
        self.declare_parameter('output_dir', str(default))
        self.declare_parameter('jpeg_quality', 95)
        self.directory = Path(self.get_parameter('output_dir').value).expanduser().resolve()
        self.quality = int(self.get_parameter('jpeg_quality').value)
        if not 1 <= self.quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100')
        self.directory.mkdir(parents=True, exist_ok=True)
        # Keep the lock file in place: unlinking it can allow two simultaneous locks.
        self.lock = (self.directory / '.recorder.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            raise RuntimeError(f'Another recorder already owns {self.directory}')
        self.bridge = CvBridge()
        self.counts = {camera: next_index(self.directory, camera)
                       for camera in CAMERA_TOPICS}
        self.saved = {name: 0 for name in self.counts}
        self.subscriptions_by_camera = [self.create_subscription(
            Image, CAMERA_TOPICS[camera],
            lambda msg, name=camera: self.capture(msg, name), qos_profile_sensor_data)
            for camera in self.counts]
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'Saving received camera1..camera5 frames at their sensor resolution to {self.directory}')

    def capture(self, message, camera):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            # Skip an occupied name if files were added externally after startup.
            while True:
                try:
                    save_jpeg(self.directory, camera, self.counts[camera], frame, self.quality)
                    break
                except FileExistsError:
                    self.counts[camera] += 1
            self.counts[camera] += 1
            self.saved[camera] += 1
        except Exception as error:
            self.get_logger().error(f'Capture failed for {camera}: {error}')
            raise  # Stop explicitly instead of silently discarding a broken capture stream.

    def report(self):
        self.get_logger().info('Frames saved this run: ' + ', '.join(
            f'{camera}={count}' for camera, count in self.saved.items()))

    def destroy_node(self):
        try:
            return super().destroy_node()
        finally:
            if hasattr(self, 'lock'):
                self.lock.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DatasetRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.report()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
