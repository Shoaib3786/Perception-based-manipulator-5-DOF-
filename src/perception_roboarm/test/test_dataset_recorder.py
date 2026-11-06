"""Check capture persistence without starting a ROS node or simulation."""
import cv2
import numpy as np
import pytest

from perception_roboarm.dataset_recorder import next_index, save_jpeg


def test_per_camera_resume(tmp_path):
    (tmp_path / 'camera1_000002.jpg').touch()
    (tmp_path / 'camera1_15.jpg').touch()
    (tmp_path / 'camera2_000099.jpg').touch()
    (tmp_path / 'camera1_bad.jpg').touch()
    assert next_index(tmp_path, 'camera1') == 16
    assert next_index(tmp_path, 'camera2') == 100
    assert next_index(tmp_path, 'camera3') == 1


def test_valid_jpeg_and_no_overwrite(tmp_path):
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    image[:, :, 2] = 240
    path = save_jpeg(tmp_path, 'camera1', 1, image)
    assert path.name == 'camera1_000001.jpg'
    decoded = cv2.imread(str(path))
    assert decoded.shape == image.shape
    assert decoded[:, :, 2].mean() > 230
    original = path.read_bytes()
    with pytest.raises(FileExistsError):
        save_jpeg(tmp_path, 'camera1', 1, np.zeros_like(image))
    assert path.read_bytes() == original
    assert next_index(tmp_path, 'camera1') == 2


def test_claw_camera_capture_and_resume(tmp_path):
    from types import SimpleNamespace
    from cv_bridge import CvBridge
    from perception_roboarm.dataset_recorder import CAMERA_TOPICS, DatasetRecorder

    assert CAMERA_TOPICS['camera5'] == '/camera/color/image_raw'
    image = np.full((32, 48, 3), 100, dtype=np.uint8)
    save_jpeg(tmp_path, 'camera5', 7, image)
    bridge = CvBridge()
    counts = {camera: next_index(tmp_path, camera) for camera in CAMERA_TOPICS}
    recorder = SimpleNamespace(directory=tmp_path, quality=95, bridge=bridge,
                               counts=counts, saved={camera: 0 for camera in counts})
    message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
    DatasetRecorder.capture(recorder, message, 'camera5')
    assert cv2.imread(str(tmp_path / 'camera5_000008.jpg')).shape == image.shape
    assert message.width == 48 and message.height == 32
    assert recorder.counts['camera5'] == 9
    assert recorder.saved['camera5'] == 1
    assert all(recorder.counts[f'camera{i}'] == 1 for i in range(1, 5))
    assert not list(tmp_path.glob('camera1_*.jpg'))


@pytest.mark.parametrize('camera', ['camera1', 'camera2', 'camera3', 'camera4', 'camera5'])
def test_common_capture_size(tmp_path, camera):
    from types import SimpleNamespace
    from cv_bridge import CvBridge
    from perception_roboarm.dataset_recorder import DatasetRecorder
    image = np.full((72, 128, 3), 180, dtype=np.uint8)
    bridge = CvBridge()
    recorder = SimpleNamespace(directory=tmp_path, quality=95, bridge=bridge,
                               counts={camera: 1}, saved={camera: 0})
    message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
    DatasetRecorder.capture(recorder, message, camera)
    decoded = cv2.imread(str(tmp_path / f'{camera}_000001.jpg'))
    assert decoded.shape == (72, 128, 3)
    assert decoded[:, 64].mean() > 170
    assert message.width == 128 and message.height == 72
