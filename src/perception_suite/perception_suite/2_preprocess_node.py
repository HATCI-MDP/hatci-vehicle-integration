"""
Stage 2: raw_frames in, preprocessed_frames out.

Resize to the working size and apply the configured enhancement, through
offroad_autonomy's ImagePreprocessor. When the config has an enabled
undistortion section, the lens distortion is then removed (see undistort.py).
The header is carried over unchanged so every later stage can measure its own
age against the capture stamp.
"""

import signal

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from offroad_autonomy.perception.camera_geometry import CameraModel
from offroad_autonomy.preprocessing.image_preprocessor import ImagePreprocessor

from perception_suite import settings, undistort
from perception_suite.common import LATEST_ONLY, StageTimer, load_pipeline_config


class PreprocessNode(Node):
    """Stage 2: resize to the working size and apply CLAHE (ImagePreprocessor)."""

    def __init__(self):
        """Build the preprocessor and subscribe to raw_frames."""
        super().__init__('preprocess_node')
        config = load_pipeline_config(self)
        self.preprocessor = ImagePreprocessor(config)
        self.undistort_maps = None
        self.size = (int(config.preprocess_width), int(config.preprocess_height))
        self._size_warned = False
        self._setup_undistortion(config)
        self.bridge = CvBridge()
        self.stage_timer = StageTimer(self, 'preprocess')
        self.publisher = self.create_publisher(Image, 'preprocessed_frames', 10)
        self.subscription = self.create_subscription(
            Image, 'raw_frames', self.on_frame, LATEST_ONLY)

    def _setup_undistortion(self, config):
        """Build the remap tables once, onto the same pinhole model every stage uses."""
        path = settings.resolve(self.get_parameter('config').value)
        calibration = undistort.load_calibration(path)
        if calibration is None:
            self.get_logger().info('Undistortion off (no enabled undistortion section)')
            return
        camera = CameraModel(config.camera, *self.size)
        map1, map2, outside = undistort.build_maps(
            calibration, self.size, camera.focal_px, camera.cx, camera.cy)
        self.undistort_maps = (map1, map2)
        self.get_logger().info(
            f'Undistortion on: calibration at {calibration.native_size[0]}x'
            f'{calibration.native_size[1]}, output {self.size[0]}x{self.size[1]} '
            f'at fov_h {config.camera.fov_x_deg:g} deg (focal {camera.focal_px:.1f} px)')
        if outside > 0.0:
            widest = undistort.widest_fov_deg(calibration, self.size, camera.cx, camera.cy)
            self.get_logger().warn(
                f'{outside:.1%} of each undistorted frame lies outside the lens image and '
                f'will be black. Set beamng.camera.sensor.fov_h to {widest:.2f} or less.')

    def on_frame(self, msg):
        """Preprocess one frame and republish it."""
        t0 = self.stage_timer.start()
        packet = self.preprocessor.process(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        frame = packet.preprocessed
        if self.undistort_maps is not None:
            if (frame.shape[1], frame.shape[0]) == self.size:
                frame = cv2.remap(frame, *self.undistort_maps, cv2.INTER_LINEAR)
            elif not self._size_warned:
                self._size_warned = True
                self.get_logger().error(
                    f'Frame is {frame.shape[1]}x{frame.shape[0]}, not the working size '
                    f'{self.size[0]}x{self.size[1]}; passing it through without undistortion')
        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.publisher.publish(out)
        self.stage_timer.done(t0, msg.header)


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    try:
        rclpy.spin(PreprocessNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under launch, Ctrl+C arrives twice (terminal and launch); the second
        # must not interrupt teardown or interpreter exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
