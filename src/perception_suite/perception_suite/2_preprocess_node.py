"""
Stage 2: raw_frames in, preprocessed_frames out.

Resize to the working size and apply the configured enhancement, through
offroad_autonomy's ImagePreprocessor. The header is carried over unchanged so
every later stage can measure its own age against the capture stamp.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from offroad_autonomy.preprocessing.image_preprocessor import ImagePreprocessor

from perception_suite.common import LATEST_ONLY, StageTimer, load_pipeline_config


class PreprocessNode(Node):
    """Stage 2: resize to the working size and apply CLAHE (ImagePreprocessor)."""

    def __init__(self):
        """Build the preprocessor and subscribe to raw_frames."""
        super().__init__('preprocess_node')
        config = load_pipeline_config(self)
        self.preprocessor = ImagePreprocessor(config)
        self.bridge = CvBridge()
        self.stage_timer = StageTimer(self, 'preprocess')
        self.publisher = self.create_publisher(Image, 'preprocessed_frames', 10)
        self.subscription = self.create_subscription(
            Image, 'raw_frames', self.on_frame, LATEST_ONLY)

    def on_frame(self, msg):
        """Preprocess one frame and republish it."""
        t0 = self.stage_timer.start()
        packet = self.preprocessor.process(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        out = self.bridge.cv2_to_imgmsg(packet.preprocessed, 'bgr8')
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
