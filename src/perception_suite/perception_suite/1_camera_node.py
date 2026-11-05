"""
Stage 1, camera source: publishes raw_frames from the GMSL dashcam.

Interchangeable with 1_video_node, so stages 2 onwards cannot tell which
source they are fed by. Capture runs on its own thread and the timer publishes
the newest frame, so a slow consumer drops frames rather than queueing them.
"""

import signal
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    """
    Stage 1: the GMSL dashcam, publishing /raw_frames.

    Interchangeable with 1_video_node: same topic, same header, same downscale,
    so stages 2-6 cannot tell which source they are fed by.
    """

    def __init__(self):
        """Open the camera, discard the warm-up frames, and start publishing."""
        super().__init__("camera_node")
        # pipeline.launch.py passes every source argument through
        # LaunchConfiguration.perform(), which yields strings, so these are
        # declared as strings and converted here. A float or int default would
        # make the node reject the launch's value at startup.
        index = int(self.declare_parameter("camera_index", "2").value)
        fps = float(self.declare_parameter("camera_fps", "30.0").value)
        # What the sensor is asked to deliver, before the downscale below.
        capture_width = int(self.declare_parameter("capture_width", "1920").value)
        capture_height = int(self.declare_parameter("capture_height", "1080").value)
        # Downscaled for DDS bandwidth only; the working size is set by the
        # preprocess stage. Aspect is kept so the camera model stays square-pixel.
        self.width = int(self.declare_parameter("width", "1280").value)

        # The sensor's pixel format. NOT negotiable by guesswork: this device
        # advertises UYVY first, NV16 second, and OpenCV's own V4L2 backend
        # decodes NONE of them correctly. Measured on the bench carrier:
        #   UYVY  diagonal shearing, every row offset from the one above
        #   YUYV  (what OpenCV picks by default) top third of the image, the
        #         rest solid green fill
        #   NV16  via OpenCV's V4L2 backend: black, with one grey band
        # Only NV16 pulled through a GStreamer pipeline produces a correct
        # frame, which is why this no longer uses cv2.VideoCapture(index).
        pixel_format = str(self.declare_parameter("pixel_format", "NV16").value)
        # A freshly started stream carries torn rows that fade over several
        # seconds rather than clearing at once. Measured on the bench carrier by
        # counting green-dominant pixels per frame: 11.6% at frame 0, 61.6% at
        # frame 1, 0.9% at frame 5, 0.1% at frame 19, and the first frame with
        # none at all is frame 120. An earlier value of 5 came with a comment
        # claiming the stream was clean by then; it is not, and the residue is
        # visible on the dashboard for the first seconds of a camera run.
        # 120 frames is 4 seconds at 30 FPS, which is short against the engine
        # load that precedes it.
        warmup = int(self.declare_parameter("warmup_frames", "120").value)
        # Some mountings present the sensor inverted. Left off by default: a
        # silent 180 degree rotation would put the road at the top of the frame
        # and the ground projection would be confidently wrong rather than
        # obviously broken. Our bench carrier needs rotate_180:=true.
        self.rotate_180 = str(
            self.declare_parameter("rotate_180", "false").value
        ).strip().lower() in ("1", "true", "yes")

        self.publisher_ = self.create_publisher(Image, "raw_frames", 10)
        pipeline = (
            f"v4l2src device=/dev/video{index} ! "
            f"video/x-raw,format={pixel_format},"
            f"width={capture_width},height={capture_height} ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera index {index} (/dev/video{index}) as "
                f"{pixel_format} at {capture_width}x{capture_height}. "
                "The GMSL camera needs nv_nru2mp loaded AND the deserializer "
                "link trained. The link is trained at driver probe, and this "
                "hardware does not hot-plug: a camera connected after boot is "
                "not seen until the board is rebooted. The symptom is a device "
                "that opens cleanly and streams buffers of entirely zero bytes."
            )

        for _ in range(max(warmup, 0)):
            self.cap.read()

        ok, probe = self.cap.read()
        if not ok or probe is None:
            raise RuntimeError(
                f"Camera {index} opened but delivered no frame after "
                f"{warmup} warm-up reads."
            )
        actual_h, actual_w = probe.shape[:2]
        if (actual_w, actual_h) != (capture_width, capture_height):
            # Not fatal: the driver is free to pick the nearest supported mode,
            # but the aspect it lands on drives the camera model, so say so.
            self.get_logger().warn(
                f"Camera delivered {actual_w}x{actual_h}, not the requested "
                f"{capture_width}x{capture_height}"
            )
        self.get_logger().info(
            f"Camera {index} at {actual_w}x{actual_h} {pixel_format}, publishing "
            f"{self.width} wide at {fps:g} FPS"
            f"{', rotated 180' if self.rotate_180 else ''}"
        )

        # INTER_AREA is an area average and costs about 4x INTER_LINEAR on this
        # board (10.9 ms vs 2.8 ms for a 4K source, measured). The source is
        # already being downscaled for bandwidth, not for analysis, and the
        # preprocess stage resizes again anyway, so linear is enough here.
        self.interpolation = cv2.INTER_LINEAR

        self.bridge = CvBridge()
        # cap.read() blocks until the sensor has a frame, so doing it on the
        # timer serialises the wait with the resize and publish: measured
        # 21.3 Hz published from a 30 Hz sensor. Capture runs on its own thread
        # and the timer publishes whatever the latest frame is.
        self._lock = threading.Lock()
        self._latest = None
        self._running = True
        self._reader = threading.Thread(target=self._capture_loop, daemon=True)
        self._reader.start()
        self.timer = self.create_timer(1.0 / max(fps, 0.1), self.timer_callback)

    def _capture_loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                # A live camera can drop a frame without the session being over.
                time.sleep(0.05)
                continue
            if self.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            with self._lock:
                self._latest = frame

    def timer_callback(self):
        """Publish the newest captured frame, if the reader has produced one."""
        with self._lock:
            frame = self._latest
            self._latest = None
        if frame is None:
            # Nothing new since the last tick: either the capture thread is
            # still waiting on the sensor, or the camera has stopped.
            self.get_logger().warn("No new camera frame", throttle_duration_sec=5.0)
            return
        h, w = frame.shape[:2]
        height = round(h * self.width / w)
        small_frame = (frame if (w, h) == (self.width, height)
                       else cv2.resize(frame, (self.width, height),
                                       interpolation=self.interpolation))

        msg = self.bridge.cv2_to_imgmsg(small_frame, encoding="bgr8")
        # Every downstream stage copies this header, which is how the
        # visualizer matches a frame to its mask and path.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.publisher_.publish(msg)

    def destroy_node(self):
        """Stop the reader thread and release the camera before shutting down."""
        self._running = False
        reader = getattr(self, "_reader", None)
        if reader is not None:
            reader.join(timeout=2.0)
        if getattr(self, "cap", None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Run the camera source until shutdown."""
    rclpy.init(args=args)
    try:
        rclpy.spin(CameraPublisher())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under launch, Ctrl+C arrives twice (terminal and launch); the second
        # must not interrupt teardown or interpreter exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
