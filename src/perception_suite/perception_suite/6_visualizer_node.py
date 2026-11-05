"""
Stage 6: the operator dashboard.

Subscribes to the four display topics, rebuilds the PipelineStepResult that
orfd's AutonomyDashboard draws from, and shows it in an OpenCV window. Runs
only with visualize:=true, and is the one stage that is routinely slower than
its publishers, which is why its QoS depths are deliberately small.
"""

import os
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
import cv2
import numpy as np
from offroad_autonomy.perception.ego_mask import road_fraction
from offroad_autonomy.perception.perception_view import PerceptionView
from offroad_autonomy.planning.perception_gate import PerceptionGate
from offroad_autonomy.runtime.timing import RuntimeStats
from offroad_autonomy.types import (
    DEBUG_VIEW_KEYS,
    ControlCommand,
    FramePacket,
    PathPlan,
    PerceptionResult,
    PipelineStepResult,
    StabilizedResult,
)
from offroad_autonomy.visualization import AutonomyDashboard, DashboardTelemetry

from perception_suite.common import (
    MAX_PLAUSIBLE_AGE_S, LATEST_ONLY, decode_mask, load_pipeline_config)
# The ROS message and the library dataclass are both named ControlCommand. The
# dashboard draws from the dataclass, so the message is the one renamed here and
# _command_callback converts between the two.
from perception_suite_msgs.msg import ControlCommand as ControlCommandMsg

# Same title as orfd-lane-detection's dashboard window.
WINDOW = "Off-Road Autonomy Dashboard"

# The dashboard draws slower than the camera produces, about 24 FPS against 30,
# so any queue in front of it sits permanently full and every frame it shows is
# as old as that queue is deep. Both the DDS reader and the synchronizer
# defaulted to 10, which put the picture 366 ms behind the vehicle while the
# planner was working on frames 53 ms old. Nothing was wrong with the planning;
# the operator was just watching the past.
#
# Depth 2 and not 1: these four topics come from three different nodes and
# arrive up to 19 ms apart, and TimeSynchronizer only fires once it holds the
# same stamp from all four. A single slot discards one of a set whenever a
# render overruns, so the dashboard would skip far more frames than it has to.
# Two slots bound the staleness at two frame periods and still absorb the skew.
DISPLAY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)
SYNC_QUEUE = 2


class VisualizationNode(Node):
    """
    Stage 6: orfd's AutonomyDashboard, fed from the stage topics.

    The dashboard draws from a PipelineStepResult, so one is rebuilt per frame
    and the mask and path overlays are drawn by orfd's own code. The control
    readouts follow 7_controller_node and the speed readout follows the odometry
    source, and both fall back to zero when their topic goes quiet.
    """

    def __init__(self):
        """Open the dashboard window and subscribe to the display topics."""
        super().__init__('visualizer_node')
        config = load_pipeline_config(self)
        view = PerceptionView(config)
        self.camera = view.camera
        self.valid_roi = view.valid_roi
        self.ego_coverage = view.ego_coverage
        self.segmentation_mode = view.mode
        self.gate = PerceptionGate(config)
        self.dashboard = AutonomyDashboard(
            width=1600,
            height=900,
            colors=config.dashboard_colors,
            sensor=config.camera.sensor,
            thresholds=config.dashboard_thresholds,
        )
        # Not orfd's DashboardWindow: it shrinks each frame to the window's
        # reported image area, and OpenCV's Qt backend (Linux) reports its
        # tiny pre-show default forever, so the dashboard never grows. Here Qt
        # scales the full canvas itself. GUI_NORMAL drops Qt's toolbar and
        # status bar, so the window is a plain frame like orfd's on Windows.
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(WINDOW, self.dashboard.width, self.dashboard.height)
        # Placement, which OpenCV otherwise leaves to the window manager: the
        # same code lands bottom-left under Docker and top-left under the venv.
        # Defaults are the top-right corner of the board's 3440x1440
        # screen, allowing for the window being 1600x900.
        win_x = int(self.declare_parameter('window_x', 1840).value)
        win_y = int(self.declare_parameter('window_y', 0).value)
        cv2.moveWindow(WINDOW, win_x, win_y)
        self.get_logger().info(f'Dashboard window at {win_x},{win_y} '
                               f'({self.dashboard.width}x{self.dashboard.height})')
        self.debug_view = config.ui_debug_view
        self.timing_overlay = config.ui_timing_overlay
        self.stats = RuntimeStats()
        self.bridge = CvBridge()

        # All four share the source frame's stamp and arrive once per segmented
        # frame, so they match however slow segmentation is. See DISPLAY_QOS
        # for why both depths are small: this subscriber is the only one in the
        # pipeline that is routinely slower than its publisher.
        self.sync = TimeSynchronizer(
            [
                Subscriber(self, Image, 'segmented_frames', qos_profile=DISPLAY_QOS),
                Subscriber(self, Image, 'road_mask', qos_profile=DISPLAY_QOS),
                Subscriber(self, Image, 'stabilized_mask', qos_profile=DISPLAY_QOS),
                Subscriber(self, Path, 'path', qos_profile=DISPLAY_QOS),
            ],
            queue_size=SYNC_QUEUE,
        )
        self.sync.registerCallback(self.display_callback)

        # command is deliberately NOT one of the synchronized inputs above.
        # 7_controller_node only runs with control:=true, and a
        # TimeSynchronizer fires only once every input has arrived, so adding
        # it there would blank the dashboard entirely in the default six-node
        # graph. Latest-value-wins costs a frame of skew on a dial and risks
        # nothing in the picture.
        self._command = ControlCommand()
        self._command_at = None
        self.command_timeout_s = float(
            self.declare_parameter('command_timeout_s', 0.5).value)
        self.create_subscription(
            ControlCommandMsg, 'command', self._command_callback, LATEST_ONLY)

        # Speed comes from odometry for the same reason and with the same
        # treatment. odom_timeout_s is the name 7_controller_node already uses
        # for this question, and the launch file feeds both from one argument:
        # "has odometry gone quiet" has one answer, not two.
        self._speed_mps = 0.0
        self._speed_at = None
        self.odom_timeout_s = float(
            self.declare_parameter('odom_timeout_s', 0.5).value)
        self.create_subscription(
            Odometry, 'odom', self._odom_callback, LATEST_ONLY)

        # Show something straight away: nothing reaches display_callback until
        # the source has waited source_delay and the segmenter has loaded its
        # model, which is 10-20s of blank screen otherwise.
        #
        # WND_PROP_VISIBLE is only honoured once it has reported a visible
        # window at least once (see display_callback). OpenCV's GTK3 build
        # returns -1 for it unconditionally, which is "not implemented", not
        # "closed"; taking it at face value shuts the dashboard down on its
        # first frame on any such build.
        self._visibility_reliable = False
        self._warned_clock = False
        splash = np.zeros((360, 640, 3), np.uint8)
        cv2.putText(splash, 'waiting for the pipeline...', (40, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
        cv2.imshow(WINDOW, splash)
        for _ in range(10):
            cv2.waitKey(30)

        self.get_logger().info(
            'Keys: T timing overlay, Q quit, '
            + ' '.join(f'{i}={name}' for i, name in DEBUG_VIEW_KEYS.items()))

    def display_callback(self, frame_msg, raw_mask_msg, mask_msg, path_msg):
        """Draw one frame, and record its capture-to-display latency."""
        self.stats.tick()
        # Capture to display: the closest ROS analogue of orfd's loop latency.
        #
        # Only when the stamp is on this clock. Under bag replay it is the
        # bag's recorded time, so the subtraction returns the age of the
        # RECORDING, not of the frame. Unguarded this reported about 2.5 billion ms
        # on the dashboard, and the far more dangerous failure would have been
        # recording it as 0: a latency tile reading "0 / 0 ms  Within Budget"
        # is a plausible screenshot, and a wrong one. StageTimer in common.py
        # draws the same line for the same reason.
        captured = Time.from_msg(frame_msg.header.stamp)
        latency_ms = (self.get_clock().now() - captured).nanoseconds / 1e6
        if 0.0 <= latency_ms <= MAX_PLAUSIBLE_AGE_S * 1000.0:
            self.stats.record('latency', latency_ms)
        elif not self._warned_clock:
            self._warned_clock = True
            self.get_logger().info(
                f'visualizer: capture stamps are not on this clock (first '
                f'sample {latency_ms / 1000.0:.1f}s), which is normal for bag '
                'replay. Capture-to-display cannot be measured; the dashboard '
                'will show it as unavailable and report render time only.')

        result = self._step_result(frame_msg, raw_mask_msg, mask_msg, path_msg)
        t0 = time.perf_counter()
        canvas = self.dashboard.render(
            result,
            self._telemetry(result),
            plan=result.plan,
            valid_roi=self.valid_roi,
            debug_view=self.debug_view,
            timing_overlay=self.timing_overlay,
        )
        self.stats.record('dashboard_render', (time.perf_counter() - t0) * 1000.0)

        # The rate drawn on the dashboard, logged every 60 frames so it can be
        # checked without standing in front of the monitor.
        self._viz_n = getattr(self, '_viz_n', 0) + 1
        if self._viz_n % 60 == 0:
            latency = self.stats.stage('latency')
            render = self.stats.stage('dashboard_render')
            measured = f'capture-to-display p50 {latency.p50_ms:.1f} ms'
            if not latency.count:
                measured = 'capture-to-display unavailable (stamps off clock)'
            self.get_logger().info(
                f'render rate {self.stats.fps():.1f} FPS | {measured} | '
                f'render {render.mean_ms:.1f} ms | n={self._viz_n}')

        cv2.imshow(WINDOW, canvas)
        key = cv2.waitKey(1) & 0xFF
        # -1 means the build does not implement this property (OpenCV's GTK3
        # build does exactly that), so it cannot be distinguished from a closed
        # window on its own. Trust it only after it has once reported visible.
        visible = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
        if visible >= 1:
            self._visibility_reliable = True
        closed = self._visibility_reliable and visible < 1
        if closed or key in (27, ord('q')):
            self.get_logger().info('Dashboard closed')
            rclpy.try_shutdown()
            return
        self._handle_key(key)

    def _command_callback(self, msg):
        """
        Hold the newest command for the dials, with its arrival time.

        Arrival time rather than the header stamp, because the question a
        frozen dial raises is whether the controller is still publishing, not
        how old the frame behind it was.
        """
        self._command = ControlCommand(
            steering=msg.steering,
            throttle=msg.throttle,
            brake=msg.brake,
            parkingbrake=msg.parkingbrake,
        )
        self._command_at = self.get_clock().now()

    def _odom_callback(self, msg):
        """Hold the newest speed for the speedometer, with its arrival time."""
        self._speed_mps = float(msg.twist.twist.linear.x)
        self._speed_at = self.get_clock().now()

    def _current_speed_mph(self):
        """
        Return the speed in mph, or zero once odometry goes quiet.

        A pipeline run from video with no CAN source never publishes odometry
        at all, so zero here is the ordinary case rather than a fault.
        """
        if self._speed_at is None:
            return 0.0
        age_ns = (self.get_clock().now() - self._speed_at).nanoseconds
        if age_ns > self.odom_timeout_s * 1e9:
            return 0.0
        return self._speed_mps * 2.236936

    def _current_command(self):
        """
        Zero once the controller stops publishing.

        Holding the last value would leave the dials showing a steering angle
        that nothing is commanding any more, which reads as live and is not.
        Zero is also what a graph with no controller shows, so the two agree.
        """
        if self._command_at is None:
            return ControlCommand()
        age_ns = (self.get_clock().now() - self._command_at).nanoseconds
        if age_ns > self.command_timeout_s * 1e9:
            return ControlCommand()
        return self._command

    def _step_result(self, frame_msg, raw_mask_msg, mask_msg, path_msg):
        image = self.bridge.imgmsg_to_cv2(frame_msg, 'bgr8')
        h, w = image.shape[:2]
        raw_mask, confidence = decode_mask(self.bridge.imgmsg_to_cv2(raw_mask_msg, 'mono8'))
        mask, _ = decode_mask(self.bridge.imgmsg_to_cv2(mask_msg, 'mono8'))

        perception = PerceptionResult(
            mask=raw_mask,
            confidences=[confidence],
            num_detections=int(raw_mask.any()),
            valid_roi=self.valid_roi,
            road_fraction=road_fraction(raw_mask, self.valid_roi),
        )
        stabilized = StabilizedResult(
            mask=mask,
            # The postprocess stage does not publish its stability score.
            stability_score=0.0,
            raw_result=perception,
            valid_roi=self.valid_roi,
            road_fraction=road_fraction(mask, self.valid_roi),
        )
        return PipelineStepResult(
            frame=FramePacket(
                raw=image, preprocessed=image, timestamp=time.perf_counter(), height=h, width=w),
            perception=perception,
            stabilized=stabilized,
            plan=self._plan(path_msg, h, w),
            command=self._current_command(),
        )

    def _plan(self, path_msg, h, w):
        """
        Project the published path back into image pixels.

        The planner publishes ground metres, but the dashboard draws on the
        image the planner read, so the path has to be projected back.
        """
        roi_top = self.gate.roi_top(h)
        if not path_msg.poses:
            # The planner publishes an empty path only when the gate stops.
            return PathPlan(
                centerline=np.empty((0, 2), dtype=np.float32),
                fallback_active=True,
                fallback_reason='no valid path - stopping',
                speed_scale=0.0,
                roi_top=roi_top,
            )
        forward = np.array([p.pose.position.x for p in path_msg.poses])
        right = -np.array([p.pose.position.y for p in path_msg.poses])
        centerline = self.camera.ground_to_image(forward, right)
        centerline[:, 0] = np.clip(centerline[:, 0], 0.0, w - 1.0)
        return PathPlan(centerline=centerline, roi_top=roi_top)

    def _telemetry(self, result):
        latency = self.stats.stage('latency')
        # NaN, not zero, when nothing measurable has arrived. The dashboard
        # renders it as "n/a" rather than drawing a number nobody measured.
        latency_mean = float('nan')
        latency_p95 = float('nan')
        if latency.count:
            latency_mean = latency.mean_ms
            latency_p95 = latency.p95_ms
        fallback_state = 'NONE'
        if result.plan.fallback_active:
            fallback_state = 'GATE STOP'
        lines = []
        if self.timing_overlay:
            lines = ['ROS VISUALIZER'] + self.stats.format_lines(('latency', 'dashboard_render'))
        return DashboardTelemetry(
            speed_mph=self._current_speed_mph(),
            steering=result.command.steering,
            throttle=result.command.throttle,
            brake=result.command.brake,
            perception_confidence=result.perception.confidences[0],
            stability_score=result.stabilized.stability_score,
            kalman_active=False,
            fps=self.stats.fps(),
            latency_ms=latency_mean,
            latency_p95_ms=latency_p95,
            road_fraction=result.stabilized.road_fraction,
            ego_coverage=self.ego_coverage,
            segmentation_mode=self.segmentation_mode,
            fallback_state=fallback_state,
            dashboard_fps=self.stats.fps(),
            timing_lines=lines,
        )

    def _handle_key(self, key):
        """
        Handle orfd's display keys.

        E and P start and stop the vehicle in orfd, so they do nothing here.
        """
        if key in (ord('t'), ord('T')):
            self.timing_overlay = not self.timing_overlay
        elif ord('0') <= key <= ord('9') and key - ord('0') in DEBUG_VIEW_KEYS:
            self.debug_view = DEBUG_VIEW_KEYS[key - ord('0')]
            self.get_logger().info(f'Debug view: {self.debug_view}')


def main(args=None):
    """Run the dashboard, refusing to start without a display."""
    rclpy.init(args=args)
    # OpenCV's Qt backend only draws through X11, and with no display it
    # aborts the process rather than raising, so check before any window opens.
    if not os.environ.get('DISPLAY'):
        rclpy.logging.get_logger('visualizer_node').error(
            'No display (DISPLAY is unset, e.g. over SSH). Run on the desktop, '
            'export DISPLAY=:0 for a local screen, or launch with visualize:=false.')
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(VisualizationNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under launch, Ctrl+C arrives twice (terminal and launch); the second
        # must not interrupt teardown or interpreter exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
