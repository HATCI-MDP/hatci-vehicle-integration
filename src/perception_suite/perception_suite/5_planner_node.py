"""
Stage 5: stabilized_mask in, path and plan out.

Gates the mask on confidence and coverage, then fits a centreline through the
traversable region. Publishes the result twice: path in ground metres for
anything that wants a standard nav_msgs type, and plan in image pixels for
7_controller_node, which runs its own projection.
"""

import signal

import array
import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from perception_suite_msgs.msg import PlannerOutput
from offroad_autonomy.perception.perception_view import PerceptionView
from offroad_autonomy.planning.centerline_planner import CenterlinePlanner
from offroad_autonomy.types import PerceptionResult, StabilizedResult, VehicleState

from perception_suite.common import LATEST_ONLY, StageTimer, decode_mask, load_pipeline_config


class PlannerNode(Node):
    """Stage 5: perception gate and centreline fit (CenterlinePlanner), publishing /path."""

    def __init__(self):
        """Build the planner and subscribe to stabilized_mask and odom."""
        super().__init__('planner_node')
        config = load_pipeline_config(self)
        # Empty means "whatever the config says", so a run without odometry
        # behaves exactly as it did before this parameter existed.
        mode = str(self.declare_parameter('planner_mode', '').value).strip().lower()
        if mode:
            if mode not in ('baseline', 'advanced', 'grid'):
                raise ValueError(
                    f"planner_mode must be 'baseline', 'advanced' or 'grid', got {mode!r}")
            if mode != config.planner_mode:
                self.get_logger().info(
                    f'planner mode {config.planner_mode} -> {mode} (launch argument)')
            config.planner_mode = mode
        view = PerceptionView(config)
        self.camera = view.camera
        self.valid_roi = view.valid_roi
        self.planner = CenterlinePlanner(config, camera=self.camera)
        # No vehicle to read from, so speed is fixed. Only the advanced
        # planner uses it.
        speed = float(self.declare_parameter('speed_mps', 0.0).value)
        self.vehicle_state = VehicleState(speed_mps=speed)
        # Without odometry the state never changes, so the BEV grid cannot
        # register one frame against the next and falls back to forgetting.
        #
        # That fallback is also what has to happen when odometry STOPS. The
        # pose used to be latched forever: `valid=True` was set on arrival and
        # nothing expired it, so a dead odometry source left the planner
        # registering new masks against a position the vehicle had already
        # left, piling observations from different real places into the same
        # cells. Corrupting the map is worse than emptying it, so the pose now
        # expires and ground_pose() goes back to returning None.
        self.last_odom_ns = None
        self.odom_timeout_s = float(
            self.declare_parameter('odom_timeout_s', 0.5).value)
        self._warned_stale_odom = False
        self.create_subscription(Odometry, 'odom', self.on_odom, LATEST_ONLY)
        # Origin on the ground below the camera, x forward and y left (REP 103).
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.bridge = CvBridge()
        self.stage_timer = StageTimer(self, 'planner')
        self.publisher = self.create_publisher(Path, 'path', 10)
        # /path stays exactly as it was. Anything already consuming ground
        # metres keeps working, and this is additive rather than a replacement.
        #
        # /plan is what the controller consumes. It cannot use /path: the
        # controller projects the centreline itself with its own range limit
        # and then keeps the pixel values, so ground metres are the wrong data
        # rather than merely the wrong units. See PlannerOutput.msg.
        self.plan_publisher = self.create_publisher(PlannerOutput, 'plan', LATEST_ONLY)
        self.subscription = self.create_subscription(
            Image, 'stabilized_mask', self.on_mask, LATEST_ONLY)

    def on_odom(self, msg):
        """Hold the newest ego pose, which shifts the BEV grid."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw straight from the quaternion; the odometry is planar so roll and
        # pitch are zero and the full conversion is not needed.
        heading = 2.0 * math.atan2(q.z, q.w)
        self.vehicle_state = VehicleState(
            position=(p.x, p.y, 0.0),
            rotation=(q.x, q.y, q.z, q.w),
            speed_mps=float(msg.twist.twist.linear.x),
            heading_rad=heading,
            # ground_pose() returns None without this, and the grid then
            # forgets every frame instead of accumulating evidence.
            direction=(math.cos(heading), math.sin(heading), 0.0),
            valid=True)
        self.last_odom_ns = self.get_clock().now().nanoseconds
        # Reset per outage, not once per process: a bus that drops out twice
        # is two events worth knowing about, not one.
        self._warned_stale_odom = False

    def _usable_state(self):
        """
        Return the pose to plan from, expiring it once odometry goes quiet.

        Never stale before the first odometry message ever arrives: a pipeline
        running on a fixed speed_mps with no CAN source at all is a supported
        configuration, not a fault, and it already plans without a grid.
        """
        if self.last_odom_ns is None:
            return self.vehicle_state
        age_s = (self.get_clock().now().nanoseconds - self.last_odom_ns) * 1e-9
        if age_s <= self.odom_timeout_s:
            return self.vehicle_state
        if not self._warned_stale_odom:
            self._warned_stale_odom = True
            self.get_logger().error(
                f'odometry is {age_s:.1f}s old, past odom_timeout_s '
                f'({self.odom_timeout_s:.1f}s). Dropping the pose so the BEV '
                'grid forgets instead of registering new masks against a '
                'position the vehicle has already left.')
        # Speed is the last measurement and is still the best estimate, but
        # position and direction are not, so the grid must not use them.
        return VehicleState(speed_mps=self.vehicle_state.speed_mps)

    def on_mask(self, msg):
        """Plan a centreline through one stabilized mask."""
        t0 = self.stage_timer.start()
        mask, confidence = decode_mask(self.bridge.imgmsg_to_cv2(msg, 'mono8'))
        stabilized = StabilizedResult(
            mask=mask,
            raw_result=PerceptionResult(
                mask=mask, confidences=[confidence], valid_roi=self.valid_roi),
            valid_roi=self.valid_roi,
        )
        plan = self.planner.plan(stabilized, vehicle_state=self._usable_state())
        if plan.fallback_active:
            self.get_logger().warn(plan.fallback_reason, throttle_duration_sec=1.0)

        # Published even when empty: an empty path means stop.
        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = self.frame_id
        if len(plan.centerline) > 0:
            forward, right, ok = self.camera.image_to_ground(plan.centerline)
            for f, r in zip(forward[ok], right[ok]):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(f)
                pose.pose.position.y = -float(r)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.publisher.publish(path)
        self.plan_publisher.publish(self._plan_message(plan, msg.header))
        self.stage_timer.done(t0, msg.header)

    @staticmethod
    def _plan_message(plan, header):
        """
        ``plan`` as the controller consumes it, in image pixels.

        Every sequence field is assigned as an ``array.array`` built from raw
        bytes. That is not a style choice. rosidl's generated setters validate
        a Python list element by element, and on this board a 640x413 mask costs
        49.5 ms that way against 0.05 ms as an ``array.array``, measured. A list
        here put the planner at 61 ms per frame and capped the whole pipeline at
        15.9 Hz. numpy arrays are rejected by the setter outright.
        """
        out = PlannerOutput()
        out.header = header
        centerline = np.asarray(plan.centerline, dtype=np.float32)
        if centerline.size:
            # (u, v) per row, the same order image_to_ground takes.
            out.centerline_u = array.array('f', centerline[:, 0].tobytes())
            out.centerline_v = array.array('f', centerline[:, 1].tobytes())
        mask = plan.planner_mask
        if mask is not None:
            # The controller only ever tests truthiness per pixel, but 0/255
            # keeps the topic viewable as an image during debugging.
            packed = np.ascontiguousarray(mask).astype(bool).view(np.uint8) * 255
            out.mask_height, out.mask_width = packed.shape[:2]
            out.planner_mask = array.array('B', packed.tobytes())
        out.speed_scale = float(plan.speed_scale)
        out.min_clearance_m = float(plan.min_clearance_m)
        out.fallback_active = bool(plan.fallback_active)
        out.fallback_reason = str(plan.fallback_reason)
        out.roi_top = int(plan.roi_top)
        return out


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    try:
        rclpy.spin(PlannerNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under launch, Ctrl+C arrives twice (terminal and launch); the second
        # must not interrupt teardown or interpreter exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
