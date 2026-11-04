import signal

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from offroad_autonomy.perception.perception_view import PerceptionView
from offroad_autonomy.planning.centerline_planner import CenterlinePlanner
from offroad_autonomy.types import PerceptionResult, StabilizedResult, VehicleState

from perception_suite.common import LATEST_ONLY, StageTimer, decode_mask, load_pipeline_config


class PlannerNode(Node):
    """Stage 5: perception gate and centreline fit (CenterlinePlanner), publishing /path."""

    def __init__(self):
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
        self.create_subscription(Odometry, 'odom', self.on_odom, LATEST_ONLY)
        # Origin on the ground below the camera, x forward and y left (REP 103).
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.bridge = CvBridge()
        self.stage_timer = StageTimer(self, 'planner')
        self.publisher = self.create_publisher(Path, 'path', 10)
        self.subscription = self.create_subscription(
            Image, 'stabilized_mask', self.on_mask, LATEST_ONLY)

    def on_odom(self, msg):
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

    def on_mask(self, msg):
        t0 = self.stage_timer.start()
        mask, confidence = decode_mask(self.bridge.imgmsg_to_cv2(msg, 'mono8'))
        stabilized = StabilizedResult(
            mask=mask,
            raw_result=PerceptionResult(
                mask=mask, confidences=[confidence], valid_roi=self.valid_roi),
            valid_roi=self.valid_roi,
        )
        plan = self.planner.plan(stabilized, vehicle_state=self.vehicle_state)
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
        self.stage_timer.done(t0, msg.header)


def main(args=None):
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
