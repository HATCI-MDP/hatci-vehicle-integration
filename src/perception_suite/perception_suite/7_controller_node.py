"""
Stage 7: plan in, command out. Runs only with control:=true.

Steering and speed from offroad_autonomy's StanleyController, or MPCController
where the config asks for it. Kept out of 5_planner_node so the camera test can
leave control out entirely, and so the vehicle's own controller can replace it
without touching the perception chain.
"""

import signal

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from offroad_autonomy.control.mpc_controller import MPCController
from offroad_autonomy.control.stanley_controller import StanleyController
from offroad_autonomy.perception.perception_view import PerceptionView
from offroad_autonomy.types import PathPlan, VehicleState
from perception_suite_msgs.msg import ControlCommand, PlannerOutput

from perception_suite.common import LATEST_ONLY, StageTimer, load_pipeline_config


class ControllerNode(Node):
    """Stage 7: steering and speed from the planner output (StanleyController)."""

    def __init__(self):
        """Build the controller and subscribe to plan and odom."""
        super().__init__('controller_node')
        config = load_pipeline_config(self)
        # The controller projects the centreline itself, so it needs the same
        # camera geometry the planner used. Both build it from the same config
        # rather than one passing it to the other, because a camera model that
        # travelled over a topic could silently disagree with the pixels it is
        # applied to.
        view = PerceptionView(config)
        self.camera = view.camera

        if config.controller not in ('stanley', 'mpc'):
            raise ValueError(
                f"controller must be 'stanley' or 'mpc', got {config.controller!r}")
        controller_cls = StanleyController
        if config.controller == 'mpc':
            controller_cls = MPCController
        self.controller = controller_cls(config, camera=self.camera)
        self.get_logger().info(f'controller: {config.controller}')

        # Used until odometry arrives, and the only speed there is when no CAN
        # source is configured at all. Matches 5_planner_node's parameter of
        # the same name so one value configures both.
        # Float defaults, not strings, and which is right depends on how the
        # launch file reaches the node. A source node is built in _source(),
        # which calls perform(context) and hands over a raw Python string, so
        # 1_camera_node and 1_bag_node must declare string defaults. This node
        # is built by stage(), which passes the LaunchConfiguration itself and
        # lets launch_ros infer the type, so '0.5' arrives as a DOUBLE and a
        # string default is rejected with InvalidParameterTypeException.
        # Declaring strings here was tried first and failed exactly that way.
        self.fallback_speed = float(self.declare_parameter('speed_mps', 0.0).value)
        # 5_planner_node latches the last pose for ever and nothing expires it
        # (OPEN-ITEMS 1.8). That is survivable for planning and is not for
        # control: the speed law would keep acting on a speed the vehicle no
        # longer has. Once odometry has been seen, losing it is a fault.
        self.odom_timeout_s = float(
            self.declare_parameter('odom_timeout_s', 0.5).value)
        self.speed_mps = self.fallback_speed
        self.last_odom_ns = None

        self.stage_timer = StageTimer(self, 'controller')
        self.publisher = self.create_publisher(ControlCommand, 'command', LATEST_ONLY)
        self.create_subscription(Odometry, 'odom', self.on_odom, LATEST_ONLY)
        self.create_subscription(PlannerOutput, 'plan', self.on_plan, LATEST_ONLY)

    def on_odom(self, msg):
        """Hold the newest measured speed, and when it arrived."""
        self.speed_mps = float(msg.twist.twist.linear.x)
        self.last_odom_ns = self.get_clock().now().nanoseconds

    def _odom_is_stale(self):
        """``(stale, age_s)``. Never stale before the first message arrives."""
        if self.last_odom_ns is None:
            return False, None
        age_s = (self.get_clock().now().nanoseconds - self.last_odom_ns) * 1e-9
        return age_s > self.odom_timeout_s, age_s

    def on_plan(self, msg):
        """Turn one planner output into a control command."""
        t0 = self.stage_timer.start()

        stale, age_s = self._odom_is_stale()
        if stale:
            # Full brake rather than coasting: the vehicle is moving and the
            # controller no longer knows how fast.
            self.get_logger().error(
                f'odometry {age_s:.2f}s old, stopping',
                throttle_duration_sec=1.0)
            self.publisher.publish(self._stop_command(
                msg.header, f'odometry stale by {age_s:.2f}s'))
            self.stage_timer.done(t0, msg.header)
            return

        plan = self._plan_from_message(msg)
        state = VehicleState(speed_mps=self.speed_mps)
        command = self.controller.compute(plan, state)
        self.publisher.publish(self._command_message(command, msg, plan))
        self.stage_timer.done(t0, msg.header)

    @staticmethod
    def _plan_from_message(msg):
        """Rebuild a ``PathPlan`` from the fields the controller reads."""
        # frombuffer rather than asarray throughout: the fields arrive as
        # array.array, and letting numpy walk them element by element costs
        # more than everything else this node does put together.
        centerline = np.empty((0, 2), dtype=np.float64)
        if len(msg.centerline_u):
            centerline = np.column_stack((
                np.frombuffer(msg.centerline_u, dtype=np.float32).astype(np.float64),
                np.frombuffer(msg.centerline_v, dtype=np.float32).astype(np.float64)))
        planner_mask = None
        if msg.mask_width and msg.mask_height:
            planner_mask = np.frombuffer(
                msg.planner_mask, dtype=np.uint8
            ).reshape(msg.mask_height, msg.mask_width).astype(bool)
        return PathPlan(
            centerline=centerline,
            min_clearance_m=float(msg.min_clearance_m),
            fallback_active=bool(msg.fallback_active),
            fallback_reason=str(msg.fallback_reason),
            speed_scale=float(msg.speed_scale),
            planner_mask=planner_mask,
            roi_top=int(msg.roi_top))

    @staticmethod
    def _stop_command(header, reason):
        out = ControlCommand()
        out.header = header
        out.steering = 0.0
        out.throttle = 0.0
        out.brake = 1.0
        out.parkingbrake = 0.0
        out.reason = reason
        return out

    @staticmethod
    def _command_message(command, msg, plan):
        out = ControlCommand()
        out.header = msg.header
        out.steering = float(command.steering)
        out.throttle = float(command.throttle)
        out.brake = float(command.brake)
        out.parkingbrake = float(command.parkingbrake)
        # An empty path and a deliberate stop produce the same numbers, so the
        # reason is the only thing that tells them apart downstream.
        reasons = []
        if plan.fallback_active and plan.fallback_reason:
            reasons.append(f'planner: {plan.fallback_reason}')
        debug = command.debug
        if debug is not None:
            if debug.controller_fallback and debug.controller_fallback_reason:
                reasons.append(f'controller: {debug.controller_fallback_reason}')
            if debug.speed_reason:
                reasons.append(f'speed: {debug.speed_reason}')
        out.reason = '; '.join(reasons)
        return out


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    try:
        rclpy.spin(ControllerNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under launch, Ctrl+C arrives twice (terminal and launch); the second
        # must not interrupt teardown or interpreter exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
