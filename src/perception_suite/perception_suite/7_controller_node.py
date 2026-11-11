"""
Stage 7: plan and CAN in, steering out. Runs only with control:=true.

Steering from offroad_autonomy's StanleyController, or MPCController where the
config asks for it. Kept out of 5_planner_node so the camera test can leave
control out entirely, and so the vehicle's own controller can replace it
without touching the perception chain.
"""

import math
import signal

import numpy as np
import rclpy
from ads_messages.msg import CANMsg, ConfigState, VehicleControl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from offroad_autonomy.control.mpc_controller import MPCController
from offroad_autonomy.control.stanley_controller import StanleyController
from offroad_autonomy.perception.perception_view import PerceptionView
from offroad_autonomy.types import PathPlan, VehicleState
from perception_suite_msgs.msg import ControlCommand, PlannerOutput

from perception_suite import autosteer
from perception_suite.common import (EXTERNAL_LATEST, LATEST_ONLY, StageTimer,
                                     load_pipeline_config)


for _name in ('OFF', 'LONG', 'AUTOSTEER', 'ABORT', 'CANCEL'):
    if getattr(autosteer, _name) != getattr(ConfigState, f'CDCU_CTRL_{_name}'):
        raise RuntimeError(f'autosteer.{_name} disagrees with ConfigState.CDCU_CTRL_{_name}')


class ControllerNode(Node):
    """Stage 7: the road wheel angle for the CAN bus, from the planner output."""

    def __init__(self):
        """Build the controller and subscribe to plan and the vehicle state."""
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

 
        self.max_wheel_rad = math.radians(config.max_wheel_angle_deg)
        # Float defaults, not strings, and which is right depends on how the
        # launch file reaches the node. A source node is built in _source(),
        # which calls perform(context) and hands over a raw Python string, so
        # 1_camera_node and 1_bag_node must declare string defaults. This node
        # is built by stage(), which passes the LaunchConfiguration itself and
        # lets launch_ros infer the type, so '0.5' arrives as a DOUBLE and a
        # string default is rejected with InvalidParameterTypeException.
        # Declaring strings here was tried first and failed exactly that way.
        self.speed_mps = float(self.declare_parameter('speed_mps', 0.0).value)
        self.odom_timeout_s = float(
            self.declare_parameter('odom_timeout_s', 0.5).value)
        self.last_speed_ns = None

        self.steering_sign = float(
            self.declare_parameter('steering_sign', -1.0).value)

        self.plan_timeout_s = float(
            self.declare_parameter('plan_timeout_s', 0.3).value)
        self.last_plan_ns = None
        self.delta = float('nan')

        self.max_rwa_rad = self.max_wheel_rad


        enabled = bool(self.declare_parameter('autosteer', False).value)
        self.toggle = autosteer.AutosteerToggle(enabled=enabled)
        self.pending_errors = []

        self.stage_timer = StageTimer(self, 'controller')
        self.publisher = self.create_publisher(ControlCommand, 'command', LATEST_ONLY)

        self.vehicle_control_publisher = self.create_publisher(
            VehicleControl, 'vehicle_control', EXTERNAL_LATEST)
        self.create_subscription(PlannerOutput, 'plan', self.on_plan, LATEST_ONLY)

        can_topic = str(self.declare_parameter('can_topic', '').value).strip()
        if can_topic:
            self.create_subscription(CANMsg, can_topic, self.on_can, EXTERNAL_LATEST)
            state = 'disabled (autosteer:=false), mode held OFF'
            if enabled:
                state = 'ENABLED: CC up in D engages steering'
            self.get_logger().info(
                f'vehicle state from {can_topic}; autosteer {state}')
        else:
            self.create_subscription(Odometry, 'odom', self.on_odom, LATEST_ONLY)
            if enabled:
                self.get_logger().warn(
                    'autosteer:=true without can_topic: there is no cruise '
                    'switch or gear to engage on, so the mode stays OFF')

    def on_odom(self, msg):
        """Speed when there is no /CAN, as in clip replay from a csv."""
        self.speed_mps = float(msg.twist.twist.linear.x)
        self.last_speed_ns = self.get_clock().now().nanoseconds

    def on_can(self, msg):
        """Read the vehicle state and advance the CDCU mode on one CAN sample."""

        if not msg.spd_error and math.isfinite(msg.spd_mps):
            self.speed_mps = float(msg.spd_mps)
            self.last_speed_ns = self.get_clock().now().nanoseconds
        if msg.max_rwa_deg > 0.0:
            self.max_rwa_rad = min(self.max_wheel_rad, math.radians(msg.max_rwa_deg))
        button = int(msg.cc_button)
        if msg.cc_button_error:
            # A switch the decoder could not read is not a press.
            button = 0
        before = self.toggle.mode
        errors = self.toggle.update(
            gear=msg.gear, cc_button=button,
            delta_valid=self._delta_is_valid(),
            now_s=self.get_clock().now().nanoseconds * 1e-9)
        self._log_mode_change(before, errors)
        self.pending_errors.extend(errors)

    def _delta_is_valid(self):
        """True when the last angle came from a plan fresh enough to steer by."""
        if self.last_plan_ns is None or not math.isfinite(self.delta):
            return False
        age_s = (self.get_clock().now().nanoseconds - self.last_plan_ns) * 1e-9
        return age_s <= self.plan_timeout_s

    def _log_mode_change(self, before, errors):
        for error in errors:
            self.get_logger().warn(error, throttle_duration_sec=1.0)
        if self.toggle.mode != before:
            self.get_logger().info(
                f'CDCU mode {autosteer.MODE_NAMES[before]} -> '
                f'{autosteer.MODE_NAMES[self.toggle.mode]}')

    def _speed_is_stale(self):
        """``(stale, age_s)``. Never stale before the first speed arrives."""
        if self.last_speed_ns is None:
            return False, None
        age_s = (self.get_clock().now().nanoseconds - self.last_speed_ns) * 1e-9
        return age_s > self.odom_timeout_s, age_s

    def on_plan(self, msg):
        """Turn one planner output into a road wheel angle."""
        t0 = self.stage_timer.start()
        self.last_plan_ns = self.get_clock().now().nanoseconds

        stale, age_s = self._speed_is_stale()
        if stale:
            reason = f'vehicle speed stale by {age_s:.2f}s'
            self.get_logger().error(f'{reason}, no steering', throttle_duration_sec=1.0)
            self.publisher.publish(self._command_message(msg.header, 0.0, reason))
            self._publish_vehicle_control(msg.header, float('nan'), reason)
            self.stage_timer.done(t0, msg.header)
            return

        plan = self._plan_from_message(msg)
        command = self.controller.compute(plan, VehicleState(speed_mps=self.speed_mps))


        delta = float('nan')
        reasons = []
        if plan.fallback_active and plan.fallback_reason:
            reasons.append(f'planner: {plan.fallback_reason}')
        debug = command.debug
        if debug is not None and debug.controller_fallback and debug.controller_fallback_reason:
            reasons.append(f'controller: {debug.controller_fallback_reason}')
        if len(plan.centerline) < 2:
            reasons.append('no path')
        elif not math.isfinite(command.steering):
            reasons.append('controller returned a non-finite steering angle')
        else:
            delta = float(np.clip(
                self.steering_sign * command.steering * self.max_wheel_rad,
                -self.max_rwa_rad, self.max_rwa_rad))
        reason = '; '.join(reasons)

        steering = 0.0
        if math.isfinite(delta):
            steering = float(command.steering)
        self.publisher.publish(self._command_message(msg.header, steering, reason))
        self._publish_vehicle_control(msg.header, delta, reason)
        self.stage_timer.done(t0, msg.header)

    def _publish_vehicle_control(self, header, delta, reason):
        """Publish the angle and the mode the decoder puts on the bus."""
        self.delta = delta
        if not math.isfinite(delta):
            before = self.toggle.mode
            errors = self.toggle.invalidate(reason or 'no steering angle')
            self._log_mode_change(before, errors)
            self.pending_errors.extend(errors)
        out = VehicleControl()
        out.header = header
        out.cdcu_ctrl_mode = int(self.toggle.mode)
        out.delta = float(delta)

        errors = list(self.pending_errors)
        if reason:
            errors.append(reason)
        out.errors = errors
        self.pending_errors = []
        self.vehicle_control_publisher.publish(out)

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
    def _command_message(header, steering, reason):
        """The dashboard's view: steering and why, nothing longitudinal."""
        out = ControlCommand()
        out.header = header
        out.steering = float(steering)
        out.throttle = 0.0
        out.brake = 0.0
        out.parkingbrake = 0.0
        out.reason = reason
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
