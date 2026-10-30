"""Publish ego odometry from the vehicle's live /CAN topic, for the BEV grid planner.

Why this node exists
--------------------
The BEV grid planner needs ego motion to register consecutive frames against
each other. In the lab that motion came from odom_node.py, which replays a CSV
extracted from the recorded bags, one row per published frame. So the
configuration this project reports on - 26s segmentation plus the BEV grid -
has only ever run against recorded odometry. This node is the live
replacement.

It subscribes to the same topic the bags recorded, on the ASSUMPTION that the
node publishing /CAN runs on the live vehicle and not only in the recording
rig. That assumption is ours and is not yet confirmed; it is item 1.1 in
OPEN-ITEMS.md. If it holds there is no DBC to decode. If /CAN turns out to
exist only in the recording setup, this node needs a raw socketcan reader and
a DBC instead, and only the input half below changes.
The two signals come straight off the message, exactly as extract_can.py takes
them out of the bag:

    speed_mps <- msg.spd_mps                        float32, m/s
    yaw_rate  <- msg.imu.imu.angular_velocity.z     float32, rad/s

Published contract, byte-for-byte identical to odom_node so that nothing
downstream has to change:

    topic            odom
    type             nav_msgs/Odometry
    QoS              RELIABLE, KEEP_LAST, depth 1   (perception_suite LATEST_ONLY)
    header.stamp     the stamp of the FRAME this sample is paired with,
                     NOT now(). 5_planner_node pairs this odometry with the
                     mask derived from that frame and the pipeline is several
                     stages deep, so a now() stamp would pair a fresh pose
                     with an old mask.
    header.frame_id  odom
    child_frame_id   base_link
    pose.position    x, y in metres, dead-reckoned
    pose.orientation yaw-only quaternion (0, 0, sin(h/2), cos(h/2))
    twist.linear.x   speed, m/s
    twist.angular.z  yaw rate, rad/s

What deliberately did not change from the replay node
-----------------------------------------------------
Still publishing one sample per FRAME, driven by the raw_frames subscription,
not one per CAN message. /CAN arrives at its own rate (30 Hz in the bags, but
that is not a guarantee), so publishing on CAN arrival would break the stamp
contract above and would let the planner see two odometry updates for one frame
or none. Holding the most recent sample and emitting it against each frame's
own stamp preserves exactly the behaviour that was measured.

Dead reckoning moved from offline to online. extract_can.py integrated the
whole drive in one pass before the run; here x, y and heading accumulate as
messages arrive, with the same arithmetic. Absolute drift does not matter: the
grid only ever compares consecutive poses and decays anything older than
memory_s. The offline integration was validated against the bags' own GPS at
0.6-1.3% of path length over ~2 km, with median position error 0.3-0.6 m over
the 1-2 s the grid actually remembers.

Two things to confirm with HATCI
--------------------------------
1. The error flags. This node skips any message with `error` or `spd_error`
   set, or with `imu.valid` false, on the assumption that those mean what
   their names say. If a flag is set permanently on a healthy vehicle this
   node will go silent and the grid will never shift, so the _report() line
   below says explicitly how many messages were dropped and why.
   `imu.yaw_rate_status` is reported but never gated on, because the meaning
   of its values is not documented in the bag's schema.
2. Sign convention. REP-103 and sensor_msgs/Imu both say angular velocity is
   right-handed about +z, so a LEFT turn is positive. The node assumes that.
   If the vehicle reports the opposite, the grid will shift the wrong way on
   every turn and it will look like a perception fault, not a sign error.
"""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image

from perception_suite.common import LATEST_ONLY

# The type is resolved at runtime rather than imported, so this module still
# imports on a board where ads_messages is not built. That matters because
# pipeline.launch.py brings every stage up together: an ImportError here would
# take down the whole pipeline instead of just leaving the grid without
# odometry. The schema recovered from the Oct 2026 bags is
# ads_messages/msg/CANMsg, whose fields are spd_mps, spd_fl_mps ... and
# imu (ads_messages/CANImu, which wraps a sensor_msgs/Imu).
DEFAULT_CAN_TYPE = 'ads_messages/msg/CANMsg'

# Sanity bounds, used only to warn once. They exist because the single most
# likely integration mistake is a unit mismatch, and it is silent: a yaw rate
# populated in deg/s instead of rad/s is 57x too large, and the grid will
# believe the vehicle is spinning. No road vehicle yaws at 170 deg/s, and none
# of ours does 130 mph, so either reading past these bounds means the units are
# wrong rather than the driving being exciting.
MAX_PLAUSIBLE_YAW_RAD_S = 3.0
MAX_PLAUSIBLE_SPEED_M_S = 60.0


class LiveOdomNode(Node):
    def __init__(self):
        super().__init__('live_odom_node')

        self.topic = str(self.declare_parameter('can_topic', '/CAN').value)
        type_name = str(self.declare_parameter('can_msg_type', DEFAULT_CAN_TYPE).value)
        # How long a CAN sample stays usable. At 30 Hz, 0.5 s is 15 missed
        # messages: comfortably past "a dropped frame" and well short of
        # "the planner has been steering on a stale pose for a second".
        self.stale_timeout_s = float(
            self.declare_parameter('stale_timeout_s', 0.5).value)
        self.trust_flags = bool(self.declare_parameter('trust_flags', True).value)

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.last_can_t = 0.0
        self.last_integrate_t = None
        self.received = 0
        self.used = 0
        self.flagged = 0
        self.warned_units = False

        self.pub = self.create_publisher(Odometry, 'odom', LATEST_ONLY)
        self.create_subscription(Image, 'raw_frames', self.on_frame, LATEST_ONLY)

        try:
            can_type = get_message(type_name)
        except (ImportError, ValueError, AttributeError) as exc:
            # Not fatal on purpose. The pipeline should still come up and run
            # the baseline planner rather than refusing to start; a pipeline
            # that will not launch in the vehicle is harder to diagnose than
            # one that launches and says why it is quiet.
            self.get_logger().error(
                f'live_odom_node: cannot resolve message type {type_name!r}: {exc}. '
                f'The ads_messages package has to be on AMENT_PREFIX_PATH inside '
                f'the container. Publishing nothing; run with '
                f'planner_mode:=baseline until this resolves.')
        else:
            self.create_subscription(can_type, self.topic, self.on_can, LATEST_ONLY)
            self.get_logger().info(
                f'live_odom_node: subscribed to {self.topic} ({type_name}), '
                f'stale after {self.stale_timeout_s:.2f}s')

        # Report health once a second. A node that receives nothing and a node
        # whose every message is flagged look identical from outside, and
        # either one is a plausible failure when bringing this up on a new
        # vehicle.
        self.create_timer(1.0, self._report)

    def on_can(self, msg):
        """Pull the two signals out and advance the dead-reckoned pose.

        Callbacks are serialized by the default single-threaded executor, so
        this and on_frame never overlap and no lock is needed. Switching to a
        MultiThreadedExecutor would change that.
        """
        self.received += 1

        if self.trust_flags and self._flagged(msg):
            self.flagged += 1
            return

        try:
            speed = float(msg.spd_mps)
            yaw_rate = float(msg.imu.imu.angular_velocity.z)
        except AttributeError as exc:
            self.get_logger().warn(
                f'live_odom_node: {self.topic} message is missing a field this '
                f'node needs ({exc}). Expected spd_mps and '
                f'imu.imu.angular_velocity.z, as recorded in the Oct 2026 bags.')
            return

        self._check_units(speed, yaw_rate)

        # Same arithmetic as extract_can.py:
        #     heading += yaw_rate * dt
        #     x       += speed * cos(heading) * dt
        #     y       += speed * sin(heading) * dt
        #
        # dt comes from arrival time rather than msg.header.stamp, matching
        # extract_can.py, which used the bag's record time. The sender's own
        # stamp would be the better clock if it is reliably populated and
        # synced; that is worth revisiting once there is a real bus to look at.
        now = time.monotonic()
        self.speed = speed
        self.yaw_rate = yaw_rate
        self.last_can_t = now
        self.used += 1

        if self.last_integrate_t is None:
            self.last_integrate_t = now
            return
        dt = now - self.last_integrate_t
        self.last_integrate_t = now
        # A long gap means the topic stalled or the process was starved.
        # Integrating across it would invent motion that did not happen.
        if dt <= 0.0 or dt > self.stale_timeout_s:
            return
        self.heading += self.yaw_rate * dt
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

    @staticmethod
    def _flagged(msg):
        """True if the vehicle is telling us not to trust this sample.

        Read defensively: a field that is absent must not take the node down,
        and absence is treated as "no complaint" rather than "bad".
        """
        if getattr(msg, 'error', False) or getattr(msg, 'spd_error', False):
            return True
        imu = getattr(msg, 'imu', None)
        return imu is not None and not getattr(imu, 'valid', True)

    def _check_units(self, speed, yaw_rate):
        if self.warned_units:
            return
        if abs(yaw_rate) > MAX_PLAUSIBLE_YAW_RAD_S:
            self.warned_units = True
            self.get_logger().error(
                f'live_odom_node: yaw rate {yaw_rate:.1f} is past anything a road '
                f'vehicle does ({MAX_PLAUSIBLE_YAW_RAD_S} rad/s). This is almost '
                f'certainly deg/s being published into a rad/s field, which is '
                f'57x too large. The grid will think the vehicle is spinning.')
        elif abs(speed) > MAX_PLAUSIBLE_SPEED_M_S:
            self.warned_units = True
            self.get_logger().error(
                f'live_odom_node: speed {speed:.1f} is past '
                f'{MAX_PLAUSIBLE_SPEED_M_S} m/s. Check whether spd_mps is '
                f'actually carrying km/h.')

    def on_frame(self, msg):
        fresh = (self.last_can_t > 0.0
                 and (time.monotonic() - self.last_can_t) <= self.stale_timeout_s)
        if not fresh:
            # Deliberately publishing nothing rather than a stale pose.
            #
            # OPEN QUESTION FOR HATCI - see QUESTIONS.md. Nothing downstream
            # currently detects missing odometry: 5_planner_node sets
            # VehicleState.valid=True on every message it receives, so a stale
            # pose would be consumed as if it were current. Staying silent
            # makes the grid fall back to forgetting, which is the conservative
            # failure. A real vehicle probably wants the planner to hold or
            # hand back control instead, and that is a decision above this node.
            return

        o = Odometry()
        o.header.stamp = msg.header.stamp      # the frame's stamp, never now()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.heading / 2.0), w=math.cos(self.heading / 2.0))
        o.twist.twist.linear.x = self.speed
        o.twist.twist.angular.z = self.yaw_rate
        self.pub.publish(o)

    def _report(self):
        received, used, flagged = self.received, self.used, self.flagged
        self.received = self.used = self.flagged = 0
        fresh = (self.last_can_t > 0.0
                 and (time.monotonic() - self.last_can_t) <= self.stale_timeout_s)

        if received == 0:
            self.get_logger().warn(
                f'live_odom_node: nothing on {self.topic} in the last second. '
                f'Check `ros2 topic hz {self.topic}` and that ROS_DOMAIN_ID '
                f'matches the publisher - the container runs with --network '
                f'host, so a topic visible on the host is visible here.')
        elif used == 0:
            self.get_logger().warn(
                f'live_odom_node: {received} msg/s on {self.topic} but none '
                f'usable ({flagged} carried error/spd_error/imu.valid flags). '
                f'If those flags are set permanently on a healthy vehicle, '
                f'relaunch with trust_flags:=false and tell us.')
        else:
            self.get_logger().info(
                f'live_odom_node: {used}/{received} msg/s used, '
                f'speed {self.speed:.2f} m/s, '
                f'yaw {math.degrees(self.yaw_rate):+.1f} deg/s'
                + (f', {flagged} flagged' if flagged else '')
                + ('' if fresh else ' [STALE - not publishing]'))


def main(args=None):
    rclpy.init(args=args)
    node = LiveOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
