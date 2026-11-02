"""Replay the bags' CAN odometry, one sample per published frame.

The BEV grid needs to register consecutive frames against each other, which
needs ego motion. The bags carry it at 30 Hz - speed from the wheels, yaw rate
from the IMU - but the pipeline plays re-encoded mp4 clips that have no CAN, so
the two are rejoined here.

Pairing is by FRAME COUNT, not wall clock: one CAN row is published for each
frame on raw_frames. The clip and the CAN came from the same recording at the
same 30 Hz, so index N matches index N, and a pipeline that stalls or runs slow
stays in sync instead of drifting against a free-running timer.

Position and heading are dead-reckoned in extract_can.py by integrating speed
along integrated yaw. Checked against the bags' own GPS: path length within
0.6-1.3% over ~2 km, median position error 0.3-0.6 m over the 1-2 s the grid
actually remembers. Absolute drift over a whole run does not matter - the grid
only ever compares consecutive poses and decays everything older than memory_s.
"""

import csv
import math
import signal

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from perception_suite.common import LATEST_ONLY


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        path = self.declare_parameter('can_csv', '').value
        if not path:
            raise ValueError('odom_node needs can_csv:=<path to an extracted CAN csv>')
        # Looping clips restart, so the odometry has to restart with them.
        self.loop = bool(self.declare_parameter('loop', True).value)

        with open(path, newline='', encoding='utf-8') as fh:
            self.rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fh)]
        if not self.rows:
            raise ValueError(f'{path} has no rows')
        self.n = 0

        self.pub = self.create_publisher(Odometry, 'odom', LATEST_ONLY)
        self.create_subscription(Image, 'raw_frames', self.on_frame, LATEST_ONLY)
        # The span, not the last timestamp: a csv trimmed out of the middle of
        # a bag starts at t=40, and reporting that as a duration made 600 rows
        # of 30 Hz CAN look like 10 Hz.
        span = self.rows[-1]['t'] - self.rows[0]['t']
        self.get_logger().info(
            f'{len(self.rows)} CAN samples over {span:.0f}s '
            f'({(len(self.rows) - 1) / span if span > 0 else 0:.1f} Hz) from {path}')

    def on_frame(self, msg):
        if self.n >= len(self.rows):
            if not self.loop:
                return
            self.n = 0
            self.get_logger().info('CAN replay wrapped with the clip')
        r = self.rows[self.n]
        self.n += 1

        o = Odometry()
        # The frame's own stamp, not now(): the planner pairs this with the mask
        # derived from that frame, and the pipeline is several stages deep.
        o.header.stamp = msg.header.stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = r['x']
        o.pose.pose.position.y = r['y']
        h = r['heading']
        o.pose.pose.orientation = Quaternion(x=0.0, y=0.0,
                                             z=math.sin(h / 2.0), w=math.cos(h / 2.0))
        o.twist.twist.linear.x = r['speed_mps']
        o.twist.twist.angular.z = r['yaw_rate']
        self.pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(OdomNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
