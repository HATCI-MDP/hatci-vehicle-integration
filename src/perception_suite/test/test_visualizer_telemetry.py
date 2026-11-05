"""
The dashboard's live readouts, and the freshness rule behind them.

6_visualizer_node holds the newest control command and the newest odometry
sample and draws them on the dials. The interesting part is not the holding,
it is the letting go: if the controller or the odometry source stops
publishing, a held value keeps a dial showing something nothing is commanding
any more, which reads as live when it is not. Zero is the honest answer, and
it is also what a graph with no controller shows, so the two agree.

Checked here with a stub clock rather than a running node, because the node
needs an X display and a model to construct, and none of this logic does.
"""

import importlib.util
import pathlib

import pytest

PKG = pathlib.Path(__file__).resolve().parent.parent
NODES = PKG / 'perception_suite'

pytest.importorskip('rclpy')
pytest.importorskip('perception_suite_msgs.msg')
pytest.importorskip('cv2')

from rclpy.time import Time                                   # noqa: E402


def _load(stem):
    """Import a node module whose name starts with a digit."""
    spec = importlib.util.spec_from_file_location(
        stem.replace('_node', '_node_mod'), NODES / f'{stem}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    visualizer = _load('6_visualizer_node')
except Exception as exc:                                      # noqa: BLE001
    pytest.skip(f'node module not importable: {exc}', allow_module_level=True)

TIMEOUT_S = 0.5


class _StubNode:
    """Just enough of the node for the freshness methods, with a clock we drive."""

    def __init__(self):
        self._command = visualizer.ControlCommand()
        self._command_at = None
        self._speed_mps = 0.0
        self._speed_at = None
        self.command_timeout_s = TIMEOUT_S
        self.odom_timeout_s = TIMEOUT_S
        self.nanos = 1_000_000_000

    def get_clock(self):
        return self

    def now(self):
        return Time(nanoseconds=self.nanos)

    def advance(self, seconds):
        self.nanos += int(seconds * 1e9)


def _command(steering=0.25, throttle=0.5, brake=0.125):
    msg = visualizer.ControlCommandMsg()
    msg.steering = steering
    msg.throttle = throttle
    msg.brake = brake
    return msg


def _odom(speed_mps):
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.twist.twist.linear.x = speed_mps
    return msg


def test_dials_read_zero_before_anything_has_been_published():
    """A six-node graph never publishes command, and that is not a fault."""
    stub = _StubNode()
    assert visualizer.VisualizationNode._current_command(stub).throttle == 0.0
    assert visualizer.VisualizationNode._current_speed_mph(stub) == 0.0


def test_a_fresh_command_reaches_the_dials():
    """What the controller published is what the dashboard draws."""
    stub = _StubNode()
    visualizer.VisualizationNode._command_callback(stub, _command())
    current = visualizer.VisualizationNode._current_command(stub)
    assert current.steering == pytest.approx(0.25)
    assert current.throttle == pytest.approx(0.5)
    assert current.brake == pytest.approx(0.125)


def test_the_dials_drop_to_zero_once_the_controller_goes_quiet():
    """The whole point of the file: a stopped controller must not look live."""
    stub = _StubNode()
    visualizer.VisualizationNode._command_callback(stub, _command())

    stub.advance(TIMEOUT_S * 0.9)
    assert visualizer.VisualizationNode._current_command(stub).throttle == pytest.approx(0.5)

    stub.advance(TIMEOUT_S * 0.2)
    gone = visualizer.VisualizationNode._current_command(stub)
    assert gone.steering == 0.0
    assert gone.throttle == 0.0
    assert gone.brake == 0.0


def test_speed_is_converted_to_mph_and_also_expires():
    """The speedometer is in mph; odometry publishes metres per second."""
    stub = _StubNode()
    visualizer.VisualizationNode._odom_callback(stub, _odom(10.0))
    assert visualizer.VisualizationNode._current_speed_mph(stub) == pytest.approx(22.369, abs=1e-3)

    stub.advance(TIMEOUT_S * 2)
    assert visualizer.VisualizationNode._current_speed_mph(stub) == 0.0
