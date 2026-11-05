"""
The planner stops trusting a pose once odometry goes quiet.

This is the defect open item 1.8 describes. `on_odom` used to store the ego
pose with `valid=True` and nothing ever expired it, so a dead odometry source
left the BEV grid registering new masks against a position the vehicle had
already left. Observations from different real places piled into the same
cells, which corrupts the map rather than emptying it: the worse of the two
failures, and the opposite of what the code claimed.

`bev_grid.ground_pose()` already does the right thing when handed a state it
cannot register with, returning None so the grid forgets. The node simply never
let that happen. These tests pin the expiry, and pin the two cases where it
must NOT fire.

Checked against a stub clock rather than a running node, because none of this
logic needs a model, a camera or a display.
"""

import importlib.util
import pathlib

import pytest

PKG = pathlib.Path(__file__).resolve().parent.parent
NODES = PKG / 'perception_suite'

pytest.importorskip('rclpy')
pytest.importorskip('perception_suite_msgs.msg')


def _load(stem):
    """Import a node module whose name starts with a digit."""
    spec = importlib.util.spec_from_file_location(
        stem.replace('_node', '_node_mod'), NODES / f'{stem}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    planner = _load('5_planner_node')
except Exception as exc:                                      # noqa: BLE001
    pytest.skip(f'node module not importable: {exc}', allow_module_level=True)

from offroad_autonomy.planning.bev_grid import ground_pose    # noqa: E402
from offroad_autonomy.types import VehicleState               # noqa: E402

TIMEOUT_S = 0.5


class _Clock:
    def __init__(self):
        self.nanos = 1_000_000_000

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return self.nanos

    def advance(self, seconds):
        self.nanos += int(seconds * 1e9)


class _StubNode:
    """Just enough of the planner for the expiry rule."""

    def __init__(self, with_odom=True):
        self.clock = _Clock()
        self.odom_timeout_s = TIMEOUT_S
        self._warned_stale_odom = False
        self.last_odom_ns = None
        self.vehicle_state = VehicleState(speed_mps=4.0)
        if with_odom:
            self.vehicle_state = VehicleState(
                position=(10.0, 2.0, 0.0), speed_mps=4.0,
                heading_rad=0.3, direction=(0.955, 0.295, 0.0), valid=True)
            self.last_odom_ns = self.clock.nanoseconds

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self

    def error(self, *a, **k):
        pass


def _state(stub):
    return planner.PlannerNode._usable_state(stub)


def test_a_fresh_pose_is_used_and_the_grid_can_register_with_it():
    stub = _StubNode()
    stub.clock.advance(TIMEOUT_S * 0.5)
    assert _state(stub).valid is True
    assert _state(stub).direction is not None


def test_a_stale_pose_is_dropped_so_the_grid_forgets():
    """The whole point: corrupting the map is worse than emptying it."""
    stub = _StubNode()
    stub.clock.advance(TIMEOUT_S * 2)
    state = _state(stub)
    assert state.direction is None, 'a stale pose must not be registrable'
    assert state.speed_mps == pytest.approx(4.0), 'last speed is still the best estimate'


def test_no_odometry_at_all_is_not_treated_as_stale():
    """Fixed speed_mps with no CAN source is supported, not a fault."""
    stub = _StubNode(with_odom=False)
    stub.clock.advance(TIMEOUT_S * 100)
    assert _state(stub) is stub.vehicle_state


def test_the_dropped_pose_actually_makes_ground_pose_return_none():
    """Pin the behaviour this is all for, not just the field values."""
    class _Camera:
        position = (0.0, 0.0, 0.0)

    camera = _Camera()
    fresh = _StubNode()
    assert ground_pose(_state(fresh), camera) is not None

    stale = _StubNode()
    stale.clock.advance(TIMEOUT_S * 2)
    assert ground_pose(_state(stale), camera) is None
