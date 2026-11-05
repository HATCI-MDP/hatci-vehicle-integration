"""
The planner-to-controller message round trips, and does it the fast way.

Two separate things are checked here, and the second is the reason this file
exists.

Correctness: what 5_planner_node encodes is what 7_controller_node decodes. The
centreline must survive in IMAGE PIXELS, because the controller projects it
itself; a round trip that quietly lost the pixel values would leave the
controller steering from the wrong units with no error anywhere.

Speed: rosidl's generated setters validate a Python list element by element. On
the bench Jetson a 640x413 mask costs 49.5 ms assigned as a list and 0.05 ms
assigned as an array.array. The list version was written first, put the planner
at 61 ms per frame and capped the whole pipeline at 15.9 Hz, with no error and
nothing in any log to say why. The rate is the only symptom, so the type is
asserted here rather than left to a reviewer to notice.

Needs ROS and the generated messages, so it skips where they are absent.
"""

import array
import importlib.util
import pathlib

import pytest

PKG = pathlib.Path(__file__).resolve().parent.parent
NODES = PKG / 'perception_suite'


def _load(stem):
    """Import a node module whose name starts with a digit."""
    spec = importlib.util.spec_from_file_location(
        stem.replace('_node', '_node_mod'), NODES / f'{stem}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


np = pytest.importorskip('numpy')
pytest.importorskip('rclpy')
pytest.importorskip('perception_suite_msgs.msg')

try:
    planner = _load('5_planner_node')
    controller = _load('7_controller_node')
except Exception as exc:                                    # noqa: BLE001
    pytest.skip(f'node modules not importable: {exc}', allow_module_level=True)

from offroad_autonomy.types import PathPlan                 # noqa: E402
from std_msgs.msg import Header                             # noqa: E402


def _plan():
    rng = np.random.default_rng(0)
    mask = rng.random((37, 53)) > 0.5
    return PathPlan(
        centerline=np.array([[12.5, 300.0], [13.25, 290.0], [14.0, 280.0]]),
        min_clearance_m=2.75,
        fallback_active=True,
        fallback_reason='narrow',
        speed_scale=0.5,
        planner_mask=mask,
        roi_top=42)


def test_round_trip_preserves_every_field_the_controller_reads():
    original = _plan()
    message = planner.PlannerNode._plan_message(original, Header())
    decoded = controller.ControllerNode._plan_from_message(message)

    # float32 on the wire, so compare at float32 precision rather than exactly.
    np.testing.assert_allclose(
        decoded.centerline, original.centerline, rtol=0, atol=1e-4)
    np.testing.assert_array_equal(decoded.planner_mask, original.planner_mask)
    assert decoded.speed_scale == pytest.approx(original.speed_scale)
    assert decoded.min_clearance_m == pytest.approx(original.min_clearance_m)
    assert decoded.fallback_active is True
    assert decoded.fallback_reason == 'narrow'
    assert decoded.roi_top == 42


def test_sequence_fields_are_array_array_not_lists():
    """A list here is correct and 1000x slower. See the module docstring."""
    message = planner.PlannerNode._plan_message(_plan(), Header())
    assert isinstance(message.planner_mask, array.array), (
        'planner_mask must be assigned as array.array; a list costs 49.5 ms '
        'per frame on the bench Jetson and silently halves the pipeline rate')
    assert message.planner_mask.typecode == 'B'
    for name in ('centerline_u', 'centerline_v'):
        field = getattr(message, name)
        assert isinstance(field, array.array), f'{name} must be array.array'
        assert field.typecode == 'f'


def test_empty_plan_round_trips_as_an_empty_centerline():
    """An empty centreline means stop, so it has to survive the round trip."""
    empty = PathPlan(centerline=np.empty((0, 2)), planner_mask=None)
    decoded = controller.ControllerNode._plan_from_message(
        planner.PlannerNode._plan_message(empty, Header()))
    assert len(decoded.centerline) == 0
    assert decoded.planner_mask is None


def test_infinite_clearance_survives():
    """The planner reports inf when it finds no constriction."""
    plan = PathPlan(centerline=np.empty((0, 2)), min_clearance_m=float('inf'))
    decoded = controller.ControllerNode._plan_from_message(
        planner.PlannerNode._plan_message(plan, Header()))
    assert decoded.min_clearance_m == float('inf')
