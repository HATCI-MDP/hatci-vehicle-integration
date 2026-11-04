"""Helpers shared by the pipeline stage nodes.

Masks travel between stages as mono8 images whose road pixels hold the
frame's best detection confidence. That keeps the one number the perception
gate reads attached to the mask it belongs to, without a custom message
package.
"""

import time
from dataclasses import replace

import numpy as np
from offroad_autonomy.runtime.timing import RuntimeStats
from offroad_autonomy.types import PipelineConfig
from offroad_autonomy.utils.config import load_config
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from perception_suite import settings

# A stage that falls behind skips to the newest message instead of working
# through a backlog.
LATEST_ONLY = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# For topics published by something OUTSIDE this pipeline, where we do not get
# to choose the publisher's QoS. BEST_EFFORT here is not a preference, it is
# the only setting that matches either kind of publisher: DDS refuses to
# connect a RELIABLE reader to a BEST_EFFORT writer, because the reader is
# asking for a guarantee the writer never offered, and it refuses SILENTLY.
# A BEST_EFFORT reader matches both.
#
# Measured, not theorised: replaying HATCI's bags publishes /CAN as
# BEST_EFFORT while publishing the image topic as RELIABLE, because a bag
# replays the QoS it recorded per topic. A RELIABLE subscription to /CAN saw
# nothing at all while `ros2 topic hz /CAN` showed 15 Hz.
EXTERNAL_LATEST = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


#: Beyond this, the stamp is not on our clock, so the difference is not latency.
_MAX_PLAUSIBLE_AGE_S = 10.0

#: Five seconds rather than one. Four stages reporting every second drowns the
#: entrypoint's own output, and nothing here changes fast enough to need it.
_REPORT_PERIOD_S = 5.0


class StageTimer:
    """Per-stage timing for one pipeline node, logged periodically.

    Two numbers, because they answer different questions.

    ``work``
        How long this node's own callback took. Always meaningful.

    ``age``
        How old the frame was by the time this node finished with it,
        measured from the capture stamp that every stage copies forward.
        Subtracting one stage's age from the next gives that stage's whole
        contribution, transport included, which is the thing a single
        end-to-end number cannot tell you.

    ``age`` only means something when the stamp came from this clock. Under
    bag replay it is the bag's recorded time, hours in the past, so
    implausible values are dropped and the reason is said once rather than
    reported as latency.
    """

    def __init__(self, node: Node, stage: str) -> None:
        self._node = node
        self._stage = stage
        self._stats = RuntimeStats()
        self._warned_clock = False
        node.create_timer(_REPORT_PERIOD_S, self._report)

    @staticmethod
    def start() -> float:
        return time.perf_counter()

    def done(self, t0: float, header, extra: dict[str, float] | None = None) -> None:
        self._stats.tick()
        self._stats.record('work', (time.perf_counter() - t0) * 1000.0)
        if extra:
            self._stats.record_many(extra)

        elapsed = self._node.get_clock().now() - Time.from_msg(header.stamp)
        age_s = elapsed.nanoseconds / 1e9
        if 0.0 <= age_s <= _MAX_PLAUSIBLE_AGE_S:
            self._stats.record('age', age_s * 1000.0)
            return
        if not self._warned_clock:
            self._warned_clock = True
            self._node.get_logger().info(
                f'{self._stage}: capture stamps are not on this clock '
                f'(first sample {age_s:.1f}s), which is normal for bag replay. '
                'Reporting work time only.')

    def _report(self) -> None:
        work = self._stats.stage('work')
        if work.count == 0:
            return
        parts = [f'{self._stage} {self._stats.fps():5.1f} Hz',
                 f'work p50 {work.p50_ms:5.1f} p95 {work.p95_ms:5.1f} ms']
        for name in self._stats.stages():
            if name in ('work', 'age'):
                continue
            extra = self._stats.stage(name)
            parts.append(f'{name} p50 {extra.p50_ms:5.1f} ms')
        age = self._stats.stage('age')
        if age.count > 0:
            parts.append(f'age p50 {age.p50_ms:5.1f} p95 {age.p95_ms:5.1f} ms')
        self._node.get_logger().info('stage timing | ' + ' | '.join(parts))


def load_pipeline_config(node: Node) -> PipelineConfig:
    """The config:= file with the model:= entry applied. Every stage applies
    the model, because the planner and visualizer gate on its threshold."""
    path = settings.resolve(node.declare_parameter('config', settings.DEFAULT_CONFIG).value)
    name = node.declare_parameter('model', settings.DEFAULT_MODEL).value
    node.get_logger().info(f'Loading pipeline config: {path} (model: {name})')
    config = load_config(path)
    model = settings.model(path, name)
    if model is None:
        node.get_logger().info('Config has no perception.models; using its model_weights')
        return config
    return replace(config, model_weights=str(model['weights']),
                   gate_min_confidence=float(model['gate_confidence_threshold']))


def encode_mask(mask: np.ndarray, confidence: float) -> np.ndarray:
    """Road mask plus a frame-wide confidence, packed into one mono8 image.

    The confidence is the pixel level, so 0 means "not road". A non-zero
    confidence that rounds to 0 is clamped to 1, otherwise a real but very low
    confidence would be indistinguishable from background and the whole mask
    would decode as empty.
    """
    level = int(round(float(confidence) * 255.0))
    level = min(255, max(0, level))
    if level == 0 and confidence > 0.0:
        level = 1
    return np.asarray(mask, dtype=bool).astype(np.uint8) * np.uint8(level)


def decode_mask(confidence_mask: np.ndarray) -> tuple[np.ndarray, float]:
    return confidence_mask > 0, float(confidence_mask.max(initial=0)) / 255.0
