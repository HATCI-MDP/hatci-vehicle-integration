"""Stage 1: H.264 frames from a compressed image topic, published as /raw_frames.

Interchangeable with 1_video_node and 1_camera_node: same topic, same encoding,
same downscale. One deliberate difference, and it is the reason this node
exists: the other two stamp each frame with the current clock, because an mp4
and a V4L2 capture carry no capture time. This node copies the INCOMING header
through untouched, so every latency figure measured downstream is relative to
when the frame was actually captured on the vehicle rather than when we got
round to republishing it.

This is not a bag reader. It subscribes to a topic, and `ros2 bag play` is only
one thing that can feed it. That matters because if HATCI's live camera node
publishes the same topic it recorded, this node is also the vehicle input path
and not just a test fixture. See item 3.8 in OPEN-ITEMS.md, which is the
question that decides it.

Why a GStreamer subprocess rather than cv2.imdecode
---------------------------------------------------
The payload is H.264, not JPEG. `cv2.imdecode` handles still images only and
returns None on every frame. The stream is also IPPP: most messages are a
single non-IDR slice and only about one in eight carries SPS+PPS+IDR, so frames
are NOT independently decodable. The decoder has to hold state and be fed in
order, which rules out any per-message decode.

Three hard-won details, each of which silently corrupts output if ignored:

1. Frames go over a NAMED FIFO, never the subprocess's stdout. The NVIDIA
   decoder plugin prints "Opening in BLOCKING MODE" to stdout itself, which is
   not a GStreamer bus message and so survives -q. Measured: 26 bytes of that
   text landed in the frame stream and permanently desynchronised a
   fixed-size-chunk reader. Over a FIFO only filesink's bytes can arrive, and
   the byte count then comes out exactly frame-aligned.

2. The output height must be EVEN. nvvidconv pads rows otherwise: at 640x413,
   the aspect-correct size, 120 frames came out 12,026 bytes long. So the
   height is rounded to even, which is why this node's frames can be one pixel
   taller than the other two sources'. Harmless, because 640xH is only a
   bandwidth reduction and 2_preprocess_node resizes again anyway.

3. A dropped message is not a dropped frame, it is corruption until the next
   keyframe, and it corrupts visually rather than raising anything. So the
   subscription is RELIABLE with a deep queue, nothing is ever dropped on
   purpose, and the node watches the frame counter in the incoming frame_id
   and complains loudly about any gap.

Measured throughput, and why it is below 30 FPS
-----------------------------------------------
On the bench Orin NX, clocks pinned. This node alone, feeding from a bag
replayed at 1.0:

    appsink      29.9 FPS
    subprocess   16-22 FPS      (varies with warmup and measurement window)

Whole mode-2 pipeline, all four bag topics replayed including /CAN, appsink:

    rate 0.5    road_mask 14.6, odom 15.1, CAN 15.0,  0 frame gaps
    rate 0.8    road_mask 24.4, odom 24.0, CAN 24.0,  2 frame gaps
    rate 1.0    road_mask 29.8, odom 30.3, CAN 30.0,  6 frame gaps

Read that table for throughput only. The lower rates are NOT better runs; see
"bag_rate defaults to 1.0" below.

Same pipeline on the subprocess backend reached about 10 FPS, so appsink is
roughly a 3x improvement and full 30 FPS replay is now reachable.

The gaps are in DDS delivery rather than in decoding: the replayer publishes
KEEP_LAST, so when our executor is momentarily busy the writer overwrites its
history instead of waiting. Closing that would mean a multi-threaded executor,
which is real work with its own risks, and mode 2 exists to prove the stages
run on real recorded data rather than to produce a frame-rate figure.

bag_rate defaults to 1.0 and should stay there
----------------------------------------------
An earlier version of this file argued the opposite, that 0.5 was the safe
default because it loses no frames. That was wrong, and wrong in the direction
that matters: a lower rate is not a slower version of the same run, it is a
different and incorrect one.

live_odom_node integrates dead reckoning on wall-clock arrival time, and
centerline_planner decays the BEV grid on wall-clock as well. Neither reads
the recorded stamps. Replaying at rate r spaces the messages out by 1/r while
the speed and yaw values on them stay the same, so the integrated motion is
scaled by 1/r:

    rate 1.0   /CAN 30 Hz   dt 33.3 ms   0.250 m per step at 7.5 m/s   correct
    rate 0.8   /CAN 24 Hz   dt 41.7 ms   0.312 m                       1.25x
    rate 0.5   /CAN 15 Hz   dt 66.7 ms   0.500 m                       2.00x

At 0.5 the vehicle appears to travel twice as far as it did, the accumulated
grid is stretched 2x along the direction of travel, and curvature therefore
reads as half what it is. The arc chosen off that grid is chosen from wrong
geometry. Nothing warns you, which is what makes it worse than the frame loss.

So 1.0 is the only rate at which mode 2 is geometrically correct, and the
roughly 0.4% of frames it loses is the cheaper of the two errors. Do not lower
bag_rate to silence the frame-gap warnings.

The real fix is to integrate on msg.header.stamp instead of the wall clock,
which would make the replay path rate-independent. It is not done here because
it depends on the live bus populating those stamps, which is unconfirmed and
is item 1.9 in OPEN-ITEMS.md. On the bags themselves the stamps are good.

For reference on where the subprocess backend's time went: decoding the same
frames straight from a file with gst-launch runs at about 3.5x realtime, so
the decoder was never the limit. It was the transport - a pipe write and a
FIFO read per frame, plus a copy - which is exactly what appsink removes.

That is acceptable here, and deliberately so. Replay exists to prove every
stage works on real recorded data, which is HATCI's requirement 1, and it
replays losslessly at whatever rate the machine sustains. It is NOT where a
frame rate or latency figure should come from: those belong to the camera and
clip paths, because a bag's rate is set by the replayer and by this decoder
rather than by the pipeline. Quoting a bag-path FPS number would understate
the pipeline by roughly half.

Two decode backends
-------------------
decode_backend:=auto picks the first that works.

  appsink      In-process GStreamer through appsrc/appsink. No child process,
               no pipe, no FIFO, and one fewer copy per frame. Needs python3-gi
               plus the Gst and GstApp typelibs.
  subprocess   gst-launch in a child process, frames back over a named FIFO.
               Needs nothing beyond what the base image already carries, and is
               the fallback when python3-gi is absent.

Both share the same bounded queue and writer/reader threads, so the
backpressure behaviour above is identical either way. Only the transport
between this process and the decoder differs.
"""
import os
import queue
import shutil
import signal
import subprocess
import tempfile
import threading

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

NAL_IDR = 5
NAL_SPS = 7

# Probed once, here, so the node can report which backend it will use before it
# starts doing anything. An absent python3-gi is not an error: it selects the
# subprocess backend, which needs nothing extra.
try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp  # noqa: F401

    HAVE_GI = True
except (ImportError, ValueError):
    HAVE_GI = False


def nal_units(buf):
    """Yield H.264 NAL unit type codes from an Annex-B byte string."""
    i, n = 0, len(buf)
    while i < n - 4:
        if buf[i] == 0 and buf[i + 1] == 0:
            if buf[i + 2] == 1:
                yield buf[i + 3] & 0x1F
                i += 4
                continue
            if buf[i + 2] == 0 and buf[i + 3] == 1 and i + 4 < n:
                yield buf[i + 4] & 0x1F
                i += 5
                continue
        i += 1


class BagPublisher(Node):
    """Decode an H.264 image topic and republish it as raw_frames."""

    def __init__(self):
        super().__init__("bag_node")

        topic = self.declare_parameter(
            "image_topic", "/image_processing/image_compressed").value
        # Launch substitutions arrive as strings, so everything is cast.
        self.width = int(self.declare_parameter("width", "1280").value)
        src_w = int(self.declare_parameter("source_width", "2880").value)
        src_h = int(self.declare_parameter("source_height", "1860").value)
        self.decoder = self.declare_parameter(
            "decoder", "nvv4l2decoder").value
        backend = self.declare_parameter("decode_backend", "auto").value
        depth = int(self.declare_parameter("queue_depth", "200").value)

        # Even height: see point 2 in the module docstring.
        self.height = 2 * round(self.width * src_h / src_w / 2)
        self.frame_bytes = self.width * self.height * 3

        self.publisher_ = self.create_publisher(Image, "raw_frames", 10)
        self.bridge = CvBridge()

        # Headers wait here while their frame is inside the decoder. Decode is
        # in order and one frame out per frame in, so the oldest header belongs
        # to the next frame out. That holds for IPPP; a stream with B-frames
        # would reorder output and mispair timestamps, which is worth knowing
        # if these numbers ever look subtly wrong.
        self.headers = queue.Queue()
        # Bounded, and a full queue BLOCKS the subscription callback rather
        # than dropping. That backpressure is the whole point: see _on_frame.
        self.feed = queue.Queue(maxsize=90)

        self.seen_idr = False
        self.last_counter = None
        self.gaps = 0
        self.decoded = 0

        if backend == "auto":
            backend = "appsink" if HAVE_GI else "subprocess"
        if backend == "appsink" and not HAVE_GI:
            self.get_logger().warn(
                "decode_backend:=appsink was asked for but python3-gi is not "
                "installed; falling back to the subprocess backend.")
            backend = "subprocess"
        self.backend = backend

        if backend == "appsink":
            self._start_appsink()
        elif backend == "subprocess":
            self._start_subprocess()
        else:
            raise RuntimeError(
                f"decode_backend must be auto, appsink or subprocess, "
                f"got {backend!r}")

        self.create_subscription(
            CameraInfo, topic.rsplit("/", 1)[0] + "/camera_info",
            self._on_info, 10)
        self.create_subscription(
            CompressedImage, topic, self._on_frame,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=depth))

        self.get_logger().info(
            f"Decoding '{topic}' with {self.decoder} via the {self.backend} "
            f"backend, publishing {self.width}x{self.height} on raw_frames")
        self.get_logger().info(
            "Waiting for the first keyframe; an H.264 stream cannot be "
            "decoded from the middle.")

    # -- decoder: shared plumbing -------------------------------------------
    def _caps_tail(self):
        """Pipeline tokens after the decoder, identical in both backends.

        A token list rather than a string, so the subprocess backend can hand
        it straight to argv without splitting it back apart. An earlier version
        returned a string and rebuilt the list by index, which silently dropped
        the "!" before filesink and produced a pipeline that ran and emitted
        nothing.
        """
        conv = "nvvidconv" if self.decoder == "nvv4l2decoder" else "videoscale"
        return [
            conv, "!",
            f"video/x-raw,format=BGRx,width={self.width},height={self.height}",
            "!", "videoconvert", "!", "video/x-raw,format=BGR",
        ]

    def _start_threads(self):
        self.running = True
        threading.Thread(target=self._writer, daemon=True).start()
        threading.Thread(target=self._pull, daemon=True).start()

    def _writer(self):
        """Feed access units to the decoder off the ROS callback thread."""
        while self.running:
            try:
                data = self.feed.get(timeout=0.5)
            except queue.Empty:
                continue
            if data is None:
                break
            if not self._push(data):
                self.running = False
                return

    # -- decoder: in-process, via appsrc/appsink ----------------------------
    def _start_appsink(self):
        # Without this every element factory call asserts gst_is_initialized()
        # and the process segfaults. Importing Gst does NOT initialise it, and
        # the failure is a hard crash rather than an exception, so there is no
        # falling back to the subprocess backend from here. Safe to call more
        # than once.
        Gst.init(None)

        # alignment=au says each buffer we push is exactly one access unit,
        # which is true because each ROS message holds one frame. Without it
        # h264parse has to rescan for boundaries we already know.
        desc = (
            "appsrc name=src is-live=true format=time "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
            f"h264parse ! {self.decoder} ! {' '.join(self._caps_tail())} ! "
            "appsink name=sink max-buffers=4 drop=false sync=false"
        )
        self.pipeline = Gst.parse_launch(desc)
        self.src = self.pipeline.get_by_name("src")
        self.sink = self.pipeline.get_by_name("sink")
        self.bus = self.pipeline.get_bus()
        self.pipeline.set_state(Gst.State.PLAYING)
        self._start_threads()

    def _push_appsink(self, data):
        ret = self.src.push_buffer(Gst.Buffer.new_wrapped(data))
        if ret != Gst.FlowReturn.OK:
            self.get_logger().error(f"appsrc rejected a buffer: {ret}. Stopping.")
            return False
        return True

    def _pull_appsink(self):
        """Pull decoded frames out of appsink. One copy, no process boundary."""
        while self.running:
            self._drain_bus()
            sample = self.sink.try_pull_sample(Gst.SECOND // 2)
            if sample is None:
                continue
            gbuf = sample.get_buffer()
            ok, info = gbuf.map(Gst.MapFlags.READ)
            if not ok:
                self.get_logger().warn("Could not map a decoded buffer.")
                continue
            try:
                if info.size != self.frame_bytes:
                    self.get_logger().error(
                        f"Decoded buffer is {info.size} bytes, expected "
                        f"{self.frame_bytes}. Geometry is wrong; stopping.")
                    self.running = False
                    return
                # A view, not a copy. cv2_to_imgmsg copies into the message, so
                # the data is safe once _publish returns and before we unmap.
                self._publish(np.frombuffer(info.data, dtype=np.uint8).reshape(
                    self.height, self.width, 3))
            finally:
                gbuf.unmap(info)

    def _drain_bus(self):
        """GStreamer reports decode failures on the bus, not through the API."""
        while True:
            msg = self.bus.pop_filtered(Gst.MessageType.ERROR
                                        | Gst.MessageType.WARNING)
            if msg is None:
                return
            err, debug = (msg.parse_error() if msg.type == Gst.MessageType.ERROR
                          else msg.parse_warning())
            text = f"gstreamer: {err.message} ({debug})"
            if msg.type == Gst.MessageType.ERROR:
                self.get_logger().error(text)
            else:
                self.get_logger().warn(text)

    # -- decoder: child process, frames over a FIFO -------------------------
    def _start_subprocess(self):
        self.tmpdir = tempfile.mkdtemp(prefix="bag_node_")
        self.fifo = os.path.join(self.tmpdir, "video.raw")
        os.mkfifo(self.fifo)

        # filesink to a FIFO, never fdsink to stdout: the NVIDIA decoder plugin
        # prints "Opening in BLOCKING MODE" to stdout itself, which is not a
        # GStreamer bus message and so survives -q. Measured: 26 bytes of that
        # text in the frame stream, permanently misaligning a chunk reader.
        argv = (["gst-launch-1.0", "-q", "fdsrc", "!", "h264parse", "!",
                 self.decoder, "!"]
                + self._caps_tail()
                + ["!", "filesink", f"location={self.fifo}"])

        self.gst = subprocess.Popen(
            argv, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._start_threads()

    def _push_subprocess(self, data):
        try:
            self.gst.stdin.write(data)
            self.gst.stdin.flush()
            return True
        except (BrokenPipeError, ValueError):
            self.get_logger().error("Decoder closed its input; stopping.")
            return False

    def _pull_subprocess(self):
        """Read whole frames off the FIFO and publish them."""
        # Opening a FIFO for reading blocks until the writer opens it, which is
        # what synchronises us with the decoder starting up.
        with open(self.fifo, "rb") as fh:
            while self.running:
                buf = fh.read(self.frame_bytes)
                if not buf:
                    break
                if len(buf) != self.frame_bytes:
                    self.get_logger().error(
                        f"Short read: {len(buf)} of {self.frame_bytes} bytes. "
                        "The frame stream is misaligned; output after this "
                        "point is not trustworthy.")
                    break
                self._publish(np.frombuffer(buf, dtype=np.uint8).reshape(
                    self.height, self.width, 3))

    # -- publish ------------------------------------------------------------
    def _push(self, data):
        if self.backend == "appsink":
            return self._push_appsink(data)
        return self._push_subprocess(data)

    def _pull(self):
        if self.backend == "appsink":
            return self._pull_appsink()
        return self._pull_subprocess()

    def _publish(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        try:
            msg.header = self.headers.get_nowait()
        except queue.Empty:
            # More frames out than in should be impossible. Stamping with our
            # own clock here would quietly invent a capture time, so say so.
            self.get_logger().warn(
                "Decoded a frame with no header queued; timestamp is ours, "
                "not the capture's.", once=True)
            msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.decoded += 1

    # -- subscriptions ------------------------------------------------------
    def _on_info(self, msg):
        """Warn if the real stream geometry contradicts our parameters."""
        if (msg.width, msg.height) == (0, 0):
            return
        expected = 2 * round(self.width * msg.height / msg.width / 2)
        if expected != self.height:
            self.get_logger().warn(
                f"camera_info says {msg.width}x{msg.height}, so the "
                f"aspect-correct output height is {expected}, not "
                f"{self.height}. Set source_width/source_height to match.",
                once=True)

    def _on_frame(self, msg):
        data = bytes(msg.data)

        if not self.seen_idr:
            types = set(nal_units(data))
            if NAL_IDR not in types and NAL_SPS not in types:
                return
            self.seen_idr = True
            self.get_logger().info("Keyframe received, decoding.")

        self._check_continuity(msg.header.frame_id)

        self.headers.put(msg.header)
        # BLOCKING on purpose. Dropping here would corrupt the stream until the
        # next keyframe; blocking instead stalls this callback, which is what
        # pushes back through the RELIABLE subscription onto whatever is
        # publishing. Against `ros2 bag play` that means replay simply runs
        # slower than realtime, losslessly, which is the right trade for a
        # replay path. Measured: 0 discards at 22 FPS standalone.
        while self.running:
            try:
                self.feed.put(data, timeout=1.0)
                return
            except queue.Full:
                self.get_logger().warn(
                    "Decoder is saturated; holding the source back rather "
                    "than dropping a frame.", once=True)

    def _check_continuity(self, frame_id):
        """The counter in frame_id is the only way to see a lost frame."""
        try:
            counter = int(frame_id.rsplit("_", 1)[1])
        except (ValueError, IndexError, AttributeError):
            return
        if self.last_counter is not None and counter != self.last_counter + 1:
            self.gaps += 1
            self.get_logger().warn(
                f"Frame gap: {self.last_counter} -> {counter}. Expect visual "
                f"corruption until the next keyframe. ({self.gaps} so far)")
        self.last_counter = counter

    # -- teardown -----------------------------------------------------------
    def shutdown(self):
        self.running = False
        try:
            self.feed.put_nowait(None)
        except queue.Full:
            pass

        if self.backend == "appsink":
            # end-of-stream first, so the decoder flushes rather than being
            # torn down mid-frame, then NULL to release the hardware decoder.
            try:
                self.src.end_of_stream()
                self.pipeline.set_state(Gst.State.NULL)
            except Exception as exc:                       # teardown, never fatal
                self.get_logger().warn(f"appsink teardown: {exc}")
            return

        if self.gst.poll() is None:
            try:
                self.gst.stdin.close()
            except (BrokenPipeError, ValueError):
                pass
            self.gst.terminate()
            try:
                self.gst.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.gst.kill()
        shutil.rmtree(self.tmpdir, ignore_errors=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BagPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
