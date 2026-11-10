#!/usr/bin/env bash
# Entrypoint for the self-contained HATCI perception image.
#
# FIVE ways to run the pipeline. Four are demo or deployment paths; the fifth
# exists to benchmark, and is called out because it is easy to mistake for a
# demo path and quote the wrong numbers from.
#
#   1. clip1|clip2|clip3          recorded mp4, no vehicle data.
#                                 Baseline centreline planner.
#   2. replay <bagpath>           a real ROS bag, WITH its CAN. Images and
#                                 odometry come off one recording on one clock,
#                                 so they are aligned by timestamp. The BEV
#                                 grid planner. Add "nocan" to replay images
#                                 only, which is how you tell a perception
#                                 fault from a planning one.
#   3. camera                     live camera, no vehicle data. The camera
#                                 bring-up path: proves capture, the GMSL link
#                                 and the six-node graph with no vehicle.
#   4. camera can                 live camera AND live CAN. The vehicle path.
#   5. clip1 can                  recorded mp4 + recorded CAN csv. BENCHMARK
#                                 ONLY. This is the one configuration that runs
#                                 the BEV grid planner at a full 30 FPS on this
#                                 board, which is why every reported latency
#                                 and frame-rate number comes from it. Mode 2
#                                 cannot replace it: bag replay now reaches
#                                 about 30 FPS, but its rate is set by the
#                                 replayer and the H.264 decoder rather than by
#                                 the pipeline, so its number says less about
#                                 what the stages can actually sustain.
#
# Also:
#   selftest                      mode 5, headless, with a pass/fail exit code
#   shell                         a prompt inside the image
#
# Any ros2 launch argument is passed straight through and wins, e.g.
#   ... clip1 can planner_mode:=baseline
#
# The two that are off by default and are not modes, because they apply to any
# mode above:
#
#   control:=true                 run 7_controller_node, which turns the plan
#                                 into steering, throttle and brake on
#                                 /command. OFF by default: the camera
#                                 integration test does not need control, and
#                                 nothing on the vehicle subscribes to /command
#                                 yet. Add speed_mps:=<m/s> when there is no
#                                 CAN source, since the controller otherwise
#                                 has no speed to plan against.
#   visualize:=true               run 6_visualizer_node, the dashboard, on the
#                                 host's X display. Needs DISPLAY and the X
#                                 socket mounted; run_vehicle.sh does both.
#                                 With control:=true the steering, throttle and
#                                 brake readouts follow the controller, and
#                                 they fall back to zero after
#                                 command_timeout_s with nothing publishing.
#
# No "set -u": the ROS setup scripts reference unbound variables.
set -eo pipefail

source /opt/ros/humble/setup.bash

# WS picks the colcon workspace to run. The image's own prebuilt one is the
# default; dev.sh points it at a mounted checkout built with
# `colcon build --symlink-install`, so source and config edits apply on the
# next launch without rebuilding the image.
WS="${WS:-/opt/ws}"
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "entrypoint: $WS/install/setup.bash not found. Build that workspace first:" >&2
  echo "entrypoint:   cd $WS && colcon build --symlink-install" >&2
  exit 1
fi
source "$WS/install/setup.bash"

# A workspace that carries its own configs/ uses them, so a config edit there
# is what runs. CONFIG=... still overrides.
if [ -z "${CONFIG:-}" ] && [ -f "$WS/configs/tmp_26s_demo.yaml" ]; then
  CONFIG="$WS/configs/tmp_26s_demo.yaml"
fi
CONFIG="${CONFIG:-/bench/configs/tmp_26s_demo.yaml}"
CLIPS=/bench/data/clips
CANDIR=/bench/data/can

IMAGE_TOPIC="${IMAGE_TOPIC:-/image_processing/image_compressed}"
INFO_TOPIC="${IMAGE_TOPIC%/*}/camera_info"

# Each clip's CAN, extracted by scripts/extract_can.py. odom_node pairs frame N
# with row N, so the clip and the csv must come from the same recording. This
# index pairing is exactly what mode 2 does not have to do.
declare -A CAN_FOR=(
  [clip1]=bag_20261006_125333.csv
  [clip2]=bag_20261006_125811.csv
  [clip3]=bag_20261006_130145.csv
)
declare -A CLIP_FOR=(
  [clip1]=hatci_bag1_lowspeed.mp4
  [clip2]=hatci_bag2_highspeed.mp4
  [clip3]=hatci_bag3_laneedges.mp4
)

MODE="${1:-clip1}"; shift || true

# bag1|bag2|bag3 meant these mp4 clips, which was always confusing and is now
# actively wrong next to a mode that really does play a bag. Aliased rather
# than removed, so anything already written down keeps working.
case "$MODE" in
  bag1|bag2|bag3)
    ALIAS="clip${MODE#bag}"
    echo "entrypoint: '$MODE' is now '$ALIAS'. bag1-3 were never ROS bags, they" >&2
    echo "entrypoint:   are mp4 clips extracted from them. Use '$ALIAS', or" >&2
    echo "entrypoint:   'replay <bagpath>' for an actual bag. Alias still works." >&2
    MODE="$ALIAS" ;;
esac

BAG_PATH=""
case "$MODE" in
  shell) exec bash ;;
  clip1|clip2|clip3|camera|selftest) ;;
  replay)
    BAG_PATH="${1:-}"; shift || true
    if [ -z "$BAG_PATH" ]; then
      echo "entrypoint: replay needs a bag path, e.g." >&2
      echo "entrypoint:   replay /bags/bag_20261006_125333" >&2
      exit 1
    fi
    # A recording is often a wrapper directory holding the actual bag one level
    # down, next to a metadata.json of scene labels. Pointing rosbag2 at the
    # wrapper fails with "no plugin found that could open URI", which reads as
    # a missing storage plugin rather than a wrong path. Resolve it here.
    if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
      NESTED=$(find "$BAG_PATH" -mindepth 2 -maxdepth 2 -name metadata.yaml \
               2>/dev/null | head -1)
      if [ -n "$NESTED" ]; then
        BAG_PATH=$(dirname "$NESTED")
        echo "entrypoint: no metadata.yaml at the top level; using $BAG_PATH" >&2
      else
        echo "entrypoint: no metadata.yaml anywhere under '$BAG_PATH'." >&2
        echo "entrypoint:   That path is not a ROS bag." >&2
        exit 1
      fi
    fi ;;
  *) echo "entrypoint: unknown mode '$MODE'." >&2
     echo "entrypoint: Use clip1, clip2, clip3, replay <bagpath>, camera," >&2
     echo "entrypoint:   selftest or shell." >&2
     exit 1 ;;
esac

SELFTEST=0
if [ "$MODE" = "selftest" ]; then
  SELFTEST=1
  MODE=clip1
  # The selftest exercises the configuration the report describes, which is 26s
  # under the BEV grid, so it turns odometry on. Without this it would quietly
  # test the baseline centreline planner instead and still pass, which is the
  # wrong thing to prove on a new board.
  case " $* " in
    *" can "*|*" planner_mode:="*) ;;
    *) set -- can "$@" ;;
  esac
fi

# ---------------------------------------------------------------------------
# Vehicle data on or off.
#
# Replay defaults to ON, because a bag carries its own CAN and there is no
# reason to throw it away. Everything else defaults to OFF, because a clip
# needs a separately extracted csv and a live camera needs a real bus.
# ---------------------------------------------------------------------------
if [ "$MODE" = "replay" ]; then WANT_CAN=1; else WANT_CAN=0; fi
ARGS=()
for arg in "$@"; do
  case "$arg" in
    can)   WANT_CAN=1 ;;
    nocan) WANT_CAN=0 ;;
    *)     ARGS+=("$arg") ;;
  esac
done
set -- ${ARGS[@]+"${ARGS[@]}"}

# ---------------------------------------------------------------------------
# ads_messages: HATCI's message package, the types their CAN node publishes on
# /CAN.
# ---------------------------------------------------------------------------
ensure_ads_messages () {
  ros2 pkg prefix ads_messages >/dev/null 2>&1
}

# ---------------------------------------------------------------------------
# Source
# ---------------------------------------------------------------------------
SOURCE_ARGS=()
CAN_ARGS=()

case "$MODE" in
  camera)
    # camera_index 0 is the GMSL capture on our bench carrier; the other three
    # channels exist in the device tree but have nothing attached. HATCI's will
    # differ; append camera_index:=N. run_vehicle.sh does not forward the
    # CAMERA_INDEX or ROTATE_180 environment variables into the container.
    #
    # rotate_180 is on because our camera presents inverted. It is NOT a
    # property of the pipeline, it is a property of how this camera is mounted,
    # so check the dashboard before trusting it on different hardware: the road
    # must be at the BOTTOM of the frame or the ground projection is wrong.
    # Append rotate_180:=false for a camera mounted the right way up.
    #
    # NV16 is not a preference. The device advertises UYVY first and OpenCV's
    # own V4L2 backend mis-decodes every format it offers; only NV16 pulled
    # through a GStreamer pipeline produces a correct frame.
    #
    # NRU-230V: with CAMERA_PORT set (dev.sh sets it), the /dev/videoN is
    # looked up from /dev/nru-portN at launch, because the N in videoN can
    # change when the driver is reloaded. CAMERA_MODEL sets the capture size.
    if [ -z "${CAMERA_INDEX:-}" ] && [ -n "${CAMERA_PORT:-}" ] && [ -e "/dev/nru-port$CAMERA_PORT" ]; then
      RDEV="$(stat -c '%t:%T' "/dev/nru-port$CAMERA_PORT")"
      for v in /dev/video[0-9]*; do
        [ "$(stat -c '%t:%T' "$v")" = "$RDEV" ] && { CAMERA_INDEX="${v#/dev/video}"; break; }
      done
      echo "entrypoint: Port#$CAMERA_PORT = /dev/video${CAMERA_INDEX:-?}" >&2
    fi
    SIZE_ARGS=()
    case "${CAMERA_MODEL:-}" in
      imx490) SIZE_ARGS=(capture_width:=2880 capture_height:=1860) ;;
      isx031) SIZE_ARGS=(capture_width:=1920 capture_height:=1536) ;;
      imx390) SIZE_ARGS=(capture_width:=1920 capture_height:=1080) ;;
      isx021) SIZE_ARGS=(capture_width:=1920 capture_height:=1280) ;;
    esac
    SOURCE_ARGS=(source:=camera "camera_index:=${CAMERA_INDEX:-0}"
                 "rotate_180:=${ROTATE_180:-true}" pixel_format:=NV16
                 camera_fps:=30.0 width:=640 ${SIZE_ARGS[@]+"${SIZE_ARGS[@]}"})
    if [ "$WANT_CAN" = 1 ]; then
      if ! ensure_ads_messages; then
        echo "entrypoint: live CAN needs ads_messages and $WS was built without" >&2
        echo "entrypoint:   it. Rebuild the workspace with src/ads_messages in it." >&2
        echo "entrypoint:   Continuing WITHOUT odometry." >&2
        WANT_CAN=0
      else
        CAN_ARGS=("can_topic:=${CAN_TOPIC:-/CAN}")
        echo "entrypoint: live odometry from ${CAN_TOPIC:-/CAN}, BEV grid planner" >&2
      fi
    fi ;;

  replay)
    SOURCE_ARGS=(source:=bag "bag_path:=$BAG_PATH" width:=640)
    if [ "$WANT_CAN" = 1 ]; then
      if ! ensure_ads_messages; then
        echo "entrypoint: cannot replay /CAN: $WS was built without ads_messages." >&2
        echo "entrypoint:   Rebuild the workspace with src/ads_messages in it." >&2
        echo "entrypoint:   Replaying images only; baseline planner." >&2
        WANT_CAN=0
      fi
    fi
    if [ "$WANT_CAN" = 1 ]; then

      SOURCE_ARGS+=("bag_topics:=$IMAGE_TOPIC $INFO_TOPIC /CAN")
      CAN_ARGS=("can_topic:=${CAN_TOPIC:-/CAN}")
      echo "entrypoint: replaying $BAG_PATH with its own CAN; BEV grid planner" >&2
    else
      SOURCE_ARGS+=("bag_topics:=$IMAGE_TOPIC $INFO_TOPIC")
      echo "entrypoint: replaying $BAG_PATH, images only; baseline planner" >&2
    fi ;;

  *)
    SOURCE_ARGS=(source:=video "video_path:=$CLIPS/${CLIP_FOR[$MODE]}"
                 loop:=true video_fps:=30.0 width:=640)
    if [ "$WANT_CAN" = 1 ]; then
      CSV="$CANDIR/${CAN_FOR[$MODE]}"
      if [ ! -f "$CSV" ]; then
        echo "entrypoint: $CSV is missing from the image." >&2
        exit 1
      fi
      CAN_ARGS=("can_csv:=$CSV")
      echo "entrypoint: CAN odometry from $(basename "$CSV"), BEV grid planner" >&2
    fi ;;
esac

# odom_node pairs frame N with CAN row N, so a clip and a csv of different
# lengths silently feed the planner motion from a different part of the drive.
# The presets always match; an overriding video_path:= need not, and the
# failure is invisible rather than loud. Mode 2 does not need this check at
# all, because a bag aligns by timestamp instead of by index.
if [ ${#CAN_ARGS[@]} -gt 0 ] && [ "$MODE" != "camera" ] && [ "$MODE" != "replay" ]; then
  CLIP_PATH="$CLIPS/${CLIP_FOR[$MODE]}"
  for arg in "$@"; do
    case "$arg" in video_path:=*) CLIP_PATH="${arg#video_path:=}" ;; esac
  done
  python3 - "$CLIP_PATH" "$CANDIR/${CAN_FOR[$MODE]}" <<'PRECHECK' || exit 1
import sys, cv2
clip, csv_path = sys.argv[1], sys.argv[2]
frames = int(cv2.VideoCapture(clip).get(cv2.CAP_PROP_FRAME_COUNT))
with open(csv_path) as fh:
    rows = sum(1 for _ in fh) - 1
drift = abs(frames - rows) / max(frames, 1)
print(f'entrypoint: {frames} frames, {rows} CAN rows ({drift:.1%} apart)')
if drift > 0.05:
    sys.exit(f'entrypoint: the clip and the CAN csv are {drift:.0%} apart. '
             f'odom_node pairs them by index, so the planner would be given ego '
             f'motion from a different part of the drive.')
PRECHECK
fi

# ---------------------------------------------------------------------------
# Dashboard. On by default when there is an X server to draw on, because every
# reported number was measured with it running as load; off automatically when
# there is not, instead of failing with "No display (DISPLAY is unset)".
# ---------------------------------------------------------------------------
VIS_ARGS=()
case " $* " in
  *" visualize:="*) ;;
  *) if [ -n "$DISPLAY" ] && [ -d /tmp/.X11-unix ]; then
       VIS_ARGS=(visualize:=true)
     else
       VIS_ARGS=(visualize:=false)
       echo "entrypoint: no DISPLAY, running headless (visualize:=false)" >&2
     fi ;;
esac

echo "entrypoint: mode=$MODE | config $CONFIG" >&2

# ---------------------------------------------------------------------------
# selftest: bring the pipeline up headless, measure the two topics that matter,
# and exit non-zero if either is dead. This is what HATCI should run first on
# their own board - it proves the image, the engine and the CUDA stack all
# work there before anything is wired to a vehicle.
# ---------------------------------------------------------------------------
if [ "$SELFTEST" = "1" ]; then
  echo "=== selftest: 26s + BEV grid on $MODE, headless ===" >&2
  LOG=$(mktemp)
  ros2 launch perception_suite pipeline.launch.py "config:=$CONFIG" \
    "${SOURCE_ARGS[@]}" ${CAN_ARGS[@]+"${CAN_ARGS[@]}"} visualize:=false "$@" \
    >"$LOG" 2>&1 &
  LAUNCH_PID=$!
  trap 'kill -INT $LAUNCH_PID 2>/dev/null || true' EXIT

  # The engine deserialises and the first inference warms up; 45 s is the
  # settle time the bench harness used before it started counting.
  for i in $(seq 45); do
    sleep 1
    kill -0 $LAUNCH_PID 2>/dev/null || { echo "FAIL: pipeline exited early" >&2
                                         tail -40 "$LOG" >&2; exit 1; }
  done

  FAIL=0
  for topic in /road_mask /path; do
    echo "--- $topic ---" >&2
    HZ=$(timeout 20 ros2 topic hz "$topic" 2>&1 | grep -m1 "average rate" || true)
    if [ -z "$HZ" ]; then
      echo "FAIL: $topic published nothing in 20 s" >&2
      FAIL=1
    else
      echo "ok: $topic $HZ" >&2
    fi
  done

  # ros2 launch does not reliably exit on SIGINT once all six nodes are up,
  # and a selftest that never returns is worse than one that is blunt about
  # shutting down. Ask nicely, wait, then insist.
  kill -INT $LAUNCH_PID 2>/dev/null || true
  for _ in $(seq 10); do
    kill -0 $LAUNCH_PID 2>/dev/null || break
    sleep 1
  done
  kill -KILL $LAUNCH_PID 2>/dev/null || true
  wait $LAUNCH_PID 2>/dev/null || true
  trap - EXIT
  if [ "$FAIL" = "0" ]; then
    echo "=== selftest PASSED ===" >&2
  else
    echo "=== selftest FAILED; last 40 log lines ===" >&2
    tail -40 "$LOG" >&2
  fi
  exit "$FAIL"
fi

exec ros2 launch perception_suite pipeline.launch.py "config:=$CONFIG" \
  "${SOURCE_ARGS[@]}" ${CAN_ARGS[@]+"${CAN_ARGS[@]}"} "${VIS_ARGS[@]}" "$@"
