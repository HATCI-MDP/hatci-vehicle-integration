#!/usr/bin/env bash
# Entrypoint for the self-contained HATCI perception image.
#
#   docker run ... hatci-vehicle:v1 bag1 can     recorded clip + recorded CAN
#   docker run ... hatci-vehicle:v1 camera can   live camera + live CAN bus
#   docker run ... hatci-vehicle:v1 selftest     headless check that it works
#   docker run ... hatci-vehicle:v1 shell        a prompt inside the image
#
# Any ros2 launch argument is passed straight through and wins, e.g.
#   ... bag1 can planner_mode:=baseline
#
# No "set -u": the ROS setup scripts reference unbound variables.
set -eo pipefail

# The workspace was colcon-built at image build time, so this is the whole of
# the setup the old harness did on every single start.
source /opt/ros/humble/setup.bash
source /opt/ws/install/setup.bash

CONFIG="${CONFIG:-/bench/configs/tmp_26s_demo.yaml}"
CLIPS=/bench/data/clips
CANDIR=/bench/data/can

# Each bag's CAN, extracted by scripts/extract_can.py. odom_node pairs frame N
# with row N, so the clip and the csv must come from the same recording.
declare -A CAN_FOR=(
  [bag1]=bag_20261006_125333.csv
  [bag2]=bag_20261006_125811.csv
  [bag3]=bag_20261006_130145.csv
)
declare -A CLIP_FOR=(
  [bag1]=hatci_bag1_lowspeed.mp4
  [bag2]=hatci_bag2_highspeed.mp4
  [bag3]=hatci_bag3_laneedges.mp4
)

MODE="${1:-bag1}"; shift || true

case "$MODE" in
  shell) exec bash ;;
  bag1|bag2|bag3|camera|selftest) ;;
  *) echo "entrypoint: unknown mode '$MODE'." >&2
     echo "Use bag1, bag2, bag3, camera, selftest or shell." >&2
     exit 1 ;;
esac

SELFTEST=0
if [ "$MODE" = "selftest" ]; then
  SELFTEST=1
  MODE=bag1
  # The selftest exercises the configuration the report describes, which is 26s
  # under the BEV grid, so it turns odometry on. Without this it would quietly
  # test the baseline centerline planner instead and still pass, which is the
  # wrong thing to prove on a new board.
  case " $* " in
    *" can "*|*" planner_mode:="*) ;;
    *) set -- can "$@" ;;
  esac
fi

# ---------------------------------------------------------------------------
# Source
# ---------------------------------------------------------------------------
SOURCE_ARGS=()
if [ "$MODE" = "camera" ]; then
  # camera_index 2 is the GMSL capture on our bench carrier. HATCI's will
  # differ; override with camera_index:=N.
  SOURCE_ARGS=(source:=camera "camera_index:=${CAMERA_INDEX:-2}"
               camera_fps:=30.0 width:=640)
else
  SOURCE_ARGS=(source:=video "video_path:=$CLIPS/${CLIP_FOR[$MODE]}"
               loop:=true video_fps:=30.0 width:=640)
fi

# ---------------------------------------------------------------------------
# Odometry. "can" anywhere in the remaining arguments turns it on, and with it
# the BEV grid planner - the grid needs ego motion to register one frame
# against the next, so the two belong to the same switch.
#
# A recorded clip gets the recorded csv; the live camera gets the live bus.
# ---------------------------------------------------------------------------
CAN_ARGS=()
ARGS=()
for arg in "$@"; do
  if [ "$arg" = "can" ]; then
    if [ "$MODE" = "camera" ]; then
      TOPIC="${CAN_TOPIC:-/CAN}"
      CAN_ARGS=("can_topic:=$TOPIC")
      echo "entrypoint: live odometry from $TOPIC, planner switches to the BEV grid" >&2
      echo "entrypoint: NOTE this needs the ads_messages package built into the" >&2
      echo "entrypoint:      image. Without it live_odom_node cannot resolve the" >&2
      echo "entrypoint:      message type, publishes nothing, and the grid forgets" >&2
      echo "entrypoint:      every frame. Add planner_mode:=baseline for now." >&2
    else
      CSV="$CANDIR/${CAN_FOR[$MODE]}"
      if [ ! -f "$CSV" ]; then
        echo "entrypoint: $CSV is missing from the image." >&2
        exit 1
      fi
      CAN_ARGS=("can_csv:=$CSV")
      echo "entrypoint: CAN odometry from $(basename "$CSV"), planner switches to the BEV grid" >&2
    fi
  else
    ARGS+=("$arg")
  fi
done
set -- "${ARGS[@]}"

# odom_node pairs frame N with CAN row N, so a clip and a csv of different
# lengths silently feed the planner motion from a different part of the drive.
# The presets always match; an overriding video_path:= need not, and the
# failure is invisible rather than loud.
if [ ${#CAN_ARGS[@]} -gt 0 ] && [ "$MODE" != "camera" ]; then
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
    "${SOURCE_ARGS[@]}" "${CAN_ARGS[@]}" visualize:=false "$@" >"$LOG" 2>&1 &
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
  "${SOURCE_ARGS[@]}" "${CAN_ARGS[@]}" "${VIS_ARGS[@]}" "$@"
