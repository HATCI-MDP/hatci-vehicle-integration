#!/usr/bin/env bash
# Run the self-contained HATCI perception image.
#
#   ./run_vehicle.sh selftest        prove the image works on this board
#   ./run_vehicle.sh bag1 can        recorded clip + recorded CAN  (the reported config)
#   ./run_vehicle.sh camera can      live camera + live CAN bus
#   ./run_vehicle.sh shell           a prompt inside the image
#
# Any ros2 launch argument is appended and therefore wins:
#   ./run_vehicle.sh bag1 can planner_mode:=baseline
#
# Unlike the old run_pipeline.sh this mounts NO source trees. The image carries
# the workspace prebuilt, the configs, the engines and the demo clips. The only
# things still mounted are the X socket, so the dashboard can draw, and /dev in
# camera mode.
set -eo pipefail

IMAGE="${IMAGE:-hatci-vehicle:draft1}"
MODE="${1:-bag1}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "run_vehicle: image $IMAGE not found. Load it with:" >&2
  echo "  gunzip -c hatci-vehicle-draft1.tar.gz | docker load" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# The dashboard draws on the board's own monitor, so it needs that X server.
# An ssh session has no DISPLAY, which is why visualize:=true alone changes
# nothing: the container inherits no display and 6_visualizer_node exits with
# "No display (DISPLAY is unset)".
#
# Access is granted by mounting the session's own X cookie, in preference to
# `xhost +local:root`, which loosens access control for every local process.
# ---------------------------------------------------------------------------
XSOCK=()
if [ -z "$DISPLAY" ] && [ -S /tmp/.X11-unix/X0 ]; then
  DISPLAY=:0
  echo "run_vehicle: no DISPLAY set, using the board's screen (:0)" >&2
fi
if [ -n "$DISPLAY" ] && [ -d /tmp/.X11-unix ]; then
  XSOCK=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
  XAUTH="${XAUTHORITY:-}"
  if [ -z "$XAUTH" ]; then
    for candidate in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
      [ -r "$candidate" ] && { XAUTH="$candidate"; break; }
    done
  fi
  if [ -n "$XAUTH" ] && [ -r "$XAUTH" ]; then
    XSOCK+=(-v "$XAUTH":/tmp/.Xauth:ro -e XAUTHORITY=/tmp/.Xauth)
  else
    echo "run_vehicle: no readable X cookie, falling back to xhost" >&2
    DISPLAY="$DISPLAY" xhost +local:root >/dev/null 2>&1 \
      || echo "run_vehicle: xhost failed; the window may not appear" >&2
  fi
else
  echo "run_vehicle: no X server found, running headless" >&2
fi

# -it only when there is a terminal. With stdin redirected, -t fails outright
# with "cannot attach stdin to a TTY-enabled container", before anything runs.
TTY=()
[ -t 0 ] && TTY=(-it)

# ---------------------------------------------------------------------------
# The Tegra capture path needs far more than /dev/videoN: capture-vi-channel*,
# capture-isp-channel*, nvhost-ctrl-vi*, nvmap and tegra_camera_ctrl. Listing
# them with --device is not enough either, because the cgroup device rules
# still block the ones not listed, and the symptom is the device opening fine
# and then every read timing out with "select() timeout". So for a camera run
# the rules are relaxed outright. A recorded run needs none of this and does
# not get it.
#
# If HATCI's policy forbids --privileged, this is the line to negotiate; see
# question 2.3 in QUESTIONS.md.
# ---------------------------------------------------------------------------
CAMERA=()
if [ "$MODE" = "camera" ]; then
  CAMERA=(-v /dev:/dev --privileged)
  echo "run_vehicle: camera mode, relaxing device rules for the Tegra capture path" >&2
fi

# --network host    is how the container sees can0 at all: a bus that is up on
#                   the host needs no --device flag.
# --ipc=host        lets the six nodes pass frames through shared memory rather
#                   than copying them. Every reported number used it.
# --runtime nvidia  mounts the host's CUDA and TensorRT userspace in. This is
#                   the one host dependency dockerizing does not remove.
exec docker run --rm "${TTY[@]}" \
  --runtime nvidia --network host --ipc=host \
  "${XSOCK[@]}" "${CAMERA[@]}" \
  "$IMAGE" "$@"
