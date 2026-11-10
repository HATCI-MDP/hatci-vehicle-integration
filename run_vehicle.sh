#!/usr/bin/env bash
# Run the self-contained HATCI perception image.
#
#   ./run_vehicle.sh selftest              prove the image works on this board
#   ./run_vehicle.sh clip1                 recorded clip, no vehicle data
#   ./run_vehicle.sh clip1 can             clip + recorded CAN  (the BENCHMARK config)
#   ./run_vehicle.sh replay /bags/<dir>    a real ROS bag, with its own CAN
#   ./run_vehicle.sh replay /bags/<dir> nocan    images only
#   ./run_vehicle.sh camera                live camera, no vehicle data
#   ./run_vehicle.sh camera can            live camera + live CAN bus
#   ./run_vehicle.sh shell                 a prompt inside the image
#
# Any ros2 launch argument is appended and therefore wins:
#   ./run_vehicle.sh clip1 can planner_mode:=baseline
#
# Unlike the old run_pipeline.sh this mounts NO source trees. The image carries
# the workspace prebuilt, the configs, the engines and the demo clips. The only
# things still mounted are the X socket, so the dashboard can draw, and /dev in
# camera mode.
set -eo pipefail

IMAGE="${IMAGE:-hatci-vehicle:draft7}"
MODE="${1:-clip1}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "run_vehicle: image $IMAGE not found. Load it with:" >&2
  echo "  gunzip -c hatci-vehicle-draft7.tar.gz | docker load" >&2
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
# Settled: the demo runs on our own Neousys, so no external container policy
# applies here. Confirmed working on this board. See section 2 of OPEN-ITEMS.md,
# which also notes that this reopens if deployment moves to HATCI's AGX Orin.
# ---------------------------------------------------------------------------
CAMERA=()
if [ "$MODE" = "camera" ]; then
  CAMERA=(-v /dev:/dev --privileged)
  echo "run_vehicle: camera mode, relaxing device rules for the Tegra capture path" >&2

  # NRU-230V: one IMX490 on Port#1. Override with e.g. NRU_PORT_MAP="1=imx490 5=imx490".
  NRU_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/NRU_Scripts/NRU-230V"
  NRU_PORT_MAP="${NRU_PORT_MAP:-1=imx490}"

  # /sys check, not lsmod|grep: grep -q closing the pipe early trips pipefail.
  if [ -d /sys/module/nv_nru_mix230 ] && compgen -G "/dev/nru-port*" >/dev/null; then
    echo "run_vehicle: nv_nru_mix230 loaded, skipping camera init" >&2
  else
    for f in init_8xIMXCam.sh nru-cam-port nv_nru_mix230.ko; do
      [ -f "$NRU_DIR/$f" ] || { echo "run_vehicle: missing $NRU_DIR/$f" >&2; exit 1; }
    done
    chmod +x "$NRU_DIR/init_8xIMXCam.sh" "$NRU_DIR/nru-cam-port"
    echo "run_vehicle: initializing NRU-230V cameras ($NRU_PORT_MAP)" >&2
    (cd "$NRU_DIR" && sudo ./init_8xIMXCam.sh) \
      || { echo "run_vehicle: init_8xIMXCam.sh failed" >&2; exit 1; }
    # shellcheck disable=SC2086
    sudo "$NRU_DIR/nru-cam-port" set $NRU_PORT_MAP \
      || { echo "run_vehicle: nru-cam-port set failed" >&2; exit 1; }
  fi

  # Lets the camera node see which ports have video lock (written by nru-cam-port).
  if [ -f /run/nru-cam-port.live ]; then
    CAMERA+=(-v /run/nru-cam-port.live:/run/nru-cam-port.live:ro)
  else
    echo "run_vehicle: WARNING: /run/nru-cam-port.live missing, camera node cannot check signal" >&2
  fi
fi

# ---------------------------------------------------------------------------
# replay needs the bag visible inside the container, and visible at the SAME
# path, so the path typed on the command line is the path rosbag2 opens. Bind
# the bag's parent directory read-only: a multi-file bag is a directory, and
# mounting the parent also lets the entrypoint resolve a wrapper directory one
# level down without a second mount.
# ---------------------------------------------------------------------------
BAGMOUNT=()
if [ "$MODE" = "replay" ]; then
  BAG="${2:-}"
  if [ -z "$BAG" ]; then
    echo "run_vehicle: replay needs a bag path:" >&2
    echo "  ./run_vehicle.sh replay /home/nvidia/bags/bag_20261006_125333" >&2
    exit 1
  fi
  if [ ! -d "$BAG" ]; then
    echo "run_vehicle: '$BAG' is not a directory. A ROS 2 bag is a directory" >&2
    echo "             holding its data files and metadata.yaml." >&2
    exit 1
  fi
  BAG=$(cd "$BAG" && pwd)                       # absolute, so the mount matches
  PARENT=$(dirname "$BAG")
  BAGMOUNT=(-v "$PARENT:$PARENT:ro")
  set -- replay "$BAG" "${@:3}"
  echo "run_vehicle: mounting $PARENT read-only for replay" >&2
fi

# --network host    is how the container sees can0 at all: a bus that is up on
#                   the host needs no --device flag.
# --ipc=host        shares the host's IPC namespace, intended to let DDS use
#                   shared memory. Every reported number used it; its effect
#                   has not been measured on its own.
# --runtime nvidia  mounts the host's driver, including libcuda, into the
#                   container. The CUDA runtime and TensorRT are in the image.
#                   This is the host dependency dockerizing does not remove.
exec docker run --rm "${TTY[@]}" \
  --runtime nvidia --network host --ipc=host \
  "${XSOCK[@]}" "${CAMERA[@]}" ${BAGMOUNT[@]+"${BAGMOUNT[@]}"} \
  "$IMAGE" "$@"
