#!/usr/bin/env bash
# Development container: the image is only the ENVIRONMENT (ROS 2, CUDA,
# TensorRT, the offroad_autonomy library, models and clips under /bench). The
# ROS packages come from THIS folder, mounted in, built once with
# `colcon build --symlink-install`, so editing src/ or configs/ here changes
# what runs next time, with no image rebuild.
#
#   ./dev.sh                 set up the camera, start the container if needed,
#                            open a shell in it (run again for more terminals)
#   ./dev.sh run <mode> ...  same, but launch straight away, e.g.
#                              ./dev.sh run camera
#                              ./dev.sh run clip1 can planner_mode:=baseline
#   ./dev.sh build           colcon build --symlink-install, then exit
#   ./dev.sh stop            stop the container (keeps it; ./dev.sh restarts it)
#   ./dev.sh rm              delete it, e.g. after changing the settings below
#
# Inside the container (see docker/dev_bashrc):
#   hatci_build              colcon build --symlink-install in the workspace
#   hatci_run camera         launch, same modes as run_vehicle.sh
#   hatci_ws <dir>           switch to another workspace under /mnt/nova_ssd/mdp
#
# Settings, read when the container is CREATED (./dev.sh rm to change them):
#   BASE_IMAGE=hatci-vehicle:draft7    environment image
#   MOUNT_ROOT=/mnt/nova_ssd/mdp       mounted at the same path inside
#   CAMERA_MODEL=imx490 CAMERA_PORT=0  NRU-230V camera (scripts/nru_camera.sh)
#   ROTATE_180=false
# Per run:
#   NO_CAMERA=1                        skip the camera bring-up on the host
#
# run_vehicle.sh is unchanged by this: it still runs the image's own prebuilt
# copy in /opt/ws, which is the verified artifact.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="${DEV_CONTAINER:-hatci-dev}"
BASE_IMAGE="${BASE_IMAGE:-hatci-vehicle:draft8}"
MOUNT_ROOT="${MOUNT_ROOT:-/mnt/nova_ssd/mdp}"
RC="$HERE/docker/dev_bashrc"
CMD="${1:-shell}"

case "$HERE/" in
  "$MOUNT_ROOT"/*) ;;
  *) echo "dev: $HERE is not under MOUNT_ROOT=$MOUNT_ROOT" >&2; exit 1 ;;
esac

# docker inspect prints an empty line before failing on a missing container,
# so test its exit status rather than its output.
state () {
  local s
  if s="$(docker container inspect -f '{{.State.Status}}' "$NAME" 2>/dev/null)"; then
    echo "$s"
  else
    echo none
  fi
}

case "$CMD" in
  stop) docker stop "$NAME" >/dev/null && echo "dev: stopped $NAME"; exit 0 ;;
  rm)   docker rm -f "$NAME" >/dev/null && echo "dev: removed $NAME"; exit 0 ;;
  shell|run|build) ;;
  *) echo "dev: unknown command '$CMD' (shell, run, build, stop, rm)" >&2; exit 1 ;;
esac

# --- image -----------------------------------------------------------------
if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
  TAR="${IMAGE_TAR:-$HERE/hatci-vehicle-${BASE_IMAGE##*:}.tar.gz}"
  [ -f "$TAR" ] || { echo "dev: $BASE_IMAGE not in Docker and no $TAR" >&2; exit 1; }
  echo "dev: loading $TAR (one-time, a few minutes)..." >&2
  gunzip -c "$TAR" | docker load >&2
fi

# --- camera, on the host (needs sudo, kernel modules) ------------------------
# Not fatal: a shell for building or for clip/replay runs needs no camera.
if [ "$CMD" != build ] && [ "${NO_CAMERA:-0}" != 1 ]; then
  # shellcheck source=scripts/nru_camera.sh
  source "$HERE/scripts/nru_camera.sh"
  NRU_TAG=dev nru_camera_setup || echo "dev: camera not ready; camera mode will fail (NO_CAMERA=1 to skip this)" >&2
fi

# --- container -------------------------------------------------------------
if [ "$(state)" = none ]; then
  X=()
  [ -z "$DISPLAY" ] && [ -S /tmp/.X11-unix/X0 ] && DISPLAY=:0
  if [ -n "$DISPLAY" ] && [ -d /tmp/.X11-unix ]; then
    X=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
    XAUTH="${XAUTHORITY:-}"
    if [ -z "$XAUTH" ]; then
      for c in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
        [ -r "$c" ] && { XAUTH="$c"; break; }
      done
    fi
    [ -n "$XAUTH" ] && [ -r "$XAUTH" ] && X+=(-v "$XAUTH":/tmp/.Xauth:ro -e XAUTHORITY=/tmp/.Xauth)
  else
    echo "dev: no X server found; the dashboard will be off" >&2
  fi

  # Runs as YOU, not root, so build/ install/ log/ in this folder stay yours.
  # Your groups (video, render, ...) carry the GPU and camera device access.
  GROUPS_ARGS=()
  for g in $(id -G); do GROUPS_ARGS+=(--group-add "$g"); done

  echo "dev: creating container $NAME from $BASE_IMAGE" >&2
  # --privileged + /dev: the Tegra capture path needs far more device nodes than
  #   /dev/videoN, and a driver reload on the host must show up inside.
  # --network host: the default bridge does not work on this board (no iptables
  #   raw table), and it is how the container sees can0 and ROS peers.
  docker run -d --name "$NAME" --init \
    --runtime nvidia --network host --ipc=host --privileged -v /dev:/dev \
    --user "$(id -u):$(id -g)" "${GROUPS_ARGS[@]}" \
    -e HOME=/tmp/hatci-home -e YOLO_CONFIG_DIR=/tmp/Ultralytics \
    -e HATCI_REPO="$HERE" -e HATCI_WS="${HATCI_WS:-$HERE}" \
    -e CAMERA_MODEL="${CAMERA_MODEL:-imx490}" -e CAMERA_PORT="${CAMERA_PORT:-0}" \
    -e ROTATE_180="${ROTATE_180:-false}" \
    -v "$MOUNT_ROOT:$MOUNT_ROOT" \
    "${X[@]}" \
    -w "$HERE" \
    --entrypoint sleep "$BASE_IMAGE" infinity >/dev/null
elif [ "$(state)" != running ]; then
  docker start "$NAME" >/dev/null
fi
docker exec "$NAME" mkdir -p /tmp/hatci-home

TTY=(-i); [ -t 0 ] && [ -t 1 ] && TTY=(-it)
case "$CMD" in
  shell) exec docker exec "${TTY[@]}" "$NAME" bash --rcfile "$RC" ;;
  build) shift; exec docker exec "${TTY[@]}" "$NAME" bash -c 'source "$0" >/dev/null 2>&1; hatci_build "$@"' "$RC" "$@" ;;
  run)   shift; exec docker exec "${TTY[@]}" "$NAME" bash -c 'source "$0" >/dev/null; hatci_run "$@"' "$RC" "$@" ;;
esac
