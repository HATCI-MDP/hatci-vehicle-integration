# shellcheck shell=bash
# Host-side NRU-230V camera bring-up. Sourced by run_vehicle.sh and dev.sh;
# not meant to be run on its own.
#
#   nru_camera_setup     loads the driver, sets CAMERA_PORT to CAMERA_MODEL,
#                        checks video lock. Sets PORT, CAM_W, CAM_H, VIDEO_IDX.
#                        Returns non-zero (with a message) on failure.
#
# Inputs (environment):
#   CAMERA_MODEL=imx490|isx031|imx390|isx021    (default imx490)
#   CAMERA_PORT=0..7                            (default 0)
#   NRU_IGNORE_SIGNAL=1                         continue without video lock
#
# The camera lives on the driver's Port#N, which is NOT the same as /dev/videoN.
# /dev/nru-portN is always Port#N; the /dev/videoN with the same device number
# is what the camera node opens.

NRU_LIVE=/run/nru-cam-port.live

# camera model -> capture size (the only mode the driver offers for each)
nru_camera_size () {
  case "$1" in
    imx490) echo "2880 1860" ;;
    isx031) echo "1920 1536" ;;
    imx390) echo "1920 1080" ;;
    isx021) echo "1920 1280" ;;
    *) return 1 ;;
  esac
}

nru_camera_setup () {
  local tag="${NRU_TAG:-camera}"
  local dir; dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/NRU_Scripts/NRU-230V"
  local tool="$dir/nru-cam-port" f have rdev v

  CAMERA_MODEL="${CAMERA_MODEL:-imx490}"
  PORT="${CAMERA_PORT:-0}"
  read -r CAM_W CAM_H < <(nru_camera_size "$CAMERA_MODEL") \
    || { echo "$tag: unknown CAMERA_MODEL '$CAMERA_MODEL'" >&2; return 1; }
  [[ "$PORT" =~ ^[0-7]$ ]] || { echo "$tag: CAMERA_PORT must be 0-7" >&2; return 1; }
  for f in init_8xIMXCam.sh nru-cam-port nv_nru_mix230.ko; do
    [ -f "$dir/$f" ] || { echo "$tag: missing $dir/$f" >&2; return 1; }
  done
  chmod +x "$dir/init_8xIMXCam.sh" "$tool"

  # Driver up? /sys check, not lsmod|grep: grep -q closing the pipe early trips pipefail.
  if [ -d /sys/module/nv_nru_mix230 ] && compgen -G "/dev/nru-port*" >/dev/null; then
    echo "$tag: nv_nru_mix230 loaded, skipping camera init" >&2
  else
    echo "$tag: initializing NRU-230V cameras" >&2
    (cd "$dir" && sudo ./init_8xIMXCam.sh) \
      || { echo "$tag: init_8xIMXCam.sh failed" >&2; return 1; }
    sudo "$tool" load || { echo "$tag: nru-cam-port load failed" >&2; return 1; }
  fi

  # The driver's built-in map says Port#0 is an isx031, so the first run remaps
  # it (reloads the driver; the one sudo prompt). Saved, so later runs skip it.
  have="$(cut -d, -f$((PORT+1)) /sys/module/nv_nru_mix230/parameters/port_size 2>/dev/null || true)"
  if [ "$have" != "${CAM_W}x${CAM_H}" ]; then
    echo "$tag: Port#$PORT is set to '${have:-?}', setting it to $CAMERA_MODEL" >&2
    sudo "$tool" set "$PORT=$CAMERA_MODEL" \
      || { echo "$tag: nru-cam-port set failed (close anything using the camera)" >&2; return 1; }
  fi

  if ! grep -qw "$PORT" "$NRU_LIVE" 2>/dev/null && [ "${NRU_IGNORE_SIGNAL:-0}" != 1 ]; then
    echo "$tag: Port#$PORT has no video lock (locked: $(cat "$NRU_LIVE" 2>/dev/null || echo none))." >&2
    echo "  Check power and cable. This hardware does not hot-plug: attach the" >&2
    echo "  camera, then reboot the board. NRU_IGNORE_SIGNAL=1 tries anyway." >&2
    return 1
  fi

  rdev="$(stat -c '%t:%T' "/dev/nru-port$PORT")"
  VIDEO_IDX=""
  for v in /dev/video[0-9]*; do
    [ "$(stat -c '%t:%T' "$v")" = "$rdev" ] && { VIDEO_IDX="${v#/dev/video}"; break; }
  done
  [ -n "$VIDEO_IDX" ] || { echo "$tag: no /dev/video* matches /dev/nru-port$PORT" >&2; return 1; }
  echo "$tag: $CAMERA_MODEL on Port#$PORT = /dev/video$VIDEO_IDX (${CAM_W}x${CAM_H})" >&2
}
