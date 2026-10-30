#!/usr/bin/env bash
# Stage a build context and build the self-contained image, on the board.
#
# The context is assembled rather than pointed at a repo because the four trees
# the old harness bind-mounted are scattered across the filesystem and two of
# them carry gigabytes nothing reads (stray engines, test videos, a 740 MB
# .git). Big files are HARDLINKED in, so staging ~1.1 GB costs no disk and
# takes under a second.
#
# Nothing here writes to any source tree. ros2-perception-suite is read for
# nothing at all: the package source comes from the mono8b export.
set -eo pipefail

CTX=/home/nvidia/handoff/ctx
BENCH=/home/nvidia/bench-ros
FELIX=/home/nvidia/ab/mono8b
BRYAN=/home/nvidia/MDP/humble_ws/orfd-lane-detection
TAG="${TAG:-hatci-vehicle:draft1}"

echo "=== staging context in $CTX ==="
rm -rf "$CTX"
mkdir -p "$CTX"/bench/data "$CTX"/src/perception_suite "$CTX"/src/offroad_autonomy "$CTX"/docker

# Small trees: real copies, because the build adds files to bench/overlays and
# a hardlink would write through into bench-ros.
cp -a "$BENCH/configs"  "$CTX/bench/configs"
cp -a "$BENCH/scripts"  "$CTX/bench/scripts"
cp -a "$BENCH/overlays" "$CTX/bench/overlays"
cp -a "$FELIX/src/perception_suite" "$CTX/src/perception_suite"

# Big trees: real copies. Hardlinks would be free, but most of these files are
# root-owned and /proc/sys/fs/protected_hardlinks forbids linking a file you do
# not own. ~1.1 GB on nvme, a few seconds.
cp -a "$BENCH/models"   "$CTX/bench/models"
cp -a "$BENCH/data/can" "$CTX/bench/data/can"
mkdir -p "$CTX/bench/data/clips"
cp "$BENCH"/data/clips/hatci_bag*.mp4 "$CTX/bench/data/clips/"

# The offroad_autonomy repo without its 740 MB of history.
(cd "$BRYAN" && tar cf - --exclude=.git --exclude=.github --exclude=.coverage .) \
  | (cd "$CTX/src/offroad_autonomy" && tar xf -)

# The files this handoff adds.
cp "$(dirname "$0")/overlays/live_odom_node.py"    "$CTX/bench/overlays/"
cp "$(dirname "$0")/overlays/install_live_odom.py" "$CTX/bench/overlays/"
cp "$(dirname "$0")/docker/entrypoint.sh"          "$CTX/docker/"
cp "$(dirname "$0")/docker/Dockerfile"             "$CTX/Dockerfile"

echo "context size: $(du -sh "$CTX" | cut -f1)"
du -sh "$CTX"/*/ | sort -rh

echo
echo "=== building $TAG ==="
cd "$CTX"
docker build -t "$TAG" -f Dockerfile . 2>&1 | tail -60
echo
docker images --format "{{.Repository}}:{{.Tag}}  {{.Size}}" | grep -E "hatci-vehicle|hatci-pipeline"
