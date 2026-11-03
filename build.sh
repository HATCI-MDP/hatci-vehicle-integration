#!/usr/bin/env bash
# Stage a build context and build the self-contained image, on the board.
#
# The source now comes from the published repo rather than from working trees
# scattered around this filesystem. That is the point: the image is built from
# exactly the source HATCI can read, so "the code is in src/" is literally
# true rather than approximately true. REPO is a plain clone of
# github.com/HATCI-MDP/hatci-vehicle-integration and nothing here writes to it.
#
# What is NOT in the repo, and so still comes off this board: the model
# weights and TensorRT engines, the demo clips, the extracted CAN csvs, and
# bench-ros/scripts. Those are either HATCI's own vehicle data or too large to
# publish, which is why this script exists instead of a plain docker build.
set -eo pipefail

REPO="${REPO:-/home/nvidia/hatci-vehicle-integration}"
BENCH="${BENCH:-/home/nvidia/bench-ros}"
CTX="${CTX:-/home/nvidia/handoff/ctx}"
TAG="${TAG:-hatci-vehicle:draft2}"

for d in "$REPO/src/perception_suite" "$REPO/src/offroad_autonomy" "$BENCH/models"; do
  [ -d "$d" ] || { echo "build: missing $d" >&2; exit 1; }
done

echo "=== source provenance ==="
git -C "$REPO" --no-pager log -1 --format='repo   %h %s' || true
if [ -n "$(git -C "$REPO" status --porcelain 2>/dev/null)" ]; then
  echo "build: WARNING $REPO has uncommitted changes; the image will not match" >&2
  echo "build:         any commit. Commit or stash before a release build." >&2
fi

echo "=== staging context in $CTX ==="
rm -rf "$CTX"
mkdir -p "$CTX"/bench/data "$CTX"/src "$CTX"/docker

# Small trees: real copies, because the build adds files to bench/overlays and
# a hardlink would write through into bench-ros.
cp -a "$BENCH/configs"  "$CTX/bench/configs"
cp -a "$BENCH/scripts"  "$CTX/bench/scripts"
cp -a "$BENCH/overlays" "$CTX/bench/overlays"

# The two packages, from the repo. Paths match the Dockerfile's COPY lines.
cp -a "$REPO/src/perception_suite"  "$CTX/src/perception_suite"
cp -a "$REPO/src/offroad_autonomy"  "$CTX/src/offroad_autonomy"

# Big trees: real copies. Hardlinks would be free, but most of these files are
# root-owned and /proc/sys/fs/protected_hardlinks forbids linking a file you do
# not own. ~1.1 GB on nvme, a few seconds.
cp -a "$BENCH/models"   "$CTX/bench/models"
cp -a "$BENCH/data/can" "$CTX/bench/data/can"
mkdir -p "$CTX/bench/data/clips"
cp "$BENCH"/data/clips/hatci_bag*.mp4 "$CTX/bench/data/clips/"

# The files this handoff adds, also from the repo.
cp "$REPO/overlays/live_odom_node.py"    "$CTX/bench/overlays/"
cp "$REPO/overlays/install_live_odom.py" "$CTX/bench/overlays/"
cp "$REPO/docker/entrypoint.sh"          "$CTX/docker/"
cp "$REPO/docker/Dockerfile"             "$CTX/Dockerfile"

# live_odom_node.py is shipped twice on purpose: install_live_odom.py copies it
# into the package at build time, and bench/overlays keeps the readable
# original. They must not drift, so fail loudly rather than ship a mismatch.
if ! diff -q "$REPO/overlays/live_odom_node.py" \
             "$REPO/src/perception_suite/perception_suite/live_odom_node.py" >/dev/null; then
  echo "build: overlays/live_odom_node.py and the copy in src/ differ." >&2
  echo "build:        One of them is stale. Reconcile before building." >&2
  exit 1
fi

echo "context size: $(du -sh "$CTX" | cut -f1)"
du -sh "$CTX"/*/ | sort -rh

echo
echo "=== building $TAG ==="
cd "$CTX"
docker build -t "$TAG" -f Dockerfile . 2>&1 | tail -60
echo
docker images --format "{{.Repository}}:{{.Tag}}  {{.Size}}" | grep -E "hatci-vehicle|hatci-pipeline"
