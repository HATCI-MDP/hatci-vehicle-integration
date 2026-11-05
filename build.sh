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
TAG="${TAG:-hatci-vehicle:draft7}"

for d in "$REPO/src/perception_suite" "$REPO/src/perception_suite_msgs" \
         "$REPO/src/offroad_autonomy" "$BENCH/models"; do
  [ -d "$d" ] || { echo "build: missing $d" >&2; exit 1; }
done

# A tag names one verified artifact. Rebuilding over it would replace a verified artifact with an unverified one of the
# same name, and nothing downstream could tell the difference, so refuse unless
# asked twice.
if docker image inspect "$TAG" >/dev/null 2>&1 && [ -z "$FORCE" ]; then
  echo "build: $TAG already exists." >&2
  echo "build:   TAG=hatci-vehicle:draft8 ./build.sh   build under a new name" >&2
  echo "build:   FORCE=1 ./build.sh                    overwrite it deliberately" >&2
  echo "build:   SLIM=1 ./build.sh                     26s weights and clip1 only" >&2
  exit 1
fi

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

# The three packages, from the repo. Paths match the Dockerfile's COPY lines.
# perception_suite_msgs is not optional: perception_suite imports it at module
# scope, and colcon still reports a clean build without it because an
# ament_python package is never checked for its imports. Leaving it out of the
# context builds an image that only fails at launch.
cp -a "$REPO/src/perception_suite"      "$CTX/src/perception_suite"
cp -a "$REPO/src/perception_suite_msgs" "$CTX/src/perception_suite_msgs"
cp -a "$REPO/src/offroad_autonomy"      "$CTX/src/offroad_autonomy"

# The replay-only ads_messages reconstruction. Staged separately and built into
# its own overlay, never into the pipeline workspace: ROS 2 Humble matches
# publisher to subscriber on type NAME alone, so ours and HATCI's real package
# are indistinguishable at runtime and must never be live together.
cp -a "$REPO/replay"                "$CTX/replay"

# Big trees: real copies. Hardlinks would be free, but most of these files are
# root-owned and /proc/sys/fs/protected_hardlinks forbids linking a file you do
# not own. ~1.1 GB on nvme, a few seconds.
cp -a "$BENCH/data/can" "$CTX/bench/data/can"
mkdir -p "$CTX/bench/data/clips"

# SLIM=1 stages only what the shipping configuration actually loads: the 26s
# weights and one clip. It is OFF by default, and the reason is worth stating
# because the obvious assumption is wrong.
#
# Measured on draft7: 34.1 GB on disk and a 12.5 GB tarball normally, 32.3 GB and
# 11.7 GB with SLIM=1. The base hatci-pipeline:v1 is 31.5 GB of that, mostly
# /usr/lib/aarch64-linux-gnu (6.5 GB) and dist-packages (2.2 GB), which cannot be
# trimmed without changing the base, and that would invalidate every number we
# have reported.
#
# So this buys about 7 percent off the tarball at the cost of entrypoint modes
# clip2 and clip3 and the ability to reproduce the model comparison. Worth it for
# a transfer, not worth it as the default.
if [ -n "$SLIM" ]; then
  echo "build: SLIM=1, staging the 26s weights and clip1 only" >&2
  mkdir -p "$CTX/bench/models/new"
  cp -a "$BENCH"/models/new/weights_26s.* "$CTX/bench/models/new/"
  cp "$BENCH/data/clips/hatci_bag1_lowspeed.mp4" "$CTX/bench/data/clips/"
  echo "build: modes clip2 and clip3 will NOT work in this image" >&2
else
  cp -a "$BENCH/models"   "$CTX/bench/models"
  cp "$BENCH"/data/clips/hatci_bag*.mp4 "$CTX/bench/data/clips/"
fi

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
