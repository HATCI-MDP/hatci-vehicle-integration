# HATCI off-road perception stack: self-contained image.
#
# Built FROM hatci-pipeline:v1 on purpose. That image is the exact environment
# every number in the R1 report was measured in (ROS 2 Humble, CUDA, TensorRT
# 10.3.0.30, ultralytics 8.4.155). Rebuilding from nvcr.io/nvidia/l4t-jetpack
# would need network access, an NGC login, and would silently pick up newer
# package versions. Starting from the measured image keeps the environment
# provably identical and makes this build take minutes instead of hours.
#
# What this changes relative to hatci-pipeline:v1: that image is an ENVIRONMENT
# only. The code arrived at runtime through four bind mounts and the ROS
# workspace was colcon-built on every single start.
# Here all of it is baked in and built once, so the container needs no mounts
# and no host checkout.
#
# The /bench layout is deliberately unchanged. The pipeline
# configs reference weights by absolute path, e.g.
#   model_weights: "/bench/models/new/weights_26s.640.engine"
# so preserving the layout means not one config file had to be rewritten.
#
# /assets is NOT carried over. It supplied *.pt/*.engine/*.mp4 symlinks so that
# bare weight names would resolve against the workspace root; the four configs
# used for every reported run all name absolute /bench paths instead. Verified
# by grep: perception_suite references neither /assets nor mobileclip.

FROM hatci-pipeline:v1

SHELL ["/bin/bash", "-lc"]

# ---------------------------------------------------------------------------
# 0. Two additions the base image lacks. Both are pinned, both were checked
#    with `apt-get install -s` first: 7 new packages, 0 removals, 0 upgrades.
#    That matters because this image is the environment every R1 number was
#    measured in, and an apt install that drags a dependency upgrade in would
#    invalidate the lot. Do not relax these pins to bare package names.
#
#    (a) The mcap storage plugin, so the image can open HATCI's bags at all.
#        Their October bags are mcap, not sqlite3. Without this, "ros2 bag
#        play" dies with "Could not load/open plugin with storage id 'mcap'",
#        while "ros2 bag info" still works because it only reads
#        metadata.yaml. That combination is why the gap went unnoticed: the
#        bag looks readable. Pinned to the rosbag2 release already here.
#
#    (b) python3-gi and the Gst/GstApp typelibs, which let 1_bag_node decode
#        H.264 in-process through appsrc/appsink instead of shelling out to
#        gst-launch and passing frames back over a FIFO. That removes a
#        process boundary and a copy per frame. The node still works without
#        these: it falls back to the subprocess backend on its own.
#        libgstreamer itself is NOT installed here, because the base image
#        already carries a matching 1.20.3.
# ---------------------------------------------------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-mcap-vendor=0.15.17-1jammy.20260909.220900 \
      ros-humble-rosbag2-storage-mcap=0.15.17-1jammy.20260909.221142 \
      libgirepository-1.0-1=1.72.0-1 \
      gir1.2-glib-2.0=1.72.0-1 \
      python3-gi=3.42.1-0ubuntu1 \
      gir1.2-gstreamer-1.0=1.20.3-0ubuntu1.1 \
      gir1.2-gst-plugins-base-1.0=1.20.1-1ubuntu0.7 \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 1. The bench tree: configs, models, overlays, helper scripts, demo clips.
# ---------------------------------------------------------------------------
COPY bench /bench

# ---------------------------------------------------------------------------
# 2. The perception_suite ROS package. Source only: the checkout on the board also
#    holds ~3.3 GB of stray engines and test videos that nothing reads.
# ---------------------------------------------------------------------------
COPY src/perception_suite /src/perception_suite

# ---------------------------------------------------------------------------
# 3. The offroad_autonomy library (planning and perception), shimmed and installed.
#
#    semantic_shim.py runs three patches, and at the pinned commit only two of
#    them actually change anything. Worth knowing before reading its docstring,
#    which describes the third:
#
#      fix_class.py    picks the road class by what the model PREDICTS, not
#                      only by what it declares. Without it the new weights,
#                      which name road "Final-Demo", give 100% empty masks.
#      path_memory.py  stops the dashboard drawing planned path over the bonnet
#                      in the same solid colour as road it can actually see.
#      the shim itself  STANDS DOWN. It adapts RoadSegmenter to a
#                      task='semantic' model, but the library already supports
#                      that at this commit, so the shim detects it and returns
#                      without patching. It is kept because it is the chain's
#                      entry point and because it still applies to older trees.
#
#    All three are idempotent and exit 0 when already applied, so running this
#    against source that is already patched is a no-op rather than an error.
#    That is what makes the build work from the published src/ tree.
#
#    Applied to the copy, never to the source tree, exactly as the harness did.
# ---------------------------------------------------------------------------
COPY src/offroad_autonomy /src/offroad_autonomy
RUN cp -r /src/offroad_autonomy /tmp/build \
 && python3 /src/offroad_autonomy/patches/semantic_shim.py --repo /tmp/build \
 && pip install --no-deps --no-build-isolation /tmp/build \
 && rm -rf /tmp/build

# ---------------------------------------------------------------------------
# 4. Apply bev_overlay.py and build the workspace ONCE.
#
#    bev_overlay.py is the only overlay that ships. The other eight on the
#    board are instrumentation (control_overlay, stage_timing_overlay,
#    record_overlay, truth_overlay) or abandoned experiments (fused_overlay,
#    isaac_overlay, bev_panel_overlay, fast_dash_overlay).
#
#    It is load-bearing, not cosmetic. From its own docstring: 5_planner_node
#    builds one VehicleState from a static speed_mps parameter and reuses it
#    forever, so position never changes and the BEV grid can never shift -
#    "which is why the grid only ever worked in simulation". The overlay adds
#    the odom subscription, derives VehicleState.direction (which defaults to
#    None because it came from BeamNG, and without which ground_pose() in
#    bev_grid.py returns None), and turns planner_mode into a launch parameter.
#
#    So the configuration this project is shipping has never existed in the
#    upstream repo. Baking the overlay in here is what makes the image
#    self-contained; merging it upstream is still an open item.
# ---------------------------------------------------------------------------
RUN mkdir -p /opt/ws/src \
 && cp -r /src/perception_suite /opt/ws/src/ \
 && python3 /bench/scripts/bev_overlay.py --pkg /opt/ws/src/perception_suite \
 && python3 /bench/overlays/install_live_odom.py --pkg /opt/ws/src/perception_suite \
 && cd /opt/ws \
 && source /opt/ros/humble/setup.bash \
 && colcon build \
 && rm -rf /opt/ws/build /opt/ws/log

# ---------------------------------------------------------------------------
# 4b. ads_messages, in a SEPARATE overlay workspace.
#
#     This is our reconstruction of HATCI's message definitions, rebuilt from
#     the schema text embedded in their own bag files. Without it `ros2 bag
#     play` cannot deserialize /CAN or /vehicle_config and bag replay is
#     limited to the image topic.
#
#     It is deliberately NOT in /opt/ws. ROS 2 Humble matches a publisher to a
#     subscriber on the type NAME only, with no structural check, so if HATCI's
#     real package were installed alongside ours the two would be
#     indistinguishable and a version skew would decode their live topic into
#     silent garbage. Keeping ours in its own overlay means the entrypoint
#     chooses one explicitly rather than someone remembering to.
#
#     The entrypoint sources this for bag replay, and for live CAN only when
#     HATCI's package is absent. See replay/ads_messages/README.md.
# ---------------------------------------------------------------------------
COPY replay/ads_messages /src/ads_messages
RUN mkdir -p /opt/replay_ws/src \
 && cp -r /src/ads_messages /opt/replay_ws/src/ \
 && cd /opt/replay_ws \
 && source /opt/ros/humble/setup.bash \
 && colcon build \
 && rm -rf /opt/replay_ws/build /opt/replay_ws/log

# ---------------------------------------------------------------------------
# 5. Entrypoint. Sources ROS and the prebuilt workspace, then dispatches.
# ---------------------------------------------------------------------------
COPY docker/entrypoint.sh /opt/hatci/entrypoint.sh
RUN chmod +x /opt/hatci/entrypoint.sh

ENV YOLO_CONFIG_DIR=/tmp/Ultralytics
WORKDIR /opt/ws
ENTRYPOINT ["/opt/hatci/entrypoint.sh"]
CMD ["bag1", "can"]
