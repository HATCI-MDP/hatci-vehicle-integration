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
#    semantic_shim.py teaches RoadSegmenter to consume a task='semantic' model.
#    Upstream reads result.masks and result.boxes.conf, which is the
#    task='segment' contract; a semantic model returns a full-resolution class
#    index map and has NO per-detection score. The shim substitutes road pixel
#    fraction within the ROI as the gate confidence, which is why these configs
#    carry their own gate threshold (0.0) and why their confidence values must
#    never be compared against a YOLOE model's.
#
#    Applied to the copy, never to the repo, exactly as the harness did.
# ---------------------------------------------------------------------------
COPY src/offroad_autonomy /src/offroad_autonomy
RUN cp -r /src/offroad_autonomy /tmp/build \
 && python3 /bench/scripts/semantic_shim.py --repo /tmp/build \
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
# 5. Entrypoint. Sources ROS and the prebuilt workspace, then dispatches.
# ---------------------------------------------------------------------------
COPY docker/entrypoint.sh /opt/hatci/entrypoint.sh
RUN chmod +x /opt/hatci/entrypoint.sh

ENV YOLO_CONFIG_DIR=/tmp/Ultralytics
WORKDIR /opt/ws
ENTRYPOINT ["/opt/hatci/entrypoint.sh"]
CMD ["bag1", "can"]
