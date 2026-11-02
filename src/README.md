# Pipeline source

Two packages.
Together they are what runs in `hatci-vehicle:draft1`, the image every reported number was measured in.

| Path | What it is | Build system |
|---|---|---|
| `perception_suite/` | the six ROS 2 nodes and the launch file, 1,772 lines | ament_python, has a `package.xml` |
| `offroad_autonomy/` | the algorithm library the nodes import, 8,850 lines plus 3,671 of tests | plain Python, `pyproject.toml`, **not a ROS package** |

`perception_suite` is thin on purpose.
Each node owns a topic and a timer; the work happens in `offroad_autonomy`.
If you are estimating effort, the second one is where almost all of it is.

## Building

Two steps, in this order.

```
pip install --no-deps --no-build-isolation src/offroad_autonomy
colcon build
```

We have run exactly this in the image and confirmed it: `colcon build` finishes, the nine node executables appear under `install/perception_suite/lib/perception_suite/`, and both packages import.

`--no-deps` is not optional.
`offroad_autonomy`'s `pyproject.toml` declares `beamngpy`, `torch`, `ultralytics` and `scipy`.
`beamngpy` is the simulator client and there is no simulator on the vehicle, and `torch` and `ultralytics` are already present at the exact versions every reported number was measured against.
Resolving those dependencies would pull a simulator client onto the board and could upgrade the inference stack out from under the measurements.
`docker/Dockerfile` installs it the same way for the same reason.

### Why colcon does not build the library

`offroad_autonomy` carries a `COLCON_IGNORE`, which is deliberate and explained in that file.

colcon's `ament_python` build type in Humble runs `setup.py --dry-run` and then `ast.literal_eval`s the dumped metadata.
This package is defined by `pyproject.toml`, and its `[project]` table puts a `SpecifierSet` object and the entire README into that dump.
Neither is valid Python literal syntax, so the build fails with a `SyntaxError` before compiling a single module.
We tested both a `pyproject.toml`-only package and a hybrid with a minimal `setup.py`; both fail, the second for exactly that reason.

Making it a colcon package would mean moving its metadata out of `pyproject.toml` and back into a `setup.py`, which downgrades the packaging for every other consumer of the library.
A one-line `pip install` costs less and is what already runs.

`perception_suite/package.xml` does not declare `offroad_autonomy` as a `<depend>` either.
It is not in rosdistro, so `rosdep` cannot resolve it under either spelling and `rosdep install` would fail on it.
There is a comment in the manifest saying so.

## Topic chain

```
1_video_node / 1_camera_node -> raw_frames          (Image, stamped at capture)
2_preprocess_node            -> preprocessed_frames (Image)
3_segmentation_node          -> road_mask (Image) + segmented_frames (Image)
4_postprocess_node           -> stabilized_mask     (Image)
5_planner_node               -> path                (Path)
6_visualizer_node            -> display only
```

Headers propagate unchanged from capture, so any stage's latency is measurable against the frame's own stamp.
`live_odom_node` publishes `nav_msgs/Odometry` on `odom`, which `5_planner_node` consumes to shift the BEV grid.

Entry point: `perception_suite/launch/pipeline.launch.py`.
It takes `source:=video` or `source:=camera`.
There is no bag source yet; one is being added.

## One file is ahead of the image

`perception_suite/perception_suite/1_camera_node.py` is the single file here that does **not** match what the image carries.
The image's copy opens the camera with `cv2.VideoCapture(index, cv2.CAP_V4L2)` and never sets a pixel format.
On our carrier that produces a broken frame, and it had never been caught because every reported number was measured from recorded video rather than a live camera.

Measured on the bench carrier, same camera, same moment:

| Path | Result |
|---|---|
| OpenCV V4L2, default (negotiates YUYV) | top third of the image, the rest solid green fill |
| OpenCV V4L2, forced UYVY | diagonal shearing, every row offset from the one above |
| OpenCV V4L2, forced NV16 | black, with one grey band |
| **GStreamer pipeline, NV16** | **correct** |

So the node now builds a GStreamer pipeline instead. Two further findings are folded into it:

- The first frame off a freshly started stream carries torn rows, about 4% of pixels. It is clean by frame 5 and stays clean, so the warm-up is discarded rather than published. `warmup_frames` controls it.
- Our camera presents inverted. `rotate_180` handles it, and it defaults to **off**: a silent rotation would put the road at the top of the frame and the ground projection would be confidently wrong rather than obviously broken. Whether your camera needs it is item 3.3.

Verified on the board: the node builds, runs, and publishes clean right-side-up frames at the requested size, with 0.00% corrupted pixels.

**One operational consequence worth knowing before the demo.** This hardware does not hot-plug. The deserializer link is trained when the driver probes at boot, so a camera connected to a running board is never seen: the device opens cleanly and streams buffers of entirely zero bytes, with no error anywhere. The camera must be connected before the board is powered on. The node's error message says so, because it cost us most of a day.

## The two patches

`offroad_autonomy` here is the `orfd-lane-detection` repository at commit `6381043`, with two files patched.
We have verified this tree is byte-identical to what is installed in the image.

| File | Lines | Why |
|---|---|---|
| `src/offroad_autonomy/perception/road_segmenter.py` | 11 | Picks the road class by what the model actually predicts, not by class name. The Oct 2026 fine-tunes declare `traversable_road` and never emit it, so matching on the name returns an empty mask on every frame. |
| `src/offroad_autonomy/visualization/dashboard.py` | 70 | Path memory: path drawn over ground the camera can currently see is distinguished from ground it is only remembering. |

The patches are not hand edits.
`patches/semantic_shim.py` applies them, chaining `patches/fix_class.py` and `patches/path_memory.py`, and `docker/Dockerfile` runs that same script at image build time.
Run it against a clean checkout of `6381043` to reproduce this tree, or read the three scripts to see exactly what changed.

The first patch is the one that matters.
Without it the pipeline runs end to end and segments nothing.

## Present but not wired into the ROS pipeline

Written and tested, but no node imports them.
Worth knowing before you estimate:

- `offroad_autonomy/src/offroad_autonomy/control/` - the Stanley and MPC controllers, 1,193 lines.
  They ran during our benchmark work through a patch script, not through a node.
  `6_visualizer_node` currently builds an empty `ControlCommand()`, which is why the dashboard's steering and throttle dials read zero.
  A controller node is being added, as its own node rather than inside the planner.
- `offroad_autonomy/src/offroad_autonomy/simulation/beamng_client.py` - the simulator interface, 335 lines.
- `offroad_autonomy/src/offroad_autonomy/main.py` and `pipeline.py` - the original standalone non-ROS entry point.
  The six nodes replace it.

## Configuration

`offroad_autonomy/configs/` holds the library's own defaults, which describe the BeamNG test vehicle.
They are not what the reported numbers used.

The configuration behind every number we have reported is `configs/tmp_26s_demo.yaml` at the root of this repository.
Read that one.
