# Pipeline source

Three packages.
Together they are the code that runs in `hatci-vehicle:draft7`.
The weights, clips and CAN logs are not in this repo and come off our board at build time.

| Path | What it is | Build system |
|---|---|---|
| `perception_suite/` | the ROS 2 nodes and the launch file | ament_python, has a `package.xml` |
| `perception_suite_msgs/` | the two message types the planner and controller pass between them | ament_cmake, needed to run the IDL generators |
| `offroad_autonomy/` | the algorithm library the nodes import, 8,873 lines plus 3,688 of tests | plain Python, `pyproject.toml`, **not a ROS package** |

`perception_suite` is thin on purpose.
Each node owns a topic and a timer; the work happens in `offroad_autonomy`.
If you are estimating effort, the second one is where almost all of it is.

## Building

Two steps, in this order.

```
pip install --no-deps --no-build-isolation src/offroad_autonomy
colcon build
```

We have run exactly this in the image and confirmed it: `colcon build` finishes and the node executables appear under `install/perception_suite/lib/perception_suite/`.
`colcon build` takes no package list because `perception_suite` declares a dependency on `perception_suite_msgs`, so colcon orders the two itself.
`offroad_autonomy` carries a `COLCON_IGNORE` and is skipped; it is the `pip install` above.

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
                             -> plan                (PlannerOutput)
6_visualizer_node            -> display only
7_controller_node            -> command             (ControlCommand)
```

`7_controller_node` only runs with `control:=true` and is off by default, so the
default graph is the six nodes above it.
`path` carries the centreline already projected to ground metres and is unchanged.
`plan` carries it in image pixels, which is what the controller needs: it runs its own
projection with its own range limit and then keeps using the pixel values, so ground
metres are the wrong data rather than merely the wrong units.
Both message types are defined in the `perception_suite_msgs` package.

Headers propagate unchanged from capture, so any stage's latency is measurable against the frame's own stamp.
`live_odom_node` publishes `nav_msgs/Odometry` on `odom`, or `odom_node` from a recorded csv, which `5_planner_node` consumes to shift the BEV grid.

Entry point: `perception_suite/launch/pipeline.launch.py`.
It takes `source:=video`, `source:=camera` or `source:=bag`.

## Why the camera node uses GStreamer

An earlier `1_camera_node.py` opened the camera with `cv2.VideoCapture(index, cv2.CAP_V4L2)` and never set a pixel format.
On our carrier that produces a broken frame, and it had never been caught because every reported number was measured from recorded video rather than a live camera.

Measured on the bench carrier, same camera, same moment:

| Path | Result |
|---|---|
| OpenCV V4L2, default (negotiates YUYV) | top third of the image, the rest solid green fill |
| OpenCV V4L2, forced UYVY | diagonal shearing, every row offset from the one above |
| OpenCV V4L2, forced NV16 | black, with one grey band |
| **GStreamer pipeline, NV16** | **correct** |

So the node now builds a GStreamer pipeline instead. Two further findings are folded into it:

- A freshly started stream carries torn rows that fade over about four seconds: 0.9% of pixels are still torn at frame 5, and the first wholly clean frame is frame 120. The warm-up is discarded rather than published. `warmup_frames` controls it and defaults to 120.
- Our camera presents inverted. `rotate_180` handles it. The node and the launch file default it to off, but `camera` mode in the entrypoint turns it on for our camera, so append `rotate_180:=false` if yours is the right way up. A wrong setting puts the road at the top of the frame and makes the ground projection confidently wrong rather than obviously broken. Item 5.1.

Verified on the board: the node builds, runs, and publishes clean right-side-up frames at the requested size, with 0.00% corrupted pixels.

**One operational consequence worth knowing before the demo.** This hardware does not hot-plug. The deserializer link is trained when the driver probes at boot, so a camera connected to a running board is never seen: the device opens cleanly and streams buffers of entirely zero bytes, with no error anywhere. The camera must be connected before the board is powered on. The node's error message says so, because it cost us most of a day.

## The patches

`offroad_autonomy` here starts from the `orfd-lane-detection` repository at commit `6381043`.
Three scripts patch two of its files, listed below.
Other differences are listed under "Where this copy differs from upstream".
The image is built from this tree.

| File | Lines | Why |
|---|---|---|
| `src/offroad_autonomy/perception/road_segmenter.py` | 11 | Picks the road class by what the model actually predicts, not by class name. The Oct 2026 fine-tunes declare `traversable_road` and never emit it, so matching on the name returns an empty mask on every frame. |
| `src/offroad_autonomy/visualization/dashboard.py` | 70 | Path memory: path drawn over ground the camera can currently see is distinguished from ground it is only remembering. |
| `src/offroad_autonomy/visualization/dashboard.py` | 22 | The latency tile says `n/a` instead of drawing a number nobody measured. Capture-to-display needs the capture stamp to be on this clock, and under bag replay it is the recording's own time. |

The patches are not hand edits.
`patches/semantic_shim.py` applies them, chaining `patches/fix_class.py`, `patches/path_memory.py` and `patches/latency_na.py`, and `docker/Dockerfile` runs that same script at image build time.
Run it against a clean checkout of `6381043` to reproduce those two files, which we have confirmed byte for byte, or read the four scripts to see exactly what changed.
Each one is idempotent and exits 0 when already applied, so running the chain against this tree is a no-op rather than an error.

The first patch is the one that matters.
Without it the pipeline runs end to end and segments nothing.
The other two are honesty rather than function: both stop the dashboard presenting something it did not measure as though it had.

## Where this copy differs from upstream

Beyond the two patched files, compared against `6381043`:

- `src/offroad_autonomy/runtime/timing.py` and `tests/test_runtime.py` add a real p50 to `StageStats`. These are direct edits, and no patch script applies them.
- Added here: `patches/`, `scripts/run_checks.sh`, `COLCON_IGNORE` and `LICENSE`.
- Left out here: `.github/`, `uv.lock` and `.coverage`.

The library's own documents are unedited, so three things in them do not match this copy:

- `offroad_autonomy/README.md` describes `.github/workflows/ci.yml`, which is not in this tree.
- The same README says the latency tile turns red past its budget. It does not mention that the tile reads `n/a` when latency cannot be measured.
- `offroad_autonomy/docs/design/swappable-driving-stacks.md` is a proposal dated 2026-10-01, and it says `--record-video` is not on this branch. It is.

## Control, and what is not wired in

`offroad_autonomy/src/offroad_autonomy/control/` holds the Stanley and MPC controllers, 1,194 lines.
They run in `7_controller_node`, which is their own node rather than code inside the planner.
`6_visualizer_node` subscribes to `command` and `odom`, so the dashboard's steering, throttle, brake and speed readouts are live.
They fall back to zero when their topic goes quiet, because a dial frozen on the last value reads as live when it is not.

Written and tested, but no node imports these.
Worth knowing before you estimate:

- `offroad_autonomy/src/offroad_autonomy/simulation/beamng_client.py` - the simulator interface, 335 lines.
- `offroad_autonomy/src/offroad_autonomy/main.py` and `pipeline.py` - the standalone non-ROS BeamNG loop.
  The ROS nodes are the vehicle path; these remain the simulator path.

## Configuration

`offroad_autonomy/configs/` holds the library's own defaults, which describe the BeamNG test vehicle.
They are not what the reported numbers used.

The configuration behind every number we have reported is `configs/tmp_26s_demo.yaml` at the root of this repository.
Read that one.
