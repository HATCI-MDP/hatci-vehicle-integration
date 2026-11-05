# HATCI off-road perception stack: handoff

**The pipeline source is in `src/`. The weights, clips and built workspace are in the image.**
The three packages under `src/` are the readable source: `perception_suite`, the ROS 2 nodes, `perception_suite_msgs`, the two message types they pass between them, and `offroad_autonomy`, the algorithm library they import.
See `src/README.md` for how they fit together and how to build them.
The model weights, the demo clips and the prebuilt workspace are baked into `hatci-vehicle-draft7.tar.gz`, which ships separately because it is 12 GB compressed.

**Read `OPEN-ITEMS.md` first.** It is what we need confirmed, ordered by what it blocks.

| File | What it is |
|---|---|
| `src/` | The pipeline source, all three packages. Start at `src/README.md`. |
| `configs/` | The run configuration behind every number we have reported. |
| `OPEN-ITEMS.md` | Everything we need confirmed or supplied, with why each one matters. |
| `HANDOFF.md` | Technical README for the image: how to run it, what is inside, what is still missing. |
| `run_vehicle.sh` | Host-side runner. Run `./run_vehicle.sh selftest` first on any new board. |
| `docker/Dockerfile` | How the image is assembled, for reference. |
| `docker/entrypoint.sh` | In-image dispatcher: `clip1 \| clip2 \| clip3 \| replay <bag> \| camera \| selftest \| shell`. |
| `overlays/live_odom_node.py` | The live odometry node. Subscribes to `/CAN`, publishes `/odom`. |
| `overlays/install_live_odom.py` | Wires that node into the launch graph at image build time. |
| `LICENSE` | MIT. |
| `NOTICE` | What the MIT licence does not cover, chiefly your own vehicle data in `configs/`. |

## Status

`hatci-vehicle:draft7` is built and passing its selftest on our bench Jetson, at 29.99 FPS on both `/road_mask` and `/path`.
It runs the reported configuration, 26s segmentation under the BEV grid planner, with nothing installed on the host and nothing read from the host filesystem.

All five run modes below work on that board.

Deployment has since moved to your compute platform: JetPack 6.1, L4T R36.4.0, CUDA 12.6.68.
The numbers in this document were measured on our Orin NX on JetPack 6.0, and the image has not yet been run on your platform.
See sections 2 and 5 of `OPEN-ITEMS.md`.

Live odometry no longer waits on the CAN message definition.
The image carries our own reconstruction of `ads_messages`, rebuilt from the schema embedded in your October bags, and it prefers your package if yours is installed.
The signals arrive decoded on that topic, so no DBC file is needed.
What is not yet confirmed is whether your live bus matches that recording.
The type name, the units and the sign conventions are section 1 of `OPEN-ITEMS.md`, and mode 4 has only ever run against a `/CAN` publisher fed from your bags.

## Running the system

Load the image once.
Everything after that is a single command.

```
gunzip -c hatci-vehicle-draft7.tar.gz | docker load
```

### Check it works on this board

```
./run_vehicle.sh selftest
```

Brings the pipeline up headless, waits 45 s for the engine to deserialize and warm up, then confirms `/road_mask` and `/path` are both publishing.
Exits non-zero if either is dead, and prints the last 40 log lines when it fails.
Run this first on any new board, before anything is connected to a vehicle.

### The five ways to run it

Four are demo or deployment paths.
The fifth exists to benchmark, and is listed separately because it is easy to mistake for a demo path and quote the wrong numbers from it.

| | Command | Input | Vehicle data | Planner |
|---|---|---|---|---|
| 1 | `./run_vehicle.sh clip1` | recorded mp4 | none | baseline centreline |
| 2 | `./run_vehicle.sh replay <bagdir>` | a real ROS bag | the bag's own `/CAN` | BEV grid |
| 3 | `./run_vehicle.sh camera` | live camera | none | baseline centreline |
| 4 | `./run_vehicle.sh camera can` | live camera | live `/CAN` | BEV grid |
| 5 | `./run_vehicle.sh clip1 can` | recorded mp4 | recorded CAN csv | BEV grid |

Any `ros2 launch` argument is appended and wins, for example `./run_vehicle.sh clip1 can planner_mode:=baseline`.
`ros2 launch perception_suite pipeline.launch.py --show-args` inside `shell` lists all of them.

The dashboard draws on the monitor attached to the board, not on the machine you connected from.
Over a plain ssh session there is no display, so the pipeline detects that and runs headless rather than failing.
For a window on your own machine, connect with `ssh -X`.

#### 1. A recorded clip, no vehicle data

```
./run_vehicle.sh clip1          # also clip2, clip3
```

The quickest proof that perception works, and the only mode that needs nothing but the image.
On draft7 with unpinned clocks, measured 29.7 FPS on `/road_mask` and 29.4 on `/path` on our bench Jetson.

These clips were previously called `bag1`, `bag2` and `bag3`, which was misleading, because they are mp4 files extracted from your October bags rather than bag playback.
The old names still work and print a warning.

#### 2. A real ROS bag, with its recorded CAN

```
./run_vehicle.sh replay /home/nvidia/bags/bag_20261006_125333
./run_vehicle.sh replay /home/nvidia/bags/bag_20261006_125333 nocan   # images only
```

Images and odometry come off one recording on one clock, so they are aligned by timestamp rather than by assumption.
This is the mode that answers requirement 1.

The bag path must be a directory, the one holding `metadata.yaml`.
`run_vehicle.sh` mounts its parent read-only at the same absolute path, so the path you type is the path `rosbag2` opens.
If `metadata.yaml` sits one level down inside a wrapper directory, the entrypoint finds it and says so.

Add `nocan` to replay images only.
That is how you tell a perception fault from a planning one, because it takes odometry and the grid out of the picture.

Measured 29.7 FPS on `/road_mask`, with no frame loss.
The source node does report frame gaps during replay, but they are in the recording rather than in our delivery: reading `bag_20261006_125333` directly found 43 absent counters and 43 repeated ones, with zero backward jumps, and repeated clean replays reproduce that list exactly.
Do not lower `bag_rate` to silence those warnings.
Replaying below real time scales the dead-reckoned ego motion by 1/rate and stretches the BEV grid by the same factor, with nothing warning you, because both the odometry and the grid decay measure time by the wall clock rather than by the recorded stamps.
See item 1.9 in `OPEN-ITEMS.md`.

Replaying the bag's `/CAN` needs the `ads_messages` definitions.
The image carries our own reconstruction of them, rebuilt from the schema embedded in your bags, and it prefers yours if yours is installed.
The entrypoint says which one it used.

#### 3. Live camera, no vehicle data

```
./run_vehicle.sh camera
./run_vehicle.sh camera camera_index:=0     # pick a different capture index
```

The camera bring-up path: proves capture, the GMSL link and the six-node graph with no vehicle attached.
Measured 29.1 FPS on `/road_mask` and 29.3 on `/path`, headless.
That shows the graph keeps up, not that the picture is good: we did not check the picture during that run.

With the dashboard drawing on the board's own screen, which is the workflow requirement 4 describes, and nothing road-like in front of the camera, this measures about 28 to 30 FPS and 28 to 34 ms capture to display at p50.
The perception gate stops on those frames and the dashboard has no path or mask contour to draw, so that is the optimistic end.

We have not yet measured the live camera with road in frame at a driving angle.
The closest real-road figure is a recorded clip on the same baseline planner with the dashboard and controller on: 71.8 ms p50, with `/path` at 29.8 Hz.
The BEV grid planner on a recorded clip with the dashboard on reads 91 to 93 ms p50.
Both were read just after a screen recording finished, so treat them as approximate.
Requirement 4's bar is no visible lag, which needs someone watching the live camera on real road.

Camera mode turns `rotate_180` on, because our camera is mounted inverted.
On a camera mounted the right way up, append `rotate_180:=false`, and check that the road is at the bottom of the frame.

Camera mode relaxes the container's device rules, because the Tegra capture path needs far more than `/dev/videoN`.
It also needs `capture-vi-channel*`, `capture-isp-channel*`, `nvhost-ctrl-vi*`, `nvmap` and `tegra_camera_ctrl`.
Listing those with `--device` is not sufficient, because the cgroup rules still block everything not listed, and the symptom is the device opening cleanly and then every read timing out with `select() timeout`.

The capture mode is `1920x1080` NV16, which is what our GMSL dashcam offers.
A different camera needs `capture_width:=` and `capture_height:=`, because V4L2 substitutes the nearest supported mode rather than failing.

#### 4. Live camera and live CAN

```
./run_vehicle.sh camera can
./run_vehicle.sh camera can can_msg_type:=your_pkg/msg/YourType
```

The vehicle path, and the configuration intended for your car.
Measured 29.9 FPS on `/road_mask` and 30.0 on `/path`, with `/CAN` replayed from one of your bags.
The caveat under mode 3 about the picture applies here too.

Two things are worth knowing before this is wired to a vehicle.

The `/CAN` subscription is BEST_EFFORT.
That is deliberate and was found the hard way: your bags publish `/CAN` as BEST_EFFORT, and DDS refuses to connect a RELIABLE reader to a BEST_EFFORT writer **silently**, so the subscription exists, both ends appear in `ros2 topic info -v`, and not one message arrives.
BEST_EFFORT matches either kind of publisher, so this can no longer break whichever your live publisher uses.

The message type defaults to `ads_messages/msg/CANMsg`, recovered from your bags, and is resolved at runtime.
A wrong type leaves the grid without odometry instead of failing the pipeline.
Override it with `can_msg_type:=` rather than editing the node.

This mode has only ever been validated against a `/CAN` publisher fed from your bags, never against a real bus.

#### 5. Recorded clip and recorded CAN, for benchmarking

```
./run_vehicle.sh clip1 can
```

**Benchmark only.**
This is the one configuration that runs the BEV grid planner at a full 30 FPS on this board, which is why every latency and frame-rate number we have reported comes from it.
Odometry is a csv extracted offline from the matching bag, replayed one row per published frame, so it never depends on wall-clock timing.

Mode 2 cannot replace it.
Bag replay now reaches about 30 FPS, but its rate is set by the replayer and the H.264 decoder rather than by the pipeline, so a number taken from it measures the wrong thing.

### Enter the container

```
./run_vehicle.sh shell
```

A bash prompt inside the image, with ROS 2 and the workspace already sourced.
Nothing else needs to be set up.

| Path | What is there |
|---|---|
| `/opt/ros/humble` | ROS 2 Humble, 193 packages |
| `/opt/ws/src` | the pipeline source |
| `/opt/ws/install` | the built workspace, including the node executables |
| `/opt/replay_ws` | our reconstructed `ads_messages`, kept out of `/opt/ws` so yours can replace it |
| `/bench/models` | the model weights and TensorRT engines |
| `/bench/configs` | the run configurations |
| `/bench/data` | the demo clips and the CAN logs |

The source is in the image, so you can edit a node and rerun `colcon build` in place.
Iterating on the code does not mean waiting for a new 12 GB image.

## Building the image

**A copy of this folder cannot build the image.**
The Dockerfile begins `FROM hatci-pipeline:v1`, and that base image is not public.
It was assembled by hand on our bench Jetson, and it is the exact environment every number in the R1 report was measured in: ROS 2 Humble, CUDA, TensorRT 10.3.0.30, ultralytics 8.4.155.

So these files are the integration recipe and the 12 GB tarball is the payload.
They travel together.
To run the pipeline, `docker load` the tarball and then `./run_vehicle.sh selftest`.

Recreating that base from scratch starts from `nvcr.io/nvidia/l4t-jetpack` and needs an NGC login, outbound network access, and a few hours.
It would also pick up newer package versions than the ones the report measured, so every reported number would need re-running.
Ask us before going down that path, because it is almost certainly not what you want.

## What is not in this folder

| Not here | Why | Where it is instead |
|---|---|---|
| The *built* workspace | the source is in `src/`; this is the colcon output | in the image at `/opt/ws/install` |
| Model weights and TensorRT engines, 395 MB | the engines are serialized against one exact TensorRT build | in the image at `/bench/models` |
| The three demo clips, 868 MB | footage recorded from your vehicle | in the image at `/bench/data/clips` |
| The CAN logs | extracted from your October bags; they carry `lat` and `lon` | in the image at `/bench/data/can` |
| The image itself, 12 GB compressed | too large to send with documentation | `hatci-vehicle-draft7.tar.gz`, delivered separately |

To see what is really inside the image without starting the pipeline:

```
docker run --rm --entrypoint bash hatci-vehicle:draft7 -lc "ls /bench /opt/ws/src"
```

## Two things worth knowing up front

1. **The camera geometry is already yours.** Every reported number used configs derived from the Oct 2026 bags: 2880x1860, `fov_h` 80.1 degrees computed from the bags' own CameraInfo. Not our bench camera. Height and pitch are the only guessed values left, items 3.1 and 3.2.
2. **Live odometry has never seen a real bus.** It now runs: mode 4 holds 30 FPS with `live_odom_node` subscribed to `/CAN`, using our reconstruction of `ads_messages`. But the publisher it has been tested against was fed from your bags, not from a vehicle. And every number we have reported still comes from the CSV path, mode 5, because that is the configuration the benchmark matrix was run in.
