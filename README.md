# HATCI off-road perception stack: handoff

**The pipeline source is in `src/`. The weights, clips and built workspace are in the image.**
The two packages under `src/` are the readable source: `perception_suite`, the six ROS 2 nodes, and `offroad_autonomy`, the algorithm library they import.
See `src/README.md` for how they fit together and how to build them.
The model weights, the demo clips and the prebuilt workspace are baked into `hatci-vehicle-draft1.tar.gz`, which ships separately because it is 12 GB compressed.

**Read `OPEN-ITEMS.md` first.** It is what we need confirmed, ordered by what it blocks.

| File | What it is |
|---|---|
| `src/` | The pipeline source, both packages. Start at `src/README.md`. |
| `configs/` | The run configuration behind every number we have reported. |
| `OPEN-ITEMS.md` | Everything we need confirmed or supplied, with why each one matters. |
| `HANDOFF.md` | Technical README for the image: how to run it, what is inside, what is still missing. |
| `run_vehicle.sh` | Host-side runner. Run `./run_vehicle.sh selftest` first on any new board. |
| `docker/Dockerfile` | How the image is assembled, for reference. |
| `docker/entrypoint.sh` | In-image dispatcher: `bag1 \| bag2 \| bag3 \| camera \| selftest \| shell`. |
| `overlays/live_odom_node.py` | The live odometry node. Subscribes to `/CAN`, publishes `/odom`. |
| `overlays/install_live_odom.py` | Wires that node into the launch graph at image build time. |
| `LICENSE` | MIT. |
| `NOTICE` | What the MIT licence does not cover, chiefly your own vehicle data in `configs/`. |

## Status

`hatci-vehicle:draft1` is built and passing its selftest on our bench Jetson.
It runs the reported configuration, 26s segmentation under the BEV grid planner, with nothing installed on the host and nothing read from the host filesystem.

One part is still incomplete.
Live odometry needs the CAN message definition inside the image so the node can subscribe to the vehicle's CAN topic.
The signals arrive decoded on that topic, so no DBC file is needed and `live_odom_node.py` is the right shape for it.
Until the definition is in the image, run with `planner_mode:=baseline`, which ignores ego motion entirely.
A step rather than a blocker. Section 1 of `OPEN-ITEMS.md` lists the conventions still worth confirming.

## Running the system

Load the image once.
Everything after that is a single command.

```
gunzip -c hatci-vehicle-draft1.tar.gz | docker load
```

### Check it works on this board

```
./run_vehicle.sh selftest
```

Brings the pipeline up headless, waits 45 s for the engine to deserialize and warm up, then confirms `/road_mask` and `/path` are both publishing.
Exits non-zero if either is dead, and prints the last 40 log lines when it fails.
Run this first on any new board, before anything is connected to a vehicle.

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
| `/bench/models` | the model weights and TensorRT engines |
| `/bench/configs` | the run configurations |
| `/bench/data` | the demo clips and the CAN logs |

The source is in the image, so you can edit a node and rerun `colcon build` in place.
Iterating on the code does not mean waiting for a new 12 GB image.

### Launch the full application

```
./run_vehicle.sh bag1 can
```

That runs the configuration behind every number we reported: 26s segmentation under the BEV grid planner, with recorded odometry.
Any `ros2 launch` argument can be appended and it wins, for example `./run_vehicle.sh bag1 can planner_mode:=baseline`.

The dashboard draws on the monitor attached to the board, not on the machine you connected from.
Over a plain ssh session there is no display, so the pipeline detects that and runs headless rather than failing.
For a window on your own machine, connect with `ssh -X`.

**A naming warning.** The modes `bag1`, `bag2` and `bag3` do **not** play a ROS bag.
Each one is an mp4 clip plus a CAN csv, both extracted offline from the corresponding October bag.
We are renaming these to `clip1`, `clip2` and `clip3` to stop them being mistaken for bag playback.

### Select the input at runtime: camera

```
./run_vehicle.sh camera                   # the default capture index
CAMERA_INDEX=0 ./run_vehicle.sh camera    # pick a different one
```

Camera mode relaxes the container's device rules, because the Tegra capture path needs far more than `/dev/videoN`.
It also needs `capture-vi-channel*`, `capture-isp-channel*`, `nvhost-ctrl-vi*`, `nvmap` and `tegra_camera_ctrl`.
Listing those with `--device` is not sufficient, because the cgroup rules still block everything not listed, and the symptom is the device opening cleanly and then every read timing out with `select() timeout`.

Add `can` to use live odometry rather than running the baseline planner:

```
./run_vehicle.sh camera can
```

That path is not yet usable.
It needs the CAN message definition built into the image, which is the package you are supplying.
The decoded-versus-raw question this used to raise is settled: decoded, on a topic.

### Select the input at runtime: ROS bag

**Not implemented yet.**
The launch file currently has two sources, `video` and `camera`.
There is no bag source, so there is no `bag_path:=` argument to give.

We are adding one.
It will subscribe to the bag's image topic, decode it if the frames are compressed, and republish with the bag's own header timestamp so the latency measurements stay meaningful.
Playing a bag's `/CAN` topic additionally needs the `ads_messages` message definition.
We have confirmed that can be reconstructed from the schema embedded in the bag files, so bag playback does not wait on anything from you.

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
| The image itself, 12 GB compressed | too large to send with documentation | `hatci-vehicle-draft1.tar.gz`, delivered separately |

To see what is really inside the image without starting the pipeline:

```
docker run --rm --entrypoint bash hatci-vehicle:draft1 -lc "ls /bench /opt/ws/src"
```

## Two things worth knowing up front

1. **The camera geometry is already yours.** Every reported number used configs derived from the Oct 2026 bags: 2880x1860, `fov_h` 80.1 degrees computed from the bags' own CameraInfo. Not our bench camera. Height and pitch are the only guessed values left, items 3.1 and 3.2.
2. **Live odometry has never run.** The reported configuration has only ever run against a CSV replayed from the bags. The live node is written and wired; it needs `ads_messages` to subscribe to `/CAN`.
