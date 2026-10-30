# HATCI off-road perception stack: handoff

**This folder is documentation. The code and data are in the image.**
These files describe how to run the stack, what is inside it, and what we still need from you.
The pipeline source, the ROS workspace, the model weights and the demo clips are all baked into `hatci-vehicle-draft1.tar.gz`, which ships separately because it is 12 GB compressed.
Once loaded, the source is readable and editable inside the image at `/opt/ws/src`.

**Read `OPEN-ITEMS.md` first.** It is what we need confirmed, ordered by what it blocks.

| File | What it is |
|---|---|
| `OPEN-ITEMS.md` | Everything we need confirmed or supplied, with why each one matters. |
| `HANDOFF.md` | Technical README for the image: how to run it, what is inside, what is still missing. |
| `run_vehicle.sh` | Host-side runner. Run `./run_vehicle.sh selftest` first on any new board. |
| `docker/Dockerfile` | How the image is assembled, for reference. |
| `docker/entrypoint.sh` | In-image dispatcher: `bag1 \| bag2 \| bag3 \| camera \| selftest \| shell`. |
| `overlays/live_odom_node.py` | The live odometry node. Subscribes to `/CAN`, publishes `/odom`. |
| `overlays/install_live_odom.py` | Wires that node into the launch graph at image build time. |

## Status

`hatci-vehicle:draft1` is built and passing its selftest on our bench Jetson.
It runs the reported configuration, 26s segmentation under the BEV grid planner, with nothing installed on the host and nothing read from the host filesystem.

One part is still incomplete.
Live odometry needs the `ads_messages` package inside the image so the node can subscribe to `/CAN`.
We have assumed that `/CAN` publishes on the live vehicle with speed and yaw rate already decoded, in which case no DBC file is needed. That assumption is unconfirmed and is item 1.1.
Until that package is in place, run with `planner_mode:=baseline`, which ignores ego motion entirely.
That is item 1.1 and it is the one hard blocker.

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
| The pipeline source | several thousand lines; it is built and installed already | in the image at `/opt/ws/src`, built at `/opt/ws/install` |
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
