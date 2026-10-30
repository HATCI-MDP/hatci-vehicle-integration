# HATCI off-road perception stack: containerized handoff

Image tag `hatci-vehicle:draft1`, 34.1 GB on disk.

This runs the configuration **26s segmentation under the BEV grid planner**.
Nothing is installed on the host and nothing is read from the host filesystem: `docker load` the tarball and run it.

**Read `OPEN-ITEMS.md` alongside this.**
One part of the live path is still incomplete and it is listed there, with everything else we need confirmed.

---

## 1. Running it

```bash
./run_vehicle.sh selftest        # prove the image works on this board
./run_vehicle.sh bag1 can        # recorded clip + recorded CAN  <- the reported config
./run_vehicle.sh bag2 can
./run_vehicle.sh bag3 can
./run_vehicle.sh camera can      # live camera + live CAN bus    <- see section 3
./run_vehicle.sh shell           # a prompt inside the image
```

Any ros2 launch argument is appended and wins: `./run_vehicle.sh bag1 can planner_mode:=baseline`.

`can` turns odometry on and, with it, switches the planner from the baseline centerline to the BEV grid.
The grid needs ego motion to register one frame against the next, so the two belong to the same switch.

**Run `selftest` first on any new board.**
It brings the pipeline up headless, waits the 45 s the engine needs to deserialize and warm up, then measures `/road_mask` and `/path` and exits non-zero if either is dead.
That is the cheapest way to find out whether the TensorRT engine is wrong for that board's JetPack, before anything is wired to a vehicle.

Verified on our bench Jetson (L4T R36.3.0, JetPack 6.0):

```
entrypoint: CAN odometry from bag_20261006_125333.csv, planner switches to the BEV grid
entrypoint: 7987 frames, 7998 CAN rows (0.1% apart)
=== selftest: 26s + BEV grid on bag1, headless ===
ok: /road_mask average rate: 30.385
ok: /path      average rate: 28.979
=== selftest PASSED ===
```

Those rates are a **liveness check, not a benchmark**.
This particular run shared the GPU with another container, and the dashboard was off while the reported numbers were all measured with it running as load.
The frame-rate evidence is the 40-run matrix in the R1 bundle: 283,477 frames across 9,578 one-second windows, none below 20 FPS.
What the selftest proves is narrower and is the thing worth proving on a new board: the image is self-contained, the TensorRT engine deserializes, CUDA works, and the full six-node graph reaches the planner.

## 2. What is inside

```
/bench/configs/      tmp_26s_demo.yaml          <- the default; the shipping config
                     tmp_new26n_demo.yaml       26n
                     tmp_26n_vos_demo.yaml      26n trained via VOS propagation
                     tmp_26s_vos_demo.yaml      26s trained via VOS propagation
                     (plus the other bench configs, unused by these runs)
/bench/models/new/   weights_26n.*  weights_26s.*     .pt, .onnx, .640.engine, .1024.engine
/bench/models/vos/   weights_26n_vos.*  weights_26s_vos.*
/bench/data/clips/   hatci_bag{1,2,3}_*.mp4     the three recorded drives
/bench/data/can/     bag_20261006_*.csv         CAN odometry extracted from your bags
/bench/overlays/     odom_node.py  live_odom_node.py  install_live_odom.py
/bench/scripts/      extract_can.py, export_engines.py, semantic_shim.py, probes
/src/perception_suite    ROS package source (also built into /opt/ws)
/src/offroad_autonomy    the planning and perception library, pip installed
/opt/ws/             the colcon workspace, prebuilt
/opt/hatci/entrypoint.sh
```

Those internal paths matter when you read a config.
The pipeline configs name weights by absolute path, `model_weights: "/bench/models/new/weights_26s.640.engine"`, and that is a path **inside the container**, not on your host.
Nothing in the image reads anything from the host filesystem.

The `.onnx` files are in the image on purpose.
If your TensorRT differs from ours the `.engine` files will not deserialize, and `trtexec` can rebuild from the ONNX without needing torch or ultralytics at all.
See item 2.1.

**Only 26s should be deployed.**
The two VOS-trained variants are present for completeness but reject their own perception 5 to 11 times more often than the originals, and should not be considered until R2 scores their segmentation accuracy.

## 3. The one incomplete part: live CAN odometry

The BEV grid planner needs ego motion.
In the lab that came from `odom_node.py`, which replays a CSV extracted from the bags, one row per published frame.
There is no live odometry source anywhere in the stack: a grep for `socketcan`, `python-can`, `cantools` and `can0` across the whole of `perception_suite` returns nothing, and `run_demo.sh` enforced it outright with `'camera' has no recorded CAN; 'can' only works with a bag`.

So the reported configuration has only ever run against recorded odometry.

`perception_suite/live_odom_node.py` in this image closes that gap.
It is built on the assumption that the node publishing `/CAN` runs on the live vehicle and not only in the recording rig, so it subscribes to that topic rather than reading raw bus frames.
**That assumption is ours and is not yet confirmed. It is item 1.1, and it is the first thing to settle.**
If it holds, no DBC file is needed. If `/CAN` exists only in the recording setup, this node needs a raw-bus reader and a DBC instead, which is a bigger change.
It takes exactly what `extract_can.py` takes out of the bag, `msg.spd_mps` and `msg.imu.imu.angular_velocity.z`, and it keeps `odom_node`'s published contract exactly:

| | |
|---|---|
| topic | `odom` |
| type | `nav_msgs/Odometry` |
| QoS | RELIABLE, KEEP_LAST, depth 1 (`perception_suite` `LATEST_ONLY`) |
| `header.stamp` | the stamp of the **frame** the sample is paired with, never `now()` |
| `header.frame_id` / `child_frame_id` | `odom` / `base_link` |
| `pose.position` | x, y in metres, dead-reckoned |
| `pose.orientation` | yaw-only quaternion |
| `twist.linear.x` | speed, m/s |
| `twist.angular.z` | yaw rate, rad/s |

Two design points worth stating, because both look like they could be simplified and cannot:

**It still publishes one sample per frame**, driven by the `raw_frames` subscription, rather than publishing as CAN arrives.
`5_planner_node` pairs this odometry with the mask derived from that frame, and the pipeline is several stages deep, so a `now()` stamp would pair a fresh pose with an old mask.
Publishing on CAN arrival would also give the planner two updates for one frame, or none.

**Dead reckoning moved from offline to online.**
`extract_can.py` integrated the whole drive in one pass beforehand; here x, y and heading accumulate as samples arrive, using the same arithmetic.
Absolute drift does not matter, because the grid only ever compares consecutive poses and decays anything older than `memory_s`.
The offline integration was validated against the bags' own GPS at 0.6 to 1.3 percent of path length over roughly 2 km, with median position error 0.3 to 0.6 m over the 1 to 2 s the grid actually remembers.

**What is still needed** is the `ads_messages` package inside the image, so the node can resolve `ads_messages/msg/CANMsg` and subscribe at all.
The full `.msg` text, including every nested type, is embedded in the bag files, which is how `extract_can.py` decodes them with no package installed.
So we can reconstruct it ourselves; your own copy is preferable only because it removes the risk that your current schema has drifted from the October bags.
That is item 1.1.

The type is resolved at runtime rather than imported, so the module still loads on a board without `ads_messages` and the rest of the pipeline comes up normally.
It logs why it is quiet instead of taking the launch down.
Until the package is in place, run the vehicle with `planner_mode:=baseline`, which ignores ego motion entirely.

The node also guards the two mistakes that would otherwise be silent.
It skips any message with `error` or `spd_error` set or `imu.valid` false, overridable with `trust_flags:=false`, and it warns once if the yaw rate or speed reads past anything a road vehicle does, which is what a deg/s or km/h mix-up looks like.

**Container networking is already verified.** From inside this image on our bench board, with only `--network host` and no `--device` and no `--privileged`, a raw socketcan bind to `can0` succeeded. The same `--network host` is what lets the container see a ROS topic published on the host, subject to a matching `ROS_DOMAIN_ID` (item 1.2). So nothing about container plumbing stands between this and live odometry.

## 4. Camera geometry

The configs behind every reported number already describe **your** camera, derived from the Oct 2026 bags rather than from our bench hardware:

```
width: 2880   height: 1860   fov_h: 80.1
```

`fov_h` is computed from the bags' own CameraInfo: fx = 1714.2 over 2880 px gives 2·atan(2880 / 2·1714.2) = 80.06 degrees.

The hood `ego_mask` was tuned and validated against your footage: over 1800 frames of bags 1 and 3, moving the line from 0.63 to 0.67 took gate stops from 75 to 46 on bag 1 and 89 to 63 on bag 3, and the recovered band was confirmed to be road rather than bodywork (called road 0.848 of the time in the centre against 0.156 at the edges).
Note it is a **flat horizontal cut at 0.67**, not a traced silhouette, and the comment above it still refers to `demo_route.mp4` from the config it was derived from. If your hood sits differently in frame, this needs rechecking.

Our bench GMSL dashcam is a different lens (1920x1080, 120 degrees) with its own separate config, and it is not what any reported number used.

Two values in that block are still guesses and are marked as such in the file: `height_m: 1.5` and `pitch_deg: 0.0`.
Only the planner's image-to-ground mapping reads them, but they scale every distance the BEV grid produces. Items 3.1 and 3.2.

One trap worth repeating from `OPEN-ITEMS.md`: the bag frames arrive **already rectified**, with every distortion coefficient zero, at an effective 80.1 degrees, while the physical lens is a 120 degree fisheye.
If your live camera node publishes raw fisheye instead, the dashboard will look fine and the ground projection will be quietly wrong. Item 3.4.

## 5. Host requirements

Dockerizing does not remove these.

| | |
|---|---|
| `--runtime nvidia` | nvidia-container-toolkit. CUDA and TensorRT userspace are mounted in from the host, not baked into the image. Ships with JetPack. |
| `--network host` | how the container sees `can0` at all. A bus up on the host needs no `--device` flag: verified by binding an `AF_CAN` socket to `can0` from inside this image with no other privilege. |
| `--ipc=host` | lets the six nodes pass frames through shared memory rather than copying. Every reported number used it. |
| `--privileged` | **camera runs only.** The Tegra capture path needs `capture-vi-channel*`, `capture-isp-channel*`, `nvhost-ctrl-vi*`, `nvmap` and `tegra_camera_ctrl`. `--device` is not enough: the cgroup device rules still block the unlisted ones, and the symptom is the device opening cleanly and then every read timing out with `select() timeout`. Recorded runs do not need it and do not get it. |
| TensorRT 10.3.0.30 | only for the prebuilt `.engine` files. Rebuild from the bundled `.onnx` if yours differs. |

Clock pinning is sysfs writes outside the container; nothing in the pipeline reads it and the stack runs either way.
Pinned is what the reported latency was measured at: pinning cut p50 latency by 25 to 42 percent across the matrix, averaging about a third.
The frame-rate requirement passes either way, so pinning is headroom rather than a dependency.

## 6. Known gaps in this draft

Honest list, so nothing is discovered on your board.

1. **`ads_messages` is not in the image.** Live odometry publishes nothing until the message package is available, because the node cannot resolve the `/CAN` type without it. Section 3, item 1.1.
2. **Camera height and pitch are guesses.** 1.5 m and 0.0 degrees. Items 3.1, 3.2.
3. **The live camera path is untested against your hardware.** `camera_index:=2` is our Neousys carrier's GMSL capture.
4. **Nothing detects missing odometry.** `5_planner_node` marks every odometry message it receives as valid. Our node goes silent rather than publishing a stale pose, which makes the grid forget, but a vehicle probably wants the planner to hold or hand back control. Item 1.8.
5. **`/command` is a `Float32MultiArray` with no header.** Needs a real typed interface and something to subscribe to it. Item 4.2.
6. **The image is 34 GB.** It carries all four models at two input sizes each and three 300 MB demo clips. Trimming to 26s only and one clip would cut it substantially; worth doing once the configuration is locked.

## 7. Rebuilding

```bash
./build.sh                       # on a board that has hatci-pipeline:v1
TAG=hatci-vehicle:v2 ./build.sh  # under another tag
```

`build.sh` assembles a context from the scattered source trees rather than pointing at one repo, skipping the roughly 3.3 GB of stray engines and test videos and the 740 MB `.git` that nothing reads. It writes to no source tree.

Delivering it:

```bash
docker save hatci-vehicle:draft1 | pigz -p 6 > hatci-vehicle-draft1.tar.gz
# on your board
pigz -dc hatci-vehicle-draft1.tar.gz | docker load     # or gunzip -c
```

This is the tarball route rather than a Dockerfile build, so you need no network access and no NGC login, and you get bit-for-bit the image we benchmarked.

The built artifact, verified:

| | |
|---|---|
| file | `hatci-vehicle-draft1.tar.gz` |
| compressed | 12 GB (from 34.1 GB on disk) |
| sha256 | `627cccac47d0676af927733ba7d41addc1f95ccb7471a18bef60bc94d985087b` |
| checked | `pigz -t` clean; valid OCI archive (`index.json`, `manifest.json`, `oci-layout`, 62 entries) |

Verify the checksum after transfer before loading:

```bash
sha256sum -c <<< "627cccac47d0676af927733ba7d41addc1f95ccb7471a18bef60bc94d985087b  hatci-vehicle-draft1.tar.gz"
```

Loading needs roughly 34 GB free on your board on top of the 12 GB archive.
