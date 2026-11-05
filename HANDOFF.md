# HATCI off-road perception stack: containerized handoff

Image tag `hatci-vehicle:draft7`, 34.1 GB on disk.

This runs the configuration **26s segmentation under the BEV grid planner**.
Nothing is installed on the host and nothing is read from the host filesystem: `docker load` the tarball and run it.

**Read `OPEN-ITEMS.md` alongside this.**
The live path works but has never run against a real vehicle bus; what we need confirmed is listed there.

---

## 1. Running it

Five ways to run the pipeline.
Four are demo or deployment paths; the fifth exists to benchmark, and is called out because it is easy to mistake for a demo path and quote the wrong numbers from it.

```bash
./run_vehicle.sh selftest                     # prove the image works on this board
./run_vehicle.sh clip1                        # 1. recorded clip, no vehicle data
./run_vehicle.sh replay /bags/<bagdir>        # 2. a real ROS bag, with its own CAN
./run_vehicle.sh replay /bags/<bagdir> nocan  #    the same bag, images only
./run_vehicle.sh camera                       # 3. live camera, no vehicle data
./run_vehicle.sh camera can                   # 4. live camera + live CAN   <- the vehicle path
./run_vehicle.sh clip1 can                    # 5. clip + recorded CAN csv  <- BENCHMARK ONLY
./run_vehicle.sh shell                        # a prompt inside the image
```

`clip2` and `clip3` exist alongside `clip1`.
Any ros2 launch argument is appended and wins: `./run_vehicle.sh clip1 can planner_mode:=baseline`.
`ros2 launch perception_suite pipeline.launch.py --show-args` inside `shell` lists all of them.

`can` turns odometry on and, with it, switches the planner from the baseline centreline to the BEV grid.
The grid needs ego motion to register one frame against the next, so the two belong to the same switch.
Mode 2 is the exception: a bag carries its own `/CAN`, so replay turns the grid on by itself, and `nocan` is how you turn it back off.

Measured on draft7 on our bench Jetson, all six configurations, headless, with the clocks unpinned after a reboot:

| Mode | `/road_mask` | `/path` | `/odom` |
|---|---|---|---|
| 1. `clip1` | 29.7 | 29.4 | - |
| 2. `replay <bag>` | 29.7 | 29.9 | 30.1 |
| 2. `replay <bag> nocan` | 28.3 | 28.6 | - |
| 3. `camera` | 29.1 | 29.3 | - |
| 4. `camera can`, with `/CAN` replayed from a bag | 29.9 | 30.0 | 29.9 |
| 5. `clip1 can` | 30.0 | 30.0 | 30.0 |

Rows 3 and 4 only show that the graph keeps up on the camera path.
We did not check the picture during those runs.

Mode 2 loses no frames.
The source node does report frame gaps while replaying, but those belong to the recording: a direct scan of `bag_20261006_125333`, with no pipeline involved, found 43 absent counters and 43 repeated ones and zero backward jumps, and three consecutive replays reproduced the same list byte for byte.
Do not lower `bag_rate` to silence them: replaying below real time scales the dead-reckoned ego motion by 1/rate and stretches the BEV grid by the same factor, silently, because the odometry and the grid decay both measure time by the wall clock rather than by the recorded stamps.
Item 1.9 in `OPEN-ITEMS.md` is the question that would let us fix that properly.

Mode 4's `/CAN` came from `ros2 bag play` feeding one of your bags, decoded with our reconstruction of `ads_messages`, because there is no vehicle bus on the bench.
It has never run against a real bus.

**Four things worth knowing before any of this touches a vehicle.**

Run `selftest` first on any new board.
It brings the pipeline up headless, waits the 45 s the engine needs to deserialize and warm up, then measures `/road_mask` and `/path` and exits non-zero if either is dead.
That is the cheapest way to find out whether the TensorRT engine is wrong for that board's JetPack, before anything is wired to a vehicle.

The `/CAN` subscription is BEST_EFFORT, deliberately.
Your bags publish `/CAN` as BEST_EFFORT, and DDS refuses to connect a RELIABLE reader to a BEST_EFFORT writer **silently**: the subscription exists, both ends appear in `ros2 topic info -v`, and not one message is delivered.
BEST_EFFORT matches either kind of publisher, so this cannot break whichever your live publisher uses.

Replaying a bag's `/CAN` needs the `ads_messages` definitions.
The image carries our reconstruction of them in a separate overlay at `/opt/replay_ws`, and prefers your package if yours is installed.
The entrypoint prints which one it used.

Every stage reports its own timing.
Each processing node logs one line every 5 s: its rate, the p50 and p95 of its own callback, and how old the frame was when it finished with it.
`age` is measured from the capture stamp every stage copies forward, so the difference between two stages' ages is the whole cost of the later one, transport included, which is what a single end-to-end number cannot tell you.
Under bag replay the stamps are the recording's rather than this clock, so the node says so once and reports work time only.
Measured in this image on mode 5 with the dashboard running as load:

```
preprocess    30.0 Hz | work p50  1.2 p95  2.3 ms |                        age p50  3.6 ms
segmentation  30.0 Hz | work p50 19.0 p95 25.9 ms | inference p50 10.1 ms | age p50 33.4 ms
postprocess   29.4 Hz | work p50  4.7 p95  8.4 ms |                        age p50 38.4 ms
planner       29.2 Hz | work p50 14.5 p95 20.4 ms |                        age p50 53.7 ms
```

So the planner is acting on frames about 54 ms old with the dashboard on, and 38 ms without it.
Half the segmentation node is not the model: 19.0 ms of work around a 10.1 ms `predict()` call.
The rest has not been timed piece by piece; it covers `cv_bridge` conversion in, mask extraction and clipping, encoding and publishing.

Verified on our bench Jetson (L4T R36.3.0, JetPack 6.0):

```
entrypoint: CAN odometry from bag_20261006_125333.csv, BEV grid planner
entrypoint: 7987 frames, 7998 CAN rows (0.1% apart)
=== selftest: 26s + BEV grid on clip1, headless ===
ok: /road_mask average rate: 29.994
ok: /path      average rate: 29.988
=== selftest PASSED ===
```

Those rates are a **liveness check, not a benchmark**.
The dashboard was off for this run, and the reported numbers were all measured with it running as load.
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
/src/perception_suite      ROS package source (also built into /opt/ws)
/src/perception_suite_msgs the two message types the planner and controller share
/src/offroad_autonomy      the planning and perception library, pip installed
/opt/ws/             the colcon workspace, prebuilt
/opt/hatci/entrypoint.sh
```

Those internal paths matter when you read a config.
The pipeline configs name weights by absolute path, `model_weights: "/bench/models/new/weights_26s.640.engine"`, and that is a path **inside the container**, not on your host.
Nothing in the image reads anything from the host filesystem.

The `.onnx` files are in the image on purpose.
The `.engine` files are tied to the TensorRT inside the image, 10.3.0.30, rather than to anything installed on your host.
TensorRT warns on every run, on our board too, that the engine plan came from a different model of device, and `trtexec` can rebuild from the ONNX without needing torch or ultralytics at all.
See items 2.1 and 5.7.

**Only 26s should be deployed.**
The two VOS-trained variants are present for completeness but reject their own perception 5 to 11 times more often than the originals, and should not be considered until R2 scores their segmentation accuracy.

## 3. Live CAN odometry

The BEV grid planner needs ego motion.
In the lab that came from `odom_node.py`, which replays a CSV extracted from the bags, one row per published frame.
Before this image there was no live odometry source anywhere in the stack: a grep for `socketcan`, `python-can`, `cantools` and `can0` across the whole of `perception_suite` returned nothing, and `run_demo.sh` enforced it outright with `'camera' has no recorded CAN; 'can' only works with a bag`.

Every number we have reported still comes from recorded odometry, because the benchmark matrix was run in mode 5.

`perception_suite/live_odom_node.py` in this image closes that gap.
It subscribes to the vehicle's CAN topic rather than reading raw bus frames, which is the right shape for the setup: the signals arrive decoded, so no DBC file and no raw-bus reader are needed.
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

**What we would still like from you** is your own `ads_messages` package, though it is no longer a dependency.
The full `.msg` text, including every nested type, is embedded in the bag files, which is how `extract_can.py` decodes them with no package installed, so we reconstructed it from there and it is in this image at `/opt/replay_ws`.
It is validated against the recorded data, and the entrypoint prefers your package over ours whenever yours is installed.
Your copy is still worth having for one reason: it removes the risk that your current schema has drifted from the October bags, which is the one thing our reconstruction cannot rule out.
This matters more now that the image runs on your host, because a package installed on the host is not visible inside the container.
Unless yours is added to the container, the entrypoint falls back to our reconstruction on a live bus, and says so in the log.
See section 1 and item 5.6 of `OPEN-ITEMS.md`.

The type is resolved at runtime rather than imported, so the module still loads on a board without `ads_messages` and the rest of the pipeline comes up normally.
It logs why it is quiet instead of taking the launch down.
If the type on your bus is not `ads_messages/msg/CANMsg`, pass `can_msg_type:=` rather than editing the node.

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
| `--runtime nvidia` | nvidia-container-toolkit. Mounts the host's driver, including `libcuda`. The CUDA 12.6.68 runtime and TensorRT 10.3.0.30 are in the image: both are present with the runtime removed, and `libcuda` is not. Ships with JetPack. |
| `--network host` | how the container sees `can0` at all. A bus up on the host needs no `--device` flag: verified by binding an `AF_CAN` socket to `can0` from inside this image with no other privilege. |
| `--ipc=host` | shares the host's IPC namespace, intended to let DDS use shared memory. Every reported number used it; its effect has not been measured on its own. |
| `--privileged` | **camera runs only.** The Tegra capture path needs `capture-vi-channel*`, `capture-isp-channel*`, `nvhost-ctrl-vi*`, `nvmap` and `tegra_camera_ctrl`. `--device` is not enough: the cgroup device rules still block the unlisted ones, and the symptom is the device opening cleanly and then every read timing out with `select() timeout`. Recorded runs do not need it and do not get it. |

Clock pinning is sysfs writes outside the container; nothing in the pipeline reads it and the stack runs either way.
Pinned is what the reported latency was measured at: in the R1 matrix, the unpinned run's median latency was 25 to 42 percent higher.
We also compared the two states directly on draft7, on the same day, in mode 5 with no dashboard, on the same code apart from comment edits:

| | Unpinned | Pinned |
|---|---|---|
| `predict()` p50 | 20.4 ms | 8.4 ms |
| Frame age at the planner, p50 | 60.3 ms | 37.6 ms |
| `/path` rate | 30.0 Hz | 30.0 Hz |

Frames reach the planner about 60 percent older unpinned, while the frame rate does not change.
That is above the R1 range, which was measured with the dashboard running, so the two figures are not directly comparable.
Pin the clocks on any deployment and before any latency measurement, and check them again after a reboot, because pinning did not survive one on our board.
The frame-rate requirement passes either way, so pinning is headroom rather than a dependency.

## 6. Known gaps in this draft

Honest list, so nothing is discovered on your board.

1. **Live CAN odometry has never run against a real vehicle bus.** It runs: mode 4 holds 30 FPS with `live_odom_node` subscribed to `/CAN`. But the publisher it has been tested against was `ros2 bag play` feeding one of your October bags, which reproduces the recorded type, QoS, rate and field layout and nothing beyond that. If your live bus differs in any of those, this is where it shows. Section 1.
2. **Replaying a bag below real time silently distorts the odometry.** `live_odom_node` integrates dead reckoning on wall-clock arrival time and the BEV grid decays on wall-clock too, neither of which reads the recorded stamps, so at `bag_rate:=r` the ego motion is scaled by 1/r and the grid stretches by the same factor with no warning. The default is 1.0, where it is correct, and the launch argument says not to lower it. It does not affect a live vehicle, where the two clocks agree. Item 1.9.
3. **Camera height and pitch are guesses.** 1.5 m and 0.0 degrees. Items 3.1, 3.2.
4. **The live camera path is tested on our carrier, not yours.** `camera_index:=0` is our Neousys carrier's GMSL capture, and the full pipeline runs live off it at 30 FPS. The node builds a plain `v4l2src` GStreamer pipeline with no NVIDIA-specific elements, so it is not tied to our hardware, but `camera` mode's defaults are: `camera_index:=0`, `pixel_format:=NV16`, 1920x1080 and `rotate_180:=true`. Append your own values to override them, and check the rotation first, because ours is on only because our camera is mounted inverted. Item 5.1. Note also that our carrier does not hot-plug, so the camera has to be attached before the board is powered on; item 5.4 has the detail.
5. **Fixed: the planner no longer latches a stale pose.** All three consumers of odometry now expire it on the same `odom_timeout_s`. `7_controller_node` commands a full brake, the dashboard zeroes its readouts, and `5_planner_node` drops the pose so `ground_pose()` returns None and the BEV grid forgets instead of accumulating. The old behaviour was the worse of the two failures: `on_odom` stored the pose with `valid=True` and nothing expired it, so new masks were registered against a position the vehicle had already left and observations from different real places piled into the same cells, corrupting the map rather than emptying it. Expiry is deliberately not armed until the first odometry message ever arrives, because a pipeline on a fixed `speed_mps` with no CAN source is a supported configuration. We triggered it on the board by killing the odometry node mid-run: the controller went to full brake within a second with the reason `odometry stale by 0.85s`, the planner logged the drop and kept publishing, and control recovered about three seconds after odometry returned. Path reach did not change during the outage, so that run cannot show the grid clearing; that part rests on the code and the unit tests. What is still open is what the *vehicle* should do about it, which is yours to specify: item 1.8 and item 4.4.
6. **Nothing on the vehicle side consumes `/command`.** The message itself is no longer a gap: it is a typed `perception_suite_msgs/ControlCommand` carrying a header, steering, throttle, brake, parking brake and a reason string, published at 30 Hz by `7_controller_node` and drawn on the dashboard. That replaces the headerless `Float32MultiArray` this item used to describe. What does not exist is anything that subscribes to it and actuates the car. Item 4.1.
7. **The image is 34.1 GB, and almost none of that is ours.** An earlier version of this item said trimming to 26s and one clip would cut it substantially. That was wrong, and the measurement is worth recording so nobody tries it expecting a large win. The base `hatci-pipeline:v1` is 31.5 GB of the 34.1; everything this repository adds is 2.6 GB, of which `/bench` is 1.33 GB (873 MB of clips and CAN, 395 MB of models). We built it to measure: dropping the other models and two clips gives 32.3 GB on disk and an 11.7 GB tarball, about 7 percent smaller, and it passes the selftest. The bulk is `/usr/lib/aarch64-linux-gnu` at 6.5 GB and `dist-packages` at 2.2 GB inside the base, which cannot be trimmed without changing the base, and changing the base would invalidate every number reported here. `SLIM=1 ./build.sh` does that trim if a transfer makes it worth losing modes `clip2` and `clip3`; it is off by default.

## 7. Rebuilding

```bash
TAG=hatci-vehicle:draft8 ./build.sh   # on a board that has hatci-pipeline:v1
```

`SLIM=1 ./build.sh` stages only the 26s weights and clip1, which saves about 0.9 GB off the tarball and costs you modes `clip2` and `clip3`. See known gap 7 for why the saving is small.

The default tag is `hatci-vehicle:draft7`, and `build.sh` refuses to overwrite a tag that already exists.
That is deliberate: rebuilding over a tag would replace a verified artifact with an unverified one of the same name, and nothing downstream could tell the difference.
`draft7` passes its selftest at 29.99 FPS on both `/road_mask` and `/path`.
It corresponds to commit `069ce2c`: every file there matches the tree the image was built from, except `HANDOFF.md`, `README.md` and `src/README.md`, which do not go into the image.
Pass a new `TAG`, or `FORCE=1` to overwrite on purpose.

`build.sh` stages the context from two places.
The three packages come from a clone of this repo, so the image is built from exactly the source in `src/` and nothing else.
What cannot be published comes off the board: the model weights and TensorRT engines, the demo clips, the extracted CAN csvs, and `bench-ros/scripts`.
It writes to no source tree, and it prints the commit it built from and warns when that tree is dirty, so an image can be traced back to a commit.

Delivering it:

```bash
docker save hatci-vehicle:draft7 | pigz -p 6 > hatci-vehicle-draft7.tar.gz
# on your board
pigz -dc hatci-vehicle-draft7.tar.gz | docker load     # or gunzip -c
```

This is the tarball route rather than a Dockerfile build, so you need no network access and no NGC login, and you get bit-for-bit the image we tested.

The built artifact, verified:

| | |
|---|---|
| file | `hatci-vehicle-draft7.tar.gz` |
| compressed | 12.5 GB (from 34.1 GB on disk) |
| sha256 | `060230cc1efec2fe439841dbcc2657b0ee4d4572eeaa621e1abb3af726e0bc18` |
| checked | `pigz -t` clean; valid OCI archive (`index.json`, `manifest.json`, `oci-layout`, 66 entries) |

Verify the checksum after transfer before loading:

```bash
sha256sum -c <<< "060230cc1efec2fe439841dbcc2657b0ee4d4572eeaa621e1abb3af726e0bc18  hatci-vehicle-draft7.tar.gz"
```

Loading needs roughly 34 GB free on your board on top of the 12.5 GB archive.
