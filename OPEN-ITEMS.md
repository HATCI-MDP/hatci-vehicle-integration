# Open integration items

Everything below is something we need confirmed or supplied before the stack can run on the vehicle.
It is ordered by what it blocks: section 1 is the one hard blocker, the rest can be answered in any order.
Read it alongside `HANDOFF.md`, which describes the image itself.

Where a row says a value is a guess, that is us flagging it rather than you discovering it.

---

## 1. CAN odometry: the one hard blocker

This is the only part of the stack that does not exist yet.
The configuration in our report, 26s segmentation plus the BEV grid planner, needs ego motion to register one camera frame against the next.
In the lab that motion was replayed from a CSV extracted from the Oct 2026 bags, one row per frame.

**We have built this on an assumption, and item 1.1 is where you confirm or correct it.**
We have assumed the node publishing `/CAN` runs on the live vehicle and not only in the recording rig.
If that is right, no DBC file is needed: we read the two signals off the decoded message rather than off raw bus frames.
If it is wrong, the node needs a raw socketcan reader and your DBC, which is a larger change and we should know early.

We have written the node that closes this gap and put it in the image at `perception_suite/live_odom_node.py`.
It subscribes to `/CAN` and takes exactly what `extract_can.py` takes out of the bag, `msg.spd_mps` and `msg.imu.imu.angular_velocity.z`, and it keeps the published contract of the replay node exactly.
The remaining questions are about getting the message type into our image and confirming a few conventions.

| # | What we need | Why it matters |
|---|---|---|
| 1.1 | **Can you send us the `ads_messages` package**, or confirm we may rebuild it from the schema embedded in the bags? | Our image needs the message definition to subscribe to `/CAN` at all. The full `.msg` text including every nested type is already inside the bag files, which is how `extract_can.py` decodes them with no package installed, so we can reconstruct it. The risk in doing that ourselves is version skew: if your current `ads_messages` has changed since the Oct 2026 bags, a reconstructed package may silently misread fields. Your copy removes that risk. |
| 1.2 | **What is the topic name on the vehicle**, and what `ROS_DOMAIN_ID` / DDS configuration does it publish under? | The bags recorded it as `/CAN`. Our container runs with `--network host`, so a topic visible on the host is visible inside, but only if the domain id and DDS discovery match. Cheapest possible failure to diagnose in advance. |
| 1.3 | **Are `spd_mps` and `imu.imu.angular_velocity.z` populated in SI units**, m/s and rad/s, as their names and the `sensor_msgs/Imu` spec say? | We now rely on the message contract rather than converting, so if either is actually km/h or deg/s the error is silent. A yaw rate left in deg/s is 57x too large and the grid will believe the car is spinning. It fails in a way that looks like a perception bug. The node warns once if either reading is implausible, but a confirmation is better than a guard. |
| 1.4 | **Sign convention for yaw rate**: is positive a left turn (counterclockwise from above, REP-103)? | A sign error puts the grid's accumulated road evidence on the wrong side of the road. That is worse than having no odometry at all, because it is confidently wrong. |
| 1.5 | **What do the `error`, `spd_error` and `imu.valid` flags mean**, and are any of them set during normal operation? | Our node currently skips any message where `error` or `spd_error` is true or `imu.valid` is false, on the assumption the names mean what they say. If one of those is set permanently on a healthy vehicle, the node goes silent and the grid never shifts. There is a `trust_flags:=false` escape hatch, but a straight answer is better. We do not gate on `imu.yaw_rate_status` because the meaning of its values is not in the bag's schema. |
| 1.6 | In **reverse**, does speed go negative or stay positive with the direction in a separate gear signal? | The bags carry a `gear` field we never used, because the clips were all forward. Dead reckoning integrates speed directly, so an unsigned speed in reverse drives the pose the wrong way. |
| 1.7 | Should we use a **single vehicle speed** or the **four wheel speeds**? | The bags carried both (`spd_mps` plus `spd_fl/fr/rl/rr_mps`). We used the single value. If your production signal set differs, or if you would rather we average the rears, tell us. |
| 1.8 | **What should the vehicle do if `/CAN` goes silent?** | Right now nothing detects it. `5_planner_node` marks every odometry message it receives as valid, so a stale pose would be consumed as if it were current. Our node publishes nothing when the last message is older than 0.5 s, which makes the grid fall back to forgetting. That is the conservative failure, but a real vehicle probably wants the planner to hold or hand back control, and that decision sits above this node. |

## 2. Container policy

Everything is dockerized because nothing can be installed on the host.
Two things in that constraint still need checking, because dockerizing does not fully remove the host dependency.

| # | What we need | Why it matters |
|---|---|---|
| 2.1 | **What JetPack / L4T version is the vehicle Jetson on?** One command: `cat /etc/nv_tegra_release` | The biggest single unknown. Our TensorRT engines are serialized against **TensorRT 10.3.0.30** (JetPack 6.0, L4T R36.3.0) and will refuse to deserialize on a different TensorRT. If they differ we ship the `.onnx` instead and build engines on your board at first run, which costs a few minutes once and then removes the version coupling entirely. We just need to know which package to build. |
| 2.2 | Is **`--runtime nvidia`** (nvidia-container-toolkit) present on the host? | This is the part dockerizing does not escape. CUDA and TensorRT userspace are mounted into the container from the host by that runtime, not baked into the image. It ships with JetPack, so it is almost certainly there, but without it nothing runs. |
| 2.3 | Can we run a container with **`--privileged`**? | Needed for the **live camera**, not for bag playback. The Tegra capture path needs far more than `/dev/videoN`: `capture-vi-channel*`, `capture-isp-channel*`, `nvhost-ctrl-vi*`, `nvmap` and `tegra_camera_ctrl`. Listing them with `--device` is not enough, because the cgroup device rules still block the unlisted ones, and the symptom is the device opening cleanly and then every read timing out with `select() timeout`. Worth raising explicitly: `--privileged` is a larger policy ask than installing a package, so if it is refused we need to know early and enumerate the device rules instead. |
| 2.4 | Are **`--network host`** and **`--ipc=host`** allowed? | `--network host` is how the container sees `can0` at all; with it, a bus that is up on the host needs no `--device` flag. `--ipc=host` is what lets the six nodes pass frames through shared memory instead of copying them. Every number in our report was measured with both. |
| 2.5 | **Docker version and free disk** on the vehicle board? | The image is roughly 33 GB on disk. Loading a tarball needs about that much again free during the import. |
| 2.6 | How should we **physically deliver** the tarball? | We have gone the `docker save` plus gzip route rather than a Dockerfile build, so you need no network access and no NGC login, and get bit-for-bit the image we benchmarked. Expect something in the 10 to 20 GB range compressed. That is a drive or a file-transfer service, not email. |

## 3. Camera and vehicle geometry

Good news first: the configs behind every reported number already describe **your** camera, not our bench one.
The geometry was derived from the Oct 2026 bags: 2880x1860, and `fov_h` 80.1 degrees computed from the bags' own CameraInfo (fx = 1714.2 over 2880 px gives 2·atan(2880/2·1714.2) = 80.06).
The hood mask was tuned against 1800 frames of your bags 1 and 3.
So this section is confirmation rather than discovery, with two genuine unknowns.

| # | What we need | Why it matters |
|---|---|---|
| 3.1 | **Camera height above ground, in metres?** | Config currently says `height_m: 1.5` and the comment next to it says it is a guess. The planner's image-to-ground mapping is the only thing that reads it, and it scales every distance the BEV grid produces. |
| 3.2 | **Camera pitch, in degrees?** | Config says `pitch_deg: 0.0`, also flagged as a guess. Same consequence as height: it is the other half of the ground projection. |
| 3.3 | Is the camera on the vehicle **the same unit, same lens, same mounting** as the one that recorded the bags? | If yes, our intrinsics are already right and 3.1/3.2 are the only open values. If it was moved or swapped, the whole geometry block needs redoing. |
| 3.4 | Will your camera node publish **rectified** frames, matching the bags' CameraInfo? | This one is easy to miss. The bag frames arrive already rectified, with every distortion coefficient zero, at an effective 80.1 degrees. The physical lens is a 120 degree fisheye. If the live node publishes raw fisheye instead, the image will look fine on the dashboard and the ground projection will be quietly wrong. |
| 3.5 | What **capture interface and device index** is the camera on your carrier? | Ours is GMSL at `camera_index:=2` on a Neousys carrier. Yours will differ; it is a launch argument, so this is a one-word answer. |
| 3.6 | Does the **hood sit at the same height in frame** on the vehicle as in the bags? | We mask everything below 0.67 of frame height as bodywork. It is a flat horizontal cut, not a traced silhouette, tuned over 1800 frames of bags 1 and 3. Too high and we throw away the nearest road the camera can see, which is what anchors the centreline walk; too low and the planner reads the bonnet as terrain. |
| 3.7 | Confirm **wheelbase 2815 mm**, **max road wheel angle 32 deg**, **camera-to-front-axle 1070 mm**. | We read these out of the bags' own `/vehicle_config` rather than guessing, so they should be right, but they drive the Stanley controller and the MPC geometry and are worth one line of confirmation. |

## 4. Integration and ownership

| # | What we need | Why it matters |
|---|---|---|
| 4.1 | **What consumes the steering output?** | `5_planner_node` publishes `/command` as a `Float32MultiArray` with no header. If there is an existing actuation interface it should probably be a proper typed message, and either way something on their side has to subscribe. |
| 4.2 | Can clocks be pinned on the vehicle board, and if so do you want it made persistent? | Pinning is sysfs writes outside the container, so nothing in the pipeline reads it and the stack runs either way. Pinned is what our reported latency numbers were measured at; unpinned adds roughly 35 percent to p50 and still passes the frame-rate requirement. |
| 4.3 | Confirm **26s is the model to deploy**. | 26s plus the BEV grid is what we benchmarked and what this image defaults to. The two VOS-trained variants are in the image but should not be deployed until R2 scores their segmentation accuracy: they reject their own perception 5 to 11 times more often than the originals. |
| 4.4 | What is the **handback and e-stop path**, and what are the **first-run conditions**? | Ties back to 1.8. Before this drives anything we should agree on what the safety driver sees and does, and whether first runs are on a closed course. |
