# Open integration items

Everything below is something we need confirmed or supplied before the stack can run on the vehicle.
It is ordered by what it blocks.

**Updated 2026-11-02.** Several items have been answered since this was first written, by your hardware email and by your note on the CAN package.
Those are marked **ANSWERED** in place rather than deleted, so the record of what was decided stays readable.
Section 2 is answered in full.

Read it alongside `HANDOFF.md`, which describes the image, and `src/README.md`, which describes the source.

Where a row says a value is a guess, that is us flagging it rather than you discovering it.

---

## What changed

| | |
|---|---|
| **The demo runs on our Neousys.** | The host software is installed and validated: JetPack 6.0, L4T R36.3.0, TensorRT 10.3.0.30, which is the exact environment every reported number was measured in. This answers all of section 2 and item 4.2. |
| **You supply the CAN package.** | The signals reach us decoded on a topic, so no DBC file is needed and item 1.1 is closed. With the CAN-to-USB interface, we read the arrangement as your adapter plus your package publishing a topic our node subscribes to. That is the shape `live_odom_node.py` was already written for. |
| **You bring the camera.** | Which creates a new question we did not have before: it now has to connect to **our** carrier rather than yours. See 5.1. |

---

## 1. CAN odometry

The configuration in our report, 26s segmentation plus the BEV grid planner, needs ego motion to register one camera frame against the next.
In the lab that motion was replayed from a CSV extracted from the Oct 2026 bags, one row per frame.

`perception_suite/live_odom_node.py` closes the gap.
It subscribes to `/CAN` and takes exactly what `extract_can.py` takes out of the bag, `msg.spd_mps` and `msg.imu.imu.angular_velocity.z`, and it keeps the published contract of the replay node exactly.

What remains is a handful of conventions.
None of them is a blocker, but 1.3 and 1.4 fail silently if they are wrong, which makes them the two worth answering first.

| # | What we need | Why it matters |
|---|---|---|
| 1.1 | ~~Can you send us the `ads_messages` package?~~ **ANSWERED.** You are providing the CAN package. | For the record: we also confirmed we can reconstruct the message definition ourselves if ever needed. The full `.msg` text, including every nested type, is embedded in the bag files, which is how `extract_can.py` decodes them with no package installed. Your package removes the version-skew risk in doing that, so we would rather have yours. |
| 1.2 | **What `ROS_DOMAIN_ID` and DDS configuration does it publish under?** | The bags recorded the topic as `/CAN`. Our container runs with `--network host`, so a topic visible on the host is visible inside, but only if the domain id and DDS discovery match. Cheapest possible failure to diagnose in advance. |
| 1.3 | **Are `spd_mps` and `imu.imu.angular_velocity.z` in SI units**, m/s and rad/s? | We rely on the message contract rather than converting, so if either is km/h or deg/s the error is silent. A yaw rate left in deg/s is 57x too large and the grid will believe the car is spinning. It fails in a way that looks like a perception bug. The node warns once if a reading is implausible, but a confirmation is better than a guard. |
| 1.4 | **Sign convention for yaw rate**: is positive a left turn (counterclockwise from above, REP-103)? | A sign error puts the grid's accumulated road evidence on the wrong side of the road. That is worse than having no odometry at all, because it is confidently wrong. |
| 1.5 | **What do the `error`, `spd_error` and `imu.valid` flags mean**, and are any set during normal operation? | Our node skips any message where `error` or `spd_error` is true or `imu.valid` is false, on the assumption the names mean what they say. If one of those is set permanently on a healthy vehicle, the node goes silent and the grid never shifts. There is a `trust_flags:=false` escape hatch, but a straight answer is better. We do not gate on `imu.yaw_rate_status` because the meaning of its values is not in the bag's schema. |
| 1.6 | In **reverse**, does speed go negative or stay positive with direction in a separate gear signal? | The bags carry a `gear` field we never used, because the clips were all forward. Dead reckoning integrates speed directly, so an unsigned speed in reverse drives the pose the wrong way. |
| 1.7 | Should we use a **single vehicle speed** or the **four wheel speeds**? | The bags carried both (`spd_mps` plus `spd_fl/fr/rl/rr_mps`). We used the single value. If your production signal set differs, or you would rather we average the rears, tell us. |
| 1.8 | **What should the vehicle do if the CAN topic goes silent?** | Our node publishes nothing when the last message is older than 0.5 s, which makes the grid fall back to forgetting rather than acting on a stale pose. That is the conservative failure, but a real vehicle probably wants the planner to hold or hand back control, and that decision sits above this node. Ties to 4.4. |

## 2. Container policy: ANSWERED IN FULL

All six items are closed by the demo running on our Neousys.
Nothing needs to be installed on any host of yours, and no container policy of yours applies.

Kept here for one reason: **if deployment later moves to your AGX Orin, every one of these reopens.**
The most important is the old 2.1.
Our TensorRT engines are serialized against TensorRT 10.3.0.30 and will refuse to deserialize on a different build, so a move to your hardware means shipping the `.onnx` files and rebuilding engines there.
That costs a few minutes once and then removes the version coupling, but it is not a thing that happens by itself.

| # | Was | Now |
|---|---|---|
| 2.1 | What JetPack / L4T version is the vehicle Jetson on? | Ours. JetPack 6.0, L4T R36.3.0, TensorRT 10.3.0.30. |
| 2.2 | Is `--runtime nvidia` present on the host? | Ours. Present. |
| 2.3 | Can we run a container with `--privileged`? | Our board, so yes. We have since confirmed the live camera path works under `--privileged` with `/dev` mounted, and that the documented `select() timeout` failure does not occur. |
| 2.4 | Are `--network host` and `--ipc=host` allowed? | Ours. Both used, as every reported number was. |
| 2.5 | Docker version and free disk on the vehicle board? | Ours. Sufficient. |
| 2.6 | How should we physically deliver the tarball? | Probably not needed. The image stays on our board. The source is in this repository. |

## 3. Camera and vehicle geometry

The configs behind every reported number already describe **your** camera, not our bench one.
The geometry was derived from the Oct 2026 bags: 2880x1860, and `fov_h` 80.1 degrees computed from the bags' own CameraInfo (fx = 1714.2 over 2880 px gives 2·atan(2880/2·1714.2) = 80.06).
The hood mask was tuned against 1800 frames of your bags 1 and 3.

So this section is confirmation rather than discovery, with two genuine unknowns: 3.1 and 3.2.
**They are the only guessed values left anywhere in the configuration.**

| # | What we need | Why it matters |
|---|---|---|
| 3.1 | **Camera height above ground, in metres?** | Config says `height_m: 1.5`, flagged in the file as a guess. The planner's image-to-ground mapping is the only thing that reads it, and it scales every distance the BEV grid produces. |
| 3.2 | **Camera pitch, in degrees?** | Config says `pitch_deg: 0.0`, also flagged as a guess. The other half of the ground projection. |
| 3.3 | Is the camera you are bringing **the same unit, same lens, same mounting** as the one that recorded the bags? | If yes, our intrinsics are already right and 3.1 and 3.2 are the only open values. If it was moved or swapped, the whole geometry block needs redoing. Our understanding is that the demo would use the camera already fitted to the vehicle, which is the good case, but it is worth one line. |
| 3.4 | Will your camera node publish **rectified** frames, matching the bags' CameraInfo? | Easy to miss. The bag frames arrive already rectified, every distortion coefficient zero, at an effective 80.1 degrees. The physical lens is a 120 degree fisheye. If the live node publishes raw fisheye instead, the image looks fine on the dashboard and the ground projection is quietly wrong. |
| 3.5 | ~~What capture interface and device index is the camera on your carrier?~~ **Replaced by 5.1**, because the camera now connects to ours. | |
| 3.6 | Does the **hood sit at the same height in frame** as in the bags? | We mask everything below 0.67 of frame height as bodywork. A flat horizontal cut, not a traced silhouette, tuned over 1800 frames of bags 1 and 3. Too high and we throw away the nearest road the camera can see, which is what anchors the centreline walk. Too low and the planner reads the bonnet as terrain. |
| 3.7 | Confirm **wheelbase 2815 mm**, **max road wheel angle 32 deg**, **camera-to-front-axle 1070 mm**. | Read out of the bags' own `/vehicle_config` rather than guessed, so they should be right, but they drive the Stanley controller and the MPC geometry and are worth one line of confirmation. |

## 4. Integration and ownership

| # | What we need | Why it matters |
|---|---|---|
| 4.1 | **What consumes the steering output?** | The controller publishes a `Float32MultiArray` with no header. If there is an existing actuation interface it should be a proper typed message, and either way something on your side has to subscribe. Not needed for the camera-integration milestone, which does not require control. |
| 4.2 | ~~Can clocks be pinned on the vehicle board?~~ **ANSWERED.** Our board, so yes. | Pinned is what our reported latency numbers were measured at. Unpinned adds roughly 35 percent to p50 and still passes the frame-rate requirement. |
| 4.3 | Confirm **26s is the model to deploy**. | 26s plus the BEV grid is what we benchmarked and what this image defaults to. The two VOS-trained variants are in the image but should not be deployed until R2 scores their segmentation accuracy: they reject their own perception 5 to 11 times more often than the originals. |
| 4.4 | What is the **handback and e-stop path**, and what are the **first-run conditions**? | Ties back to 1.8. Before this drives anything we should agree on what the safety driver sees and does, and whether first runs are on a closed course. |

## 5. New, from running the demo on our hardware

These did not exist while the plan was to deploy onto your board.

| # | What we need | Why it matters |
|---|---|---|
| 5.1 | **What interface and connector does your camera use?** | It now has to connect to our Neousys carrier rather than yours. Ours is GMSL over coax, and we have a camera streaming on it. If yours is GMSL on the same connector family we are in good shape; if it is USB, GigE or a different serializer, we need to know now rather than on the day. This replaces the old 3.5. |
| 5.4 | **Not a question, a procedure note: the camera must be connected before the board is powered on.** | This hardware does not hot-plug. The deserializer link is trained when the driver probes at boot, so a camera connected to a running board is never seen. The failure is silent and convincing: the device opens, streaming starts, buffers arrive at the right size and cadence, and every byte in them is zero, with no error logged anywhere. A reboot with the camera already attached fixes it. Worth building into the demo-day sequence rather than discovering it in the vehicle. |
| 5.2 | **When do you want to install the CAN dependencies on our Neousys?** | Installing them on our system needs physical or network access to the board, and it should happen before it goes into the vehicle rather than after. |
| 5.3 | **When does our Neousys need to be in the vehicle, and for how long?** | Once it is installed we lose the remote access we currently develop and test against. Anything we need to measure on the bench has to happen before that date, so knowing it early changes what we prioritise. |
