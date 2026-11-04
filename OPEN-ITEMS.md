# Open integration items

Everything below is something we need confirmed or supplied before the stack can run on the vehicle.
It is ordered by what it blocks.

**Updated 2026-11-02.** Several items have been answered since this was first written: by your hardware email, by your note on the CAN package, and by decoding your October bags directly.
Those are marked **ANSWERED** in place rather than deleted, so the record of what was decided stays readable.
Section 2 is answered in full.
Four more were answered out of the bags themselves rather than by asking you, which is noted on each.

Read it alongside `HANDOFF.md`, which describes the image, and `src/README.md`, which describes the source.

Where a row says a value is a guess, that is us flagging it rather than you discovering it.

---

## What changed

| | |
|---|---|
| **The demo runs on our Neousys.** | The host software is installed and validated: JetPack 6.0, L4T R36.3.0, TensorRT 10.3.0.30, which is the exact environment every reported number was measured in. This answers all of section 2 and item 4.2. |
| **You supply the CAN package.** | The signals reach us decoded on a topic, so no DBC file is needed and item 1.1 is closed. With the CAN-to-USB interface, we read the arrangement as your adapter plus your package publishing a topic our node subscribes to. That is the shape `live_odom_node.py` was already written for. |
| **You bring the camera.** | Which creates a new question we did not have before: it now has to connect to **our** carrier rather than yours. See 5.1. |
| **We can replay your bags end to end now.** | We rebuilt the `ads_messages` definitions from the schemas embedded in the bag files themselves, so `/CAN` and `/vehicle_config` decode offline with nothing from you. That answered 1.3, 1.5, 3.4 and 3.7 out of your own recorded data instead of by asking. It does not replace your package; see 1.1. |
| **Replaying them caught a fault that was heading for the vehicle.** | Our CAN subscription was RELIABLE; your bags publish `/CAN` BEST_EFFORT, and that pairing delivers nothing and reports no error. Fixed, and now compatible with either. Detail in 1.2. We mention it because it is the argument for replaying your recordings rather than our own transcodes of them: the bag reproduced your publisher's settings, and a transcoded clip never would have. |

---

## 1. CAN odometry

The configuration in our report, 26s segmentation plus the BEV grid planner, needs ego motion to register one camera frame against the next.
In the lab that motion was replayed from a CSV extracted from the Oct 2026 bags, one row per frame.

`perception_suite/live_odom_node.py` closes the gap.
It subscribes to `/CAN` and takes exactly what `extract_can.py` takes out of the bag, `msg.spd_mps` and `msg.imu.imu.angular_velocity.z`, and it keeps the published contract of the replay node exactly.

What remains is a handful of conventions.
1.3 and 1.5 are now answered from the bags, which leaves **1.4 as the one that still fails silently if it is wrong**, so it is the first worth a reply.

We have also rebuilt the `ads_messages` definitions ourselves, from the schema text embedded in the bag files, purely so our own offline testing is not blocked.
It is replay-only and deliberately kept out of the pipeline workspace.
We are not proposing it for the vehicle: ROS 2 Humble matches publisher to subscriber on type *name* alone, with no structural check, so if your package has drifted from the version that recorded these bags our copy would decode your live topic into silent garbage.
Yours is the one that should be installed on anything connected to the car.

| # | What we need | Why it matters |
|---|---|---|
| 1.1 | ~~Can you send us the `ads_messages` package?~~ **ANSWERED.** You are providing the CAN package. | For the record: we also confirmed we can reconstruct the message definition ourselves if ever needed. The full `.msg` text, including every nested type, is embedded in the bag files, which is how `extract_can.py` decodes them with no package installed. Your package removes the version-skew risk in doing that, so we would rather have yours. |
| 1.2 | **What `ROS_DOMAIN_ID` does your CAN node publish under?** The reliability question that used to be here is **DE-RISKED** rather than answered: we now match either setting. | Two separate things, and the second one nearly cost us the demo. (a) Domain id and DDS discovery still have to match, which is cheap to confirm in advance and expensive to diagnose on the day. (b) Replaying your bags showed `/CAN` is published **BEST_EFFORT**, while the image topic on the same bag is RELIABLE, because a bag replays the QoS it recorded per topic. Our node had subscribed RELIABLE since it was written, and DDS refuses to connect a RELIABLE reader to a BEST_EFFORT writer **silently**: the subscription exists, both ends appear in `ros2 topic info -v`, and not one message is delivered. If your live publisher is BEST_EFFORT too, which is the usual choice at 30 Hz, we would have seen "nothing on /CAN" on the vehicle and gone looking at the package, the domain id and the container networking, none of which would have been the cause. The subscription is now BEST_EFFORT, which matches either kind of publisher, so this can no longer break us whatever you answer. |
| 1.3 | ~~Are `spd_mps` and `imu.imu.angular_velocity.z` in SI units, m/s and rad/s?~~ **ANSWERED from the bags.** Decoded live from bag 1: `spd_mps: 6.95` and `imu.imu.angular_velocity.z: 0.0026`. Those are m/s and rad/s. | This is one sampled message rather than a sweep, so it is strong evidence and not proof: a yaw rate left in deg/s would have read 57x larger, and 6.95 km/h is implausible for that clip. We are treating it as settled and would only want to hear from you if it is wrong, because the failure mode is silent and presents as a perception bug rather than a units bug. |
| 1.4 | **Sign convention for yaw rate**: is positive a left turn (counterclockwise from above, REP-103)? | A sign error puts the grid's accumulated road evidence on the wrong side of the road. That is worse than having no odometry at all, because it is confidently wrong. |
| 1.5 | ~~What do the `error`, `spd_error` and `imu.valid` flags mean, and are any set during normal operation?~~ **ANSWERED in part.** On a healthy sample from bag 1: `error: false`, `spd_error: false`, `cc_button_error: false`, `imu.valid: true`. | That closes the risk we actually cared about, which was our node going permanently silent on a healthy vehicle because a flag is always set. What the flags *mean* when they do fire is still open, and we still do not gate on `imu.yaw_rate_status` because its values are not described anywhere in the bag's schema. The `trust_flags:=false` escape hatch stays in place. |
| 1.6 | In **reverse**, does speed go negative or stay positive with direction in a separate gear signal? | The bags carry a `gear` field we never used, because the clips were all forward. Dead reckoning integrates speed directly, so an unsigned speed in reverse drives the pose the wrong way. |
| 1.7 | Should we use a **single vehicle speed** or the **four wheel speeds**? | The bags carried both (`spd_mps` plus `spd_fl/fr/rl/rr_mps`). We used the single value. If your production signal set differs, or you would rather we average the rears, tell us. |
| 1.8 | **What should the vehicle do if the CAN topic goes silent?** We currently have no defined behaviour, and we need yours. | `live_odom_node` publishes nothing once the last CAN message is older than `stale_timeout_s` (0.5 s), so it never emits a stale pose. **Correction, 3 Nov 2026: an earlier version of this row said that made the grid fall back to forgetting, and called it the conservative failure. That was wrong.** We traced the path. `5_planner_node` latches: `on_odom` stores the pose with `valid=True`, and nothing ever expires it. The file is 112 lines and contains no reference to time at all, no timer and no timeout. So when our node goes silent the planner keeps planning from the last pose it received, indefinitely. The vehicle is still moving, so every new mask gets stamped into the grid at a position the vehicle has already left, and observations from different real positions pile into the same cells. That corrupts the map rather than emptying it, which is the worse of the two failures and the opposite of what we claimed. Our node going quiet is necessary but not sufficient; something downstream has to notice. A real vehicle probably wants the planner to hold or hand back control, and that decision sits above this node. Ties to 4.4. |
| 1.9 | **Are `header.stamp` values on your live `/CAN` messages populated and clock-synced to the camera?** | Right now we do not use them, and that is a known weakness rather than a preference. `live_odom_node` integrates dead reckoning on wall-clock arrival time, and the BEV grid decays on wall-clock as well, so anything that changes message arrival spacing without changing the values on the messages scales the ego motion. Replaying a bag below real time does exactly that: at `bag_rate:=0.5` the integrated path is 2x too long and the grid is stretched by the same factor, with nothing warning you. On a live vehicle the arrival clock and the real clock agree, so this does not bite in deployment, which is why the default is now 1.0 and we have left the arithmetic alone. If your stamps are trustworthy we would rather integrate on them, which removes the whole class of problem. If they are not, say so and we will keep the arrival clock. |

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

Decoding the bags closed 3.4 and 3.7 outright, and opened three new questions, so this section grew rather than shrank.
3.1 and 3.2 are **the only guessed values left anywhere in the configuration**.
3.3, 3.8 and 3.9 all came out of reading your recorded data.
**3.8 is the one we would most like answered**, because the answer decides whether a video decoder sits on the vehicle path or only in our test rig.

| # | What we need | Why it matters |
|---|---|---|
| 3.1 | **Camera height above ground, in metres?** | Config says `height_m: 1.5`, flagged in the file as a guess. The planner's image-to-ground mapping is the only thing that reads it, and it scales every distance the BEV grid produces. Useful detail: your own schema already has the slot for this. `ExtraCameraInfo.t_vc`, the camera-to-vehicle translation, is all zeros in every bag we have, so this may be a field to populate rather than a measurement to take. |
| 3.2 | **Camera pitch, in degrees?** | Config says `pitch_deg: 0.0`, also flagged as a guess. The other half of the ground projection. Same situation as 3.1: `ExtraCameraInfo.r_vc` is the slot for it, and it is all zeros in the bags we have. |
| 3.3 | **Which camera recorded the bags, and is it the one you are bringing?** Your `/vehicle_config` names two. The top-level `camera_name` is `mx56h4573-ac-isx031`, while the nested intrinsics block is `mx56h4573-ac-imx490-h120`, and that nested block is where `2880x1860` and `fx 1714.20536` live. | So either the vehicle carries two cameras and our geometry came from the second one, or one of those labels is stale. Everything we have calibrated assumes the `imx490-h120`. If the demo uses the other unit, the whole geometry block needs redoing. The `h120` is also consistent with the 120 degree lens, which is what makes 3.4 matter. |
| 3.4 | ~~Will your camera node publish rectified frames, matching the bags' CameraInfo?~~ **ANSWERED for the recorded data**, still open for the live feed. All eight `distortion_coefficients` are `0.0` and `rectification_matrix` is identity, under `distortion_model: rational_polynomial`. | So the bags are rectified and our effective 80.1 degrees is correct for them. The live half stands: the physical lens is a 120 degree fisheye, so if your live node publishes raw fisheye instead, the image looks fine on the dashboard and the ground projection is quietly wrong. This is the one in this section most likely to be missed on the day. |
| 3.5 | ~~What capture interface and device index is the camera on your carrier?~~ **Replaced by 5.1**, because the camera now connects to ours. | |
| 3.6 | Does the **hood sit at the same height in frame** as in the bags? | We mask everything below 0.67 of frame height as bodywork. A flat horizontal cut, not a traced silhouette, tuned over 1800 frames of bags 1 and 3. Too high and we throw away the nearest road the camera can see, which is what anchors the centreline walk. Too low and the planner reads the bonnet as terrain. |
| 3.7 | ~~Confirm wheelbase 2815 mm, max road wheel angle 32 deg, camera-to-axle 1070 mm.~~ **ANSWERED from the bags.** `/vehicle_config` reports `wheelbase: 2815`, `max_rwa_deg: 32.0` and `x_cam2axle: 1070`, as `uint32` millimetres. | One thing the field name does not settle: `x_cam2axle` does not say *which* axle. We have been reading it as the front. If it is the rear, the Stanley geometry is wrong by a wheelbase, which is 2.8 m. That is the only part of this row still worth a reply. |
| 3.8 | **Does your live camera node publish H.264 on `/image_processing/image_compressed`, the same as the bags do? If so, is it IPPP or all-keyframe?** | Your bags carry `format: h264`, not JPEG. The structure is IPPP: most messages are a single non-IDR slice, with roughly one `SPS+PPS+IDR` keyframe every eight frames. If the live feed is the same, then H.264 decoding sits on the **vehicle path** rather than only in our test rig, and a P-frame stream is fragile in a way a still-image stream is not. One lost frame corrupts every frame after it until the next keyframe, and it corrupts *visually* rather than raising an error, so it looks like a perception fault. If your live feed is raw or MJPEG instead, our input node is far simpler and none of this applies. This supersedes the live half of 3.4. |
| 3.9 | **How do you carry the image topic today, and under what QoS?** | Measured from your bags: about 678 KB per message at 30 Hz, so roughly 20 MB/s or 163 Mbps. We found we have to subscribe RELIABLE with a deep queue or we silently drop frames, and per 3.8 a dropped frame on this stream means corruption rather than a gap. We would rather match whatever you already run than find the limit on demo day. Same question as 1.2 asks for `/CAN`. |

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
