# Open integration items

Everything below is something we need confirmed or supplied before the stack can run on the vehicle.
It is ordered by what it blocks.

**Updated 2026-11-04.** Several items have been answered since this was first written: by your hardware note, by your note on the CAN package, and by decoding your October bags directly.
The deployment target has since changed from our Neousys to your compute platform, which reopens section 2 and rewrites section 5.
Both are marked in place.
Those are marked **ANSWERED** in place rather than deleted, so the record of what was decided stays readable.
Section 2 is answered in full.
Four more were answered out of the bags themselves rather than by asking you, which is noted on each.

Read it alongside `HANDOFF.md`, which describes the image, and `src/README.md`, which describes the source.

Where a row says a value is a guess, that is us flagging it rather than you discovering it.

---

## What changed

| | |
|---|---|
| **Deployment moves to your compute platform.** | Your host is JetPack 6.1, L4T R36.4.0, Ubuntu 22.04, CUDA 12.6.68. Ours is JetPack 6.0, L4T R36.3.0, which is the environment every reported number was measured in. This reopens section 2 and replaces section 5. The practical consequences are 2.1 and 5.5 through 5.7. |
| **Our Neousys becomes the validation box, not the demo box.** | Everything we report is still measured on it, on an Orin NX. Your AGX Orin is the faster part, so our frame rates are a floor rather than a ceiling. |
| **You supply the CAN package.** | The signals reach us decoded on a topic, so no DBC file is needed and item 1.1 is closed. With the CAN-to-USB interface, we read the arrangement as your adapter plus your package publishing a topic our node subscribes to. That is the shape `live_odom_node.py` was already written for. |
| **Your camera stays on your carrier.** | The earlier plan had it connecting to ours. What we still need is its capture format and whether it is mounted the right way up. See 5.1. |
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
The entrypoint will still fall back to it on a live bus if no other `ads_messages` is visible inside the container, and says so in the log, which is what 5.6 is there to avoid.
We are not proposing it for the vehicle: ROS 2 Humble matches publisher to subscriber on type *name* alone, with no structural check, so if your package has drifted from the version that recorded these bags our copy would decode your live topic into silent garbage.
Yours is the one that should be installed on anything connected to the car.

| # | What we need | Why it matters |
|---|---|---|
| 1.1 | ~~Can you send us the `ads_messages` package?~~ **ANSWERED.** You are providing the CAN package. | For the record: we also confirmed we can reconstruct the message definition ourselves if ever needed. The full `.msg` text, including every nested type, is embedded in the bag files, which is how `extract_can.py` decodes them with no package installed. Your package removes the version-skew risk in doing that, so we would rather have yours. |
| 1.2 | **What `ROS_DOMAIN_ID` does your CAN node publish under?** The reliability question that used to be here is **DE-RISKED** rather than answered: we now match either setting. | Two separate things, and the second one nearly cost us the demo. (a) Domain id and DDS discovery still have to match, which is cheap to confirm in advance and expensive to diagnose on the day. (b) Replaying your bags showed `/CAN` is published **BEST_EFFORT**, while the image topic on the same bag is RELIABLE, because a bag replays the QoS it recorded per topic. Our node had subscribed RELIABLE since it was written, and DDS refuses to connect a RELIABLE reader to a BEST_EFFORT writer **silently**: the subscription exists, both ends appear in `ros2 topic info -v`, and not one message is delivered. If your live publisher is BEST_EFFORT too, which is the usual choice at 30 Hz, we would have seen "nothing on /CAN" on the vehicle and gone looking at the package, the domain id and the container networking, none of which would have been the cause. The subscription is now BEST_EFFORT, which matches either kind of publisher, so this can no longer break us whatever you answer. |
| 1.3 | ~~Are `spd_mps` and `imu.imu.angular_velocity.z` in SI units, m/s and rad/s?~~ **ANSWERED from the bags.** Decoded live from bag 1: `spd_mps: 6.95` and `imu.imu.angular_velocity.z: 0.0026`. Those are m/s and rad/s. | This is one sampled message rather than a sweep, so it is strong evidence and not proof: a yaw rate left in deg/s would have read 57x larger, and 6.95 km/h is implausible for that clip. We are treating it as settled and would only want to hear from you if it is wrong, because the failure mode is silent and presents as a perception bug rather than a units bug. |
| 1.4 | **Sign convention for yaw rate**: is positive a left turn (counterclockwise from above, REP-103)? | A sign error puts the grid's accumulated road evidence on the wrong side of the road. That is worse than having no odometry at all, because it is confidently wrong. |
| 1.5 | ~~What do the `error`, `spd_error` and `imu.valid` flags mean, and are any set during normal operation?~~ **ANSWERED in part.** Across all 21,847 `/CAN` messages in your three October bags, `error` and `spd_error` are never set and `imu.valid` is never false. | That closes the risk we actually cared about, which was our node going permanently silent on a healthy vehicle because a flag is always set. What the flags *mean* when they do fire is still open, and we still do not gate on `imu.yaw_rate_status` because its values are not described anywhere in the bag's schema. The `trust_flags:=false` escape hatch stays in place. |
| 1.6 | In **reverse**, does speed go negative or stay positive with direction in a separate gear signal? | The bags carry a `gear` field we never used. Every `/CAN` message in the three bags has `gear` 5 and a positive speed, lowest 0.41 m/s, so the recordings do not answer this. Dead reckoning integrates speed directly, so an unsigned speed in reverse drives the pose the wrong way. |
| 1.7 | Should we use a **single vehicle speed** or the **four wheel speeds**? | The bags carried both (`spd_mps` plus `spd_fl/fr/rl/rr_mps`). We used the single value. If your production signal set differs, or you would rather we average the rears, tell us. |
| 1.8 | **What should the vehicle do if the CAN topic goes silent?** We currently have no defined behaviour, and we need yours. | `live_odom_node` publishes nothing once the last CAN message is older than `stale_timeout_s` (0.5 s), so it never emits a stale pose. **Correction, 3 Nov 2026: an earlier version of this row said that made the grid fall back to forgetting, and called it the conservative failure. That was wrong.** We traced the path. `5_planner_node` latched: `on_odom` stored the pose with `valid=True`, and nothing ever expired it. So when our node went silent the planner kept planning from the last pose it received, indefinitely. The vehicle was still moving, so every new mask was stamped into the grid at a position the vehicle had already left, and observations from different real positions piled into the same cells. That corrupts the map rather than emptying it, which is the worse of the two failures and the opposite of what we claimed. Our node going quiet is necessary but not sufficient; something downstream has to notice. **Update, 4 Nov 2026: all three consumers of odometry now expire it on the same `odom_timeout_s`, default 0.5 s, and we have triggered it on the board by killing the odometry node mid-run.** `7_controller_node` commands a full brake rather than coasting, with the reason on the message. The dashboard drops its readouts to zero. And `5_planner_node` drops the pose, so `ground_pose()` returns None and the grid forgets rather than registering new masks against a position the vehicle has already left: the corruption described above no longer happens, though the grid clearing itself rests on the code and unit tests, since path reach did not change during the outage. Expiry is not armed until the first odometry message ever arrives, because a fixed `speed_mps` with no CAN source is a supported configuration rather than a fault. **What remains is yours and is the reason this row is still open:** none of that is a policy for the vehicle. Forgetting the grid and braking are what our nodes do to avoid acting on bad data; what the car should do, and whether it hands back to the safety driver, sits above these nodes. Ties to 4.4. |
| 1.9 | **Are `header.stamp` values on your live `/CAN` messages populated and clock-synced to the camera?** | Right now we do not use them, and that is a known weakness rather than a preference. `live_odom_node` integrates dead reckoning on wall-clock arrival time, and the BEV grid decays on wall-clock as well, so anything that changes message arrival spacing without changing the values on the messages scales the ego motion. Replaying a bag below real time does exactly that: at `bag_rate:=0.5` the integrated path is 2x too long and the grid is stretched by the same factor, with nothing warning you. On a live vehicle the arrival clock and the real clock agree, so this does not bite in deployment, which is why the default is now 1.0 and we have left the arithmetic alone. If your stamps are trustworthy we would rather integrate on them, which removes the whole class of problem. If they are not, say so and we will keep the arrival clock. |

## 2. Container policy: REOPENED

This section was closed while the demo was going to run on our Neousys.
Deployment has moved to your compute platform, so it is open again, exactly as the previous revision said it would be.

The good news first, because it narrows the question a long way.
We checked what our image actually carries by running it with the nvidia container runtime stripped off, so that only what is genuinely baked in remains:

| Component | Where it lives | Version |
|---|---|---|
| CUDA runtime | inside the image | 12.6.68 |
| TensorRT | inside the image | 10.3.0.30 |
| `libcuda.so` (the driver) | mounted from the host, absent without the runtime | the host's |

Our image already carries CUDA 12.6.68, which is the version your host runs.
So the CUDA runtime and TensorRT travel with the image, and `libcuda` comes from your host, which is where a driver should come from.
We have not listed everything else the nvidia runtime mounts from a host, and that is part of what 5.5 tests.

What we cannot do is test it.
Our board is pinned to JetPack 6.0 and cannot be moved, so we have no R36.4.0 host to try this on.
NVIDIA's own guidance is to match container and host L4T versions, and we are not going to claim it works when we have not seen it work.
One run of the image on your board settles it; that is 5.5.

| # | Was | Now |
|---|---|---|
| 2.1 | What JetPack / L4T version is the vehicle Jetson on? | **Yours.** JetPack 6.1, L4T R36.4.0, CUDA 12.6.68. Ours is JetPack 6.0 / R36.3.0. See 5.5 and 5.7. |
| 2.2 | Is `--runtime nvidia` present on the host? | **Needs confirming on your board.** JetPack 6.1 ships it, but we have not seen yours. |
| 2.3 | Can we run a container with `--privileged`? | **Reopened.** Needed for the camera path. We have confirmed it works on ours. |
| 2.4 | Are `--network host` and `--ipc=host` allowed? | **Reopened, and now load-bearing.** Both were used for every reported number, and `--network host` plus `--ipc=host` is also how our container would see a `/CAN` topic published by your package outside it. See 5.6. |
| 2.5 | Docker version and free disk on the vehicle board? | **Reopened.** Loading needs roughly 34 GB free on top of the 12.5 GB archive. |
| 2.6 | How should we physically deliver the tarball? | **Reopened, and now needed.** The image no longer stays on our board. 12.5 GB compressed; `HANDOFF.md` has the checksum. |

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
| 3.5 | ~~What capture interface and device index is the camera on your carrier?~~ **See 5.1.** | |
| 3.6 | Does the **hood sit at the same height in frame** as in the bags? | We mask everything below 0.67 of frame height as bodywork. A flat horizontal cut, not a traced silhouette, tuned over 1800 frames of bags 1 and 3. Too high and we throw away the nearest road the camera can see, which is what anchors the centreline walk. Too low and the planner reads the bonnet as terrain. |
| 3.7 | ~~Confirm wheelbase 2815 mm, max road wheel angle 32 deg, camera-to-axle 1070 mm.~~ **ANSWERED from the bags.** `/vehicle_config` reports `wheelbase: 2815`, `max_rwa_deg: 32.0` and `x_cam2axle: 1070`, as `uint32` millimetres. | One thing the field name does not settle: `x_cam2axle` does not say *which* axle. Our config reads it as the rear, placing the camera 1.07 m ahead of the rear axle, and that sets where the planner's arcs and the MPC reference begin. If it means the front axle, those are wrong. That is the only part of this row still worth a reply. |
| 3.8 | **Does your live camera node publish H.264 on `/image_processing/image_compressed`, the same as the bags do? If so, is it IPPP or all-keyframe?** | Your bags carry `format: h264`, not JPEG. The structure is IPPP: in all three bags, exactly every fifth message carries `SPS+PPS+IDR`, and the other four are a single non-IDR slice. If the live feed is the same, then H.264 decoding sits on the **vehicle path** rather than only in our test rig, and a P-frame stream is fragile in a way a still-image stream is not. One lost frame corrupts every frame after it until the next keyframe, and it corrupts *visually* rather than raising an error, so it looks like a perception fault. If your live feed is raw or MJPEG instead, our input node is far simpler and none of this applies. This supersedes the live half of 3.4. |
| 3.9 | **How do you carry the image topic today, and under what QoS?** | Measured across your three bags: 1,045 to 1,086 KB per message at 30 Hz, so 32 to 33 MB/s or 257 to 267 Mbps. We found we have to subscribe RELIABLE with a deep queue or we silently drop frames, and per 3.8 a dropped frame on this stream means corruption rather than a gap. We would rather match whatever you already run than find the limit on demo day. Same question as 1.2 asks for `/CAN`. |

## 4. Integration and ownership

| # | What we need | Why it matters |
|---|---|---|
| 4.1 | **What consumes the steering output?** | Half of this is now closed on our side. `7_controller_node` publishes `/command` at 30 Hz as a typed `perception_suite_msgs/ControlCommand` with a header, steering, throttle, brake, parking brake and a reason string, which replaces the headerless `Float32MultiArray` this item used to describe. The node is opt-in, off unless you pass `control:=true`, so it stays out of the camera-integration milestone exactly as you asked. What we still need is whether you have an existing actuation interface we should publish into instead, and what subscribes on your side. |
| 4.2 | **Can clocks be pinned on your board?** This was answered while the demo was on ours, and it is open again. | Pinned is what our reported latency numbers were measured at. In the R1 matrix, unpinned median latency was 25 to 42 percent higher, and every run still passed the frame-rate requirement. |
| 4.3 | Confirm **26s is the model to deploy**. | 26s plus the BEV grid is what we benchmarked and what this image defaults to. The two VOS-trained variants are in the image but should not be deployed until R2 scores their segmentation accuracy: they reject their own perception 5 to 11 times more often than the originals. |
| 4.4 | What is the **handback and e-stop path**, and what are the **first-run conditions**? | Ties back to 1.8. Before this drives anything we should agree on what the safety driver sees and does, and whether first runs are on a closed course. |

## 5. From moving deployment to your platform

5.1 through 5.4 were written while the demo was going to run on our Neousys with your camera attached to it.
That is no longer the plan, so they are superseded rather than deleted: 5.1 and 5.2 move to your side, and 5.3 stops applying.
5.4 is kept because it is a property of our carrier and still bites anyone testing on it.

| # | What we need | Why it matters |
|---|---|---|
| 5.5 | **Run the image once on your board.** | This is the cheapest thing on the list and it retires the largest unknown. No camera, no CAN, no DBC: `./run_vehicle.sh selftest` plays a recorded clip and exits pass or fail. If it passes, section 2's version question is closed. If it fails, it prints the last 40 log lines, which is where we would start. We cannot run this ourselves because we have no JetPack 6.1 host. |
| 5.6 | **The `ads_messages` message definitions.** | Not the DBC and not your CAN parsing code. Our nodes subscribe to `/CAN` as ROS messages and never touch raw bus frames, so no DBC is needed; that has not changed. What has changed is that our code now runs inside a container on your host, and deserializing your topic needs the generated type support compiled into that container. The `.msg` files are enough. We rebuilt a stand-in from the schema text embedded in your October bags, and it may already be correct, but a type-name match with a field mismatch decodes to silent garbage rather than an error, so we would rather check than assume. |
| 5.7 | **Regenerate the TensorRT engines on your hardware.** | TensorRT warns against using an engine across device models, saying it can affect performance or cause errors. Our segmentation node logs that warning on every run on our own board, so an engine from another device does load, but we have not established which device built ours. The `.onnx` sources ship inside the image next to the engines we use, and `trtexec` is in there too, so regenerating does not need any retraining. |
| 5.1 | ~~What interface and connector does your camera use?~~ **Superseded.** | Your camera now connects to your own carrier, so this is yours to answer rather than ours. What we would still like is the capture format and resolution, because `camera` mode's defaults are tuned to our GMSL carrier (`camera_index:=0`, `pixel_format:=NV16`, 1920x1080, `rotate_180:=true`). All of them can be overridden by appending launch arguments. The rotation matters most: ours is on only because our camera is mounted inverted. The node itself uses plain `v4l2src` with no NVIDIA-specific GStreamer elements, so it is not tied to our hardware. |
| 5.4 | **Procedure note, still true on our carrier: the camera must be connected before the board is powered on.** | This hardware does not hot-plug. The deserializer link is trained when the driver probes at boot, so a camera connected to a running board is never seen. The failure is silent and convincing: the device opens, streaming starts, buffers arrive at the right size and cadence, and every byte in them is zero, with no error logged anywhere. A reboot with the camera already attached fixes it. Whether your carrier behaves the same way we do not know. |
| 5.2 | ~~When do you want to install the CAN dependencies on our Neousys?~~ **Superseded.** | They go on your platform now, which is yours to schedule. |
| 5.3 | ~~When does our Neousys need to be in the vehicle, and for how long?~~ **No longer applies.** | It stays on our bench as the validation box. |
