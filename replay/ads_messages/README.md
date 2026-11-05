# ads_messages (reconstructed, replay only)

This package exists so we can replay HATCI's recorded bags without waiting on anything.
It is type definitions only and contains no logic.

**The definitions are HATCI's, not ours.**
They were reproduced verbatim from the schemas embedded in HATCI's own October 2026 bag files.
A ROS 2 bag carries the full text of every message definition it recorded, including nested types, which is the only reason this package can be built at all.
The five `.msg` files were generated directly from that embedded text rather than transcribed by hand.

## Why the name is not a choice

`rosbag2` resolves the type name recorded in the bag, which is `ads_messages/msg/CANMsg`.
A package under any other name cannot deserialize these bags, so this one has to be called `ads_messages`.

That creates a collision risk with HATCI's real package, which is why this lives outside `src/` and is built into its own overlay workspace rather than the main one.

## Why this must never be live on the vehicle

CDR serialization is positional, not named.
ROS 2 Humble matches a publisher to a subscriber on the type *name* only; structural type hashes did not arrive until Iron.

So if HATCI's current `ads_messages` has drifted at all from the version that recorded these bags, by one added field or one reordered field, this reconstruction would deserialize their live topic into silent garbage.
No error would be raised anywhere, and the symptom would look like a perception or planning bug rather than a decoding one.
A yaw rate read from the wrong offset is still a plausible-looking float.

For bag replay there is no such risk, because the schema came out of the bags being replayed.

**Use HATCI's package for anything touching the vehicle.**
Then delete this one, or leave it unbuilt.

The image does not enforce this on its own.
If no other `ads_messages` is visible inside the container, the entrypoint falls back to this one on a live bus and says so in the log.
A package installed only on the host is not visible inside the container.

## The five types

| Type | Where it is used |
|---|---|
| `CANMsg` | the `/CAN` topic, 30 Hz |
| `CANImu` | nested inside `CANMsg` |
| `VehicleConfig` | the `/vehicle_config` topic, 2 Hz |
| `ExtraCameraInfo` | nested inside `VehicleConfig` |
| `CANSignals` | nested inside `VehicleConfig` |

Everything else they reference is standard ROS 2: `std_msgs/Header`, `sensor_msgs/Imu`, `sensor_msgs/NavSatFix`, `geometry_msgs/Quaternion`.

## Building it

Into its own overlay, kept separate from the pipeline workspace:

```bash
mkdir -p /opt/replay_ws/src
cp -r replay/ads_messages /opt/replay_ws/src/
cd /opt/replay_ws && colcon build
```

Source it only when replaying:

```bash
source /opt/replay_ws/install/setup.bash
ros2 bag play <bag>
```

## One thing the schema settled

`wheelbase` and `x_cam2axle` are `uint32`, so those values are millimetres rather than metres.
That confirms the 2815 and 1070 figures in `configs/` are in the right units.
Note the field is named `x_cam2axle` without saying which axle, so item 3.7 in `OPEN-ITEMS.md` still needs an answer.
