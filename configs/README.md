# Run configuration

`tmp_26s_demo.yaml` is the configuration behind every number we have reported: 26s segmentation under the BEV grid planner.
The file itself says `planning.mode: baseline`; the launch file switches to the grid when a CAN source is present.
The name is the one the image uses, so it matches `/bench/configs/tmp_26s_demo.yaml` inside the container and the `CONFIG` default in `docker/entrypoint.sh`.
It is a working name and we intend to rename it, but not before the image is rebuilt, because a rename here without one would leave the repo and the image disagreeing.

Read this file before reading the source.
It carries the parts that are specific to your vehicle rather than to the algorithm:

- the camera as recorded in the Oct 2026 bags, 2880x1860, `fov_h` 80.1 degrees computed from the bags' own `CameraInfo`
- the hood mask, a flat horizontal cut at 0.67 of frame height, tuned over 1800 frames
- vehicle geometry read from the bags' own `/vehicle_config`: wheelbase 2.815 m, max road wheel angle 32 degrees, camera 1.07 m ahead of the rear axle
- speeds and lateral limits derived from how the vehicle was actually driven on that route, per bag, with the measurements in the comments
- the gate thresholds, which depend on what a given model's confidence means and are not transferable between models

Two values in it are still guesses and are the only ones we need from you: camera height above ground (`height_m`) and camera pitch (`pitch_deg`).
Both are marked in the file.

The `model_weights` path points at `/bench/models` inside the image.
The weights are not in this repository; see the README at the repository root for why.
