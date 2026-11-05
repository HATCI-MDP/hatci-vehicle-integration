# Quickstart: ROS bag and live camera

This runs the pipeline on its own, from a recorded ROS bag or from a live camera, before any CAN integration.
Everything here is covered in more depth in `README.md`.

## 1. Load the image (once)

You need the `hatci-vehicle-draft7.tar.gz` file and a clone of https://github.com/HATCI-MDP/hatci-vehicle-integration, which has `run_vehicle.sh`, the script that starts the image.

Check the file arrived intact:

```
sha256sum hatci-vehicle-draft7.tar.gz
```

It should print `060230cc1efec2fe439841dbcc2657b0ee4d4572eeaa621e1abb3af726e0bc18`.

Load it into Docker.
This needs about 34 GB of free disk on top of the file itself.

```
gunzip -c hatci-vehicle-draft7.tar.gz | docker load
```

## 2. Check it works on this board

From this repository's folder:

```
./run_vehicle.sh selftest
```

This plays a clip and a CAN log that are recorded inside the image, so it needs no camera, no bag and no CAN bus.
It takes about a minute and ends with `=== selftest PASSED ===`.
If it fails, it prints the last 40 lines of the log.

The board needs JetPack with the NVIDIA container runtime, which JetPack installs.
The image was built and tested on JetPack 6.0 (L4T R36.3).
On any other version, this selftest is the check that it works.

## 3. Run it from a ROS bag

```
./run_vehicle.sh replay /path/to/bag nocan
```

`/path/to/bag` is the bag's directory, the one that holds its `metadata.yaml`.

`nocan` plays the images only, which is the standalone check.
Leave it off to also replay the bag's recorded `/CAN`.

The bag needs the same topics as the October 2026 recordings: H.264 frames on `/image_processing/image_compressed`, plus `/CAN` when `nocan` is left off.
A bag recorded with different topic names will not play as it is.

## 4. Run it from a live camera

```
./run_vehicle.sh camera
```

The defaults match our test camera, so expect to change some of these by adding them to the end of the command:

| Add this | Our default | Change it when |
|---|---|---|
| `camera_index:=N` | `0` | your camera is `/dev/videoN` with a different N |
| `rotate_180:=false` | `true` | your camera is mounted the right way up; the road must be at the bottom of the picture |
| `capture_width:=W capture_height:=H` | `1920` and `1080` | your camera captures at a different resolution |
| `pixel_format:=F` | `NV16` | your camera does not offer NV16 |

For example:

```
./run_vehicle.sh camera camera_index:=1 rotate_180:=false
```

On our board the camera has to be connected before the board is powered on, or every frame comes through black.

## What you should see

Each stage prints a line every 5 seconds, for example `stage timing | segmentation 30.0 Hz | ...`.
Rates near 30 Hz mean the pipeline is keeping up with the input.

If the board has a desktop session running, the dashboard usually opens on the board's own monitor, even when you start it over ssh.
Without one, it runs without the dashboard and only the terminal lines appear.

Stop it with Ctrl+C.
