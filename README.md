# ZedX Roboracer Package

> **Stereolabs ZedX integration with the Roboracer Platform**

```
Platform:   NVIDIA Jetson · ROS 2 Humble
Camera:     ZED X
Language:   Python 3 & C++
```
---

  
## Overview

This repo contains a lightweight and minimal implementation of VSLAM, meshing, and Pure Pursuit with the Stereolabs ZEDX camera, set up to work with the Roboracer (previously F1Tenth) platform.  It contains the first steps toward a robust 3D localization solution for the F1Tenth platform.  Due to the edge compute constraints of the Roboracer platform, this package contains the minimal stack needed to allow mapping/localization with the ZEDX.  For those needing more comprehensive topics and fine-grain control, the [ZEDX ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) provides that support at the cost of higher compute.  This repo also details integration with other F1Tenth systems needed to run a full autonomy stack on the Roboracer platform. Information on setting up the ZEDX hardware can be found in the [SETUP.md](./SETUP.md) file.  ZED SDK documentation can be found [here](https://www.stereolabs.com/docs/zed-ecosystem).
  
<p align="center">
  <img src="./data/img/IMG_7968.jpeg" width="40%"/><br/>
  <em>F1Tenth car with mounted ZEDX.</em>
</p>

---

## Nodes

### `zed_slam_node`

Lightweight node that runs positional tracking on the ZED X and publishes pose + spatial memory status.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fps` | int | `30` | Camera framerate |
| `resolution` | string | `SVGA` | Camera resolution (`HD2K`, `HD1080`, `SVGA`) |
| `area_file` | string | `''` | Path to `.area` map file |
| `initial_mapping` | bool | `false` | If true, builds a new map instead of localizing on provided area file|

**Published Topics**

| Topic | Type | Description |
|---|---|---|
| `/zed/zed_node/pose` | `geometry_msgs/PoseStamped` | Camera pose in map frame |
| `/zed/spatial_memory_status` | `diagnostic_msgs/DiagnosticArray` | Spatial memory state changes |

**Broadcast Transforms**

| From | To |
|---|---|
| `map` | `zed_camera_link` |

---

### `zed_record`

Records ZED camera output to an `.svo2` file for offline meshing.

| Argument | Default | Description |
|---|---|---|
| `--fps` | `60` | Camera framerate |
| `--resolution` | `SVGA` | Camera resolution |
| `--dir` | `/home/nvidia/.../data/svo` | Output directory |
| `--output` | `test.svo2` | Output filename |

---

## Setup
### Installation

```bash
cd ~/zed_ws/src
git clone https://github.com/jtappen1/zedx_pure_pursuit.git
cd ~/zed_ws
colcon build --packages-select zedx_pure_pursuit
source install/setup.bash
```

---

## Usage
NOTE: See [COMMAND.md](./COMMAND.md) for the step by step list of commands to run the full Roboracer stack.

### Localization (existing map)

```bash
ros2 run zedx_pure_pursuit zed_slam.py --ros-args -p fps:=30 -p resolution:=SVGA -p area_file:=/home/nvidia/zed_ws/src/zedx_pure_pursuit/data/maps/test.area -p initial_mapping:=false -p update_map:=false
```

### Lifetime Mapping (adding to an existing map)

```bash
ros2 run zedx_pure_pursuit zed_slam.py --ros-args -p fps:=30 -p resolution:=SVGA -p area_file:=/home/nvidia/zed_ws/src/zedx_pure_pursuit/data/maps/test.area -p initial_mapping:=false -p update_map:=true
```


### Mapping (build new map)

```bash
ros2 run zedx_pure_pursuit zed_slam.py --ros-args -p fps:=30 -p resolution:=SVGA -p area_file:=/home/nvidia/zed_ws/src/zedx_pure_pursuit/data/maps/test.area -p initial_mapping:=true -p update_map:=true
```

### Recording .SVO2

```bash
python zed_record.py --fps 30 --resolution SVGA --dir /path/to/output
```

---

## Meshing

The ZED SDK support generating high fidelity meshes through both on and offline spatial mapping. More documentation can be found in the [ZED Spatial Mapping docs](https://www.stereolabs.com/docs/spatial-mapping), but through testing accurate meshes have been created by recording a .svo2 file of the area and using Stereolab's ZEDfu application to perform offline spatial mapping of the area.  The best meshing is done by carrying the camera at stable height and walking slowly through the area. Through the .svo2 file and ZEDfu application, positional tracking is automatically calculated and can be used to create both area files and meshes of the area.

<p align="center">
  <img src="./data/img/mesh-pumptrack.png" width="90%"/><br/>
  <em>Example of a mesh generated of an outdoor track using the above method.</em>
</p>

---

## Map (.area) Files
More information on .area files and the VSLAM mapping process can be found on the [ZED SDK docs](https://www.stereolabs.com/docs/positional-tracking/area-memory)

---

## Troubleshooting (In Progress)

| Symptom | Likely Cause | Fix |
|---|---|---|
| `ZED Camera failed to open` | Camera not connected or permissions | Check USB/power, run `ZED_Explorer` |
| Tracking state `LOST` immediately | Bad area file or wrong starting pose | Re-map or clear area file |
| No topics publishing | Node crashed silently | Check `ros2 node list` and logs |
