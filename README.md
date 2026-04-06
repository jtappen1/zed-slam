# ZedX Roboracer Package

> **Package integrating Stereolabs ZedX with the Roboracer Platform**

```
Platform:   NVIDIA Jetson · ROS 2 Humble
Camera:     ZED X
Language:   Python 3
```
---

  
## Overview

This repo contains a lightweight and minimal implementation of SLAM, meshing, and Pure Pursuit with the Stereolabs ZEDX camera, set up to work with the Roboracer (previously F1/10th) platform.  Due to the edge compute constraints of the Roboracer platform, this package contains the minimal stack needed to allow mapping/localization with the ZEDX.  For those needing more comprehensive topics fine-grain control, the [ZEDX ROS2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) provides that support at the cost of higher compute.  This repo also details integration with other F1/10th systems needed to run a full autonomy stack on the Roboracer platform.
  
<p align="center">
  <img src="./data/img/IMG_7968.jpeg" width="40%"/>
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

### Prerequisites

```bash
# List dependencies here
# e.g.
# - ZED SDK >= X.X
# - ROS 2 [distro]
# - pyzed
```

### Installation

```bash
cd ~/zed_ws/src
git clone [repo url]
cd ~/zed_ws
colcon build --packages-select zedx_pure_pursuit
source install/setup.bash
```

---

## Usage
NOTE: See [COMMAND.md](./COMMAND.md) for the step by step list of commands to run for the full roboracer stack.

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

## Configuration

[Describe any config files, how to tune parameters, tips for different environments, etc.]

---

## Map Files

[Describe how area maps work in your setup — where they're stored, how to name them, when to remap, etc.]

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `ZED Camera failed to open` | Camera not connected or permissions | Check USB/power, run `ZED_Explorer` |
| Tracking state `LOST` immediately | Bad area file or wrong starting pose | Re-map or clear area file |
| No topics publishing | Node crashed silently | Check `ros2 node list` and logs |
| [Add your own] | | |

---

## Known Issues

- [ ] [Issue or limitation 1]
- [ ] [Issue or limitation 2]

---

## Roadmap

- [ ] [Planned feature 1]
- [ ] [Planned feature 2]

---

## License

[License name] — see [`LICENSE`](LICENSE) for details.