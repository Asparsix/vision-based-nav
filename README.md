# UGV Vision Nav Workspace

Stereo + VIO (RTAB-Map) + semantic segmentation + Nav2 for outdoor camera-only UGV navigation.

## Architecture
```
Gazebo stereo + IMU + depth
  → left/right images, /imu, depth/cloud
  → UNet segmentation → traversability costmap / semantic obstacles
  → RTAB-Map (RGB-D) → /map + map→odom pose
  → Nav2 (global/local costmaps) → /cmd_vel
```

## Packages
- `ugv_sim` — Gazebo outdoor world + stereo UGV + IMU + depth
- `ugv_data_collect` — save camera images for UNet labeling
- `ugv_perception` — UNet segmentation + traversability costmap/obstacles
- `ugv_slam` — stereo topic bringup + RTAB-Map (+ optional slam_toolbox fallback)
- `ugv_nav` — Nav2 params/launch + full stack

## Setup
```bash
cd ~/ugv_vision_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source scripts/setup_env.sh   # ROS + RTAB .opt overlay + torch venv
```

## 1) Mapping pass (build RTAB DB + /map)
```bash
# Terminal A
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_sim sim.launch.py

# Terminal B
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_nav full_stack.launch.py mapping:=true use_nav2:=false

# Terminal C — drive the path slowly
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
DB saved to `~/ugv_vision_nav_ws/data/maps/rtabmap.db`.

Optional occupancy snapshot:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ugv_vision_nav_ws/data/maps/outdoor_path
```

## 2) Localize + Nav2 A→B (semantic costs)
```bash
# Terminal A — sim
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_sim sim.launch.py

# Terminal B — stereo + seg + RTAB localize + Nav2
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_nav full_stack.launch.py mapping:=false use_nav2:=true
```
In RViz: set Fixed Frame `map`, use **Nav2 Goal** tool to send Point B.

## Key topics
| Topic | Meaning |
|-------|---------|
| `/stereo/left/image_raw` | Left camera |
| `/stereo/right/image_raw` | Right camera |
| `/camera/image_raw` | Left alias (UNet/RTAB RGB) |
| `/camera/depth/image_raw` | Depth (VSLAM Depth block) |
| `/stereo/depth`, `/stereo/points` | Architecture depth/cloud aliases |
| `/imu` | IMU |
| `/perception/segmentation_overlay` | Colored seg view |
| `/perception/semantic_obstacles` | Tree/grass points for Nav2 |
| `/map` | RTAB occupancy map |
| `/cmd_vel` | Nav2 output |

## Notes
- Primary navigator is **Nav2** (not the old reactive `vision_local_planner`).
- Depth uses Gazebo depth camera (stable); stereo left/right still published for the architecture.
- RTAB binaries come from `~/ugv_vision_nav_ws/.opt` (symlink to prior RTAB extract). Always `source scripts/setup_env.sh`.
