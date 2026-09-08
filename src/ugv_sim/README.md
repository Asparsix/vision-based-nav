# ugv_sim

Differential-drive UGV + outdoor Gazebo world (road, grass, trees).

## Labels
| Model | Meaning | Look |
|-------|---------|------|
| `grass` | Grass field | Green |
| `road` | Driveable path | Dark gray |
| `tree_*` | Trees / obstacles | Brown + green |
| `point_A` / `point_B` | Start / goal | Yellow / cyan |

## Build & run
```bash
cd ~/ugv_vision_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ugv_sim
source install/setup.bash
ros2 launch ugv_sim sim.launch.py
```

Teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
