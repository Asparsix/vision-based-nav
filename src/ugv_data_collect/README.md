# ugv_data_collect — Step 1: collect Gazebo camera images

Saves `/camera/image_raw` frames while you drive in Gazebo. Use them later for UNet labeling.

## Build

```bash
cd ~/ugv_vision_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ugv_data_collect
source install/setup.bash
```

## How to collect

**Terminal 1 — sim**
```bash
source ~/ugv_vision_nav_ws/install/setup.bash
ros2 launch ugv_sim sim.launch.py
```

**Terminal 2 — teleop**
```bash
source ~/ugv_vision_nav_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3 — saver**
```bash
source ~/ugv_vision_nav_ws/install/setup.bash
ros2 launch ugv_data_collect collect_images.launch.py
```

Drive along the road, turn toward grass and trees so the camera sees all classes.

Images land in:
`~/ugv_vision_nav_ws/data/images_YYYYMMDD_HHMMSS/`

## Useful args

```bash
ros2 launch ugv_data_collect collect_images.launch.py \
  save_every_n:=3 \
  max_images:=500 \
  output_dir:=$HOME/ugv_vision_nav_ws/data/run1
```

| Arg | Default | Meaning |
|-----|---------|---------|
| `save_every_n` | 5 | Save 1 of every N camera frames |
| `max_images` | 400 | Stop after N saved images |
| `output_dir` | auto timestamp folder | Where PNGs are written |
| `image_topic` | `/camera/image_raw` | Camera topic |

## Next (Step 2–3)

Label these PNGs (road / grass / tree / …) with CVAT or LabelMe, then train UNet.
