# UGV Vision Nav Workspace

From-scratch ROS 2 workspace for camera-based outdoor UGV navigation.

```bash
cd ~/ugv_vision_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ugv_sim sim.launch.py
```
