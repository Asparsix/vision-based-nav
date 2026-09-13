#!/usr/bin/env bash
# Fresh start helper for UGV vision stack.
# Usage: bash scripts/run_fresh.sh
set -euo pipefail

WS="${HOME}/ugv_vision_nav_ws"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

echo "Workspace: $WS"
echo
echo "Open THREE terminals and run one command in each:"
echo
echo "  [1] SIM (Gazebo + robot + camera)"
echo "      ros2 launch ugv_sim sim.launch.py"
echo
echo "  [2] SEGMENTER (UNet on camera)"
echo "      ros2 launch ugv_perception segment.launch.py"
echo
echo "  [3] VIEW (pick topic in dropdown)"
echo "      ros2 run rqt_image_view rqt_image_view"
echo "      → /perception/segmentation_overlay"
echo
echo "Optional drive:"
echo "      ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo
echo "Topics:"
echo "  /camera/image_raw                  camera from Gazebo"
echo "  /perception/segmentation           class mask (0/1/2/3)"
echo "  /perception/segmentation_color     colored mask"
echo "  /perception/segmentation_overlay   camera + mask (look here)"
