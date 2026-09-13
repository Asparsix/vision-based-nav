#!/usr/bin/env bash
# Source ROS 2 Jazzy + local RTAB-Map .opt overlay + ugv_vision_nav_ws install.
# Usage: source ~/ugv_vision_nav_ws/scripts/setup_env.sh

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_WS_ROOT="$(cd "$_SCRIPT_DIR/.." && pwd)"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

_LOCAL_PREFIX="$_WS_ROOT/.opt/opt/ros/jazzy"

_apply_local_overlay() {
  if [[ ! -d "$_LOCAL_PREFIX/share/ament_index" ]]; then
    echo "WARNING: RTAB .opt missing at $_LOCAL_PREFIX" >&2
    return 0
  fi
  export AMENT_PREFIX_PATH="$_LOCAL_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export CMAKE_PREFIX_PATH="$_LOCAL_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$_LOCAL_PREFIX/lib:$_LOCAL_PREFIX/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  if [[ -d "$_LOCAL_PREFIX/lib/python3.12/site-packages" ]]; then
    export PYTHONPATH="$_LOCAL_PREFIX/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
  fi
  export PATH="$_LOCAL_PREFIX/bin${PATH:+:$PATH}"
}

_apply_local_overlay

if [[ -f "$_WS_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$_WS_ROOT/install/setup.bash"
  _apply_local_overlay
fi

# Torch for UNet segmenter
_VENV_SITE="$_WS_ROOT/.venv/lib/python3.12/site-packages"
if [[ -d "$_VENV_SITE" ]]; then
  export PYTHONPATH="$_VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi

echo "Environment ready (ws=$_WS_ROOT)"
echo "  rtabmap_slam: $(ros2 pkg prefix rtabmap_slam 2>/dev/null || echo MISSING)"
