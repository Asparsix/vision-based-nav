# ugv_sim

Compact **enclosed courtyard** world for RTAB (+ stereo UGV).

## World features
- ~26×26 m grass field inside **brick perimeter walls** (closed space)
- Short E–W + N–S roads (textured asphalt)
- Houses, buildings, trees
- Tables, chairs, benches, barrels
- Checkerboard landmark panels (strong visual features)
- **Shadows off**

## Run
```bash
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_sim sim.launch.py
```

Default spawn near `point_A` (`x:=-7`).
