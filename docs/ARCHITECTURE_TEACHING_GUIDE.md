# UGV Vision Navigation — Architecture Overlay (Teaching Guide)

This document explains the **full system** we built for a GPS-denied outdoor UGV: what each block does, what the words mean, and how data flows. Use it when explaining the flowchart to students.

---

## 1. Problem in one sentence

**Drive a ground robot from A → B outdoors without GPS, using cameras (and derived depth), understanding the scene, building/using a map, and planning a safe path.**

Constraints we designed for:
- No GPS (denied / indoor-ish outdoor courtyard)
- Cameras as primary sensing (no lidar required in the core stack)
- Simulation first (Gazebo), same architecture later on a real robot

---

## 2. Big-picture architecture (flowchart)

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                         GAZEBO / ROBOT SENSORS                          │
│  Left RGB camera │ Right RGB camera │ Depth camera │ IMU │ Wheel odom   │
└────────────┬───────────────┬──────────────┬───────────┬─────────────────┘
             │               │              │           │
             ▼               ▼              ▼           ▼
      ┌────────────┐   (stereo pair)   ┌────────┐  ┌────────────┐
      │  Seg UNet  │                   │ Depth  │  │ Wheel+IMU  │
      │ (perception)│                   │ image  │  │ + Visual   │
      └──────┬─────┘                   └───┬────┘  │ Odometry   │
             │                             │       │ → EKF      │
             ▼                             ▼       └──────┬─────┘
      ┌────────────┐               ┌────────────┐         │
      │ Semantic / │               │ Point cloud│         │
      │ cost hints │               │ (XYZ)      │         │
      └──────┬─────┘               └─────┬──────┘         │
             │                           │                │
             │         ┌─────────────────┴────────────────┘
             │         │  RGB (left) + Depth + Filtered odom (+ IMU)
             │         ▼
             │   ┌──────────────┐
             │   │   RTAB-Map   │  ← SLAM: localize + build /map
             │   │  (VSLAM)     │     publishes map → odom TF
             │   └──────┬───────┘
             │          │ /map  +  pose (TF)
             ▼          ▼
      ┌─────────────────────────────┐
      │     Nav2 Costmaps           │
      │  Global + Local layers      │
      │  (map + semantic obstacles) │
      └─────────────┬───────────────┘
                    ▼
      ┌─────────────────────────────┐
      │  Global planner (A*/NavFn)  │  → long path A→B on map
      │  Local planner / Controller │  → short-term tracking + avoid
      └─────────────┬───────────────┘
                    ▼
                 /cmd_vel  →  robot motors
```

**One-line story for students:**  
*Cameras see the world → AI labels what’s drivable → SLAM figures out where we are on a map → Nav2 plans and steers.*

---

## 3. Glossary (meanings of the blocks)

### 3.1 RGB image
Normal color camera picture (what humans see).  
We use mainly the **left** camera for:
- Segmentation (Seg UNet)
- RTAB-Map visual features

### 3.2 Stereo (left + right)
Two cameras separated by a small baseline.  
In theory you can compute depth by matching left↔right.  
**In our stack:** both cameras exist, but depth for SLAM comes from a **Gazebo depth camera** (cleaner in sim), not from stereo matching. Stereo is still useful for realism / future real robots.

### 3.3 Depth (depth image)
A same-size image where **each pixel is a distance** (meters) from the camera, not a color.  
Example: road ahead = ~2 m, far wall = ~15 m, sky = invalid/far.

**Why we need it:**
- Turns 2D pictures into 3D understanding
- Lets SLAM and costmaps know *how far* obstacles are

### 3.4 Point cloud
Depth converted into a set of **3D points (X, Y, Z)** in space.  
If you “paint” many depth pixels into the world, you get a cloud of points (trees, walls, ground).

**In our stack:** `/stereo/points` is built from the depth camera (naming is historical; it’s depth→cloud, not stereo disparity).

### 3.5 IMU (Inertial Measurement Unit)
Sensor for **acceleration** and **angular rate** (and often orientation).  
Helps estimate rotation and motion when vision is weak (fast turns, textureless areas).

### 3.6 Wheel odometry
Estimate of motion from **wheel encoders** (how much each wheel turned).  
Good short-term, drifts over time (slip, bumps).

### 3.7 Visual odometry (VO)
Estimate of motion by watching how camera features move frame-to-frame (we use RTAB’s `rgbd_odometry`).

### 3.8 EKF (`robot_localization`)
**Extended Kalman Filter** — fuses several noisy motion sources into one smoother pose stream:
- Wheel odom
- IMU
- Visual odometry  

Output: `/odometry/filtered` + TF `odom → base_footprint`.

**Why:** better pose than any single source alone.

### 3.9 Seg UNet (segmentation model)
**UNet** is a neural network for **semantic segmentation**: every pixel gets a class label.

| Class ID | Meaning        | Nav meaning (typical)     |
|----------|----------------|---------------------------|
| 0        | background     | unknown / not road        |
| 1        | road           | free / preferred          |
| 2        | grass          | costly / soft avoid       |
| 3        | tree           | obstacle / lethal         |

**What the model does:**  
Input = RGB image → Output = mask (same size, each pixel = class).

We trained it on Gazebo images with auto-generated masks (color rules), then run it live in ROS as the segmenter node.

**“SegUNet” in the flowchart** = this perception block (UNet used for segmentation).

### 3.10 Semantic / traversability costmap
Convert the segmentation mask into **costs** for navigation:
- Road → low cost (0)
- Grass → medium cost
- Tree → high / lethal  
Optionally publish obstacle points for Nav2 layers.

This is **camera intelligence**, not the global map itself.

### 3.11 SLAM / RTAB-Map
**SLAM** = Simultaneous Localization and Mapping  
Build a map **while** figuring out where you are in it.

**RTAB-Map** (Real-Time Appearance-Based Mapping) is the SLAM package we use.

**What we feed RTAB:**
| Input              | Topic / source              | Role                          |
|--------------------|-----------------------------|-------------------------------|
| RGB                | left `/camera/image_raw`    | visual features / loop closure|
| Depth              | `/camera/depth/image_raw`   | 3D structure / obstacles      |
| Odometry           | `/odometry/filtered` (EKF)  | motion prior                  |
| IMU (available)    | `/imu`                      | orientation aid               |

**What RTAB gives us:**
- Occupancy grid `/map` (for Nav2)
- Transform `map → odom` (global pose correction)
- Database file `rtabmap.db` (saved map for later localization)

**Why RTAB (not “just odometry”)?**  
Odometry drifts. RTAB recognizes places again (**loop closure**) and corrects the map/pose so the robot doesn’t get permanently lost.

**2D vs 3D map:**  
Nav2 drives on a **2D** costmap (`Grid/3D=false`). RTAB can still show a **3D** view of the same database for screenshots/teaching. Not a loss for flat-ground driving.

### 3.12 Global planner
Looks at the **whole map** and finds a path from start to goal (e.g. NavFn / A*).  
Output: a path of waypoints (coarse plan).

### 3.13 Local planner / Controller
Looks at the **near field** (local costmap + robot dynamics) and outputs continuous `/cmd_vel` (speed + turn) to follow the global path while avoiding nearby obstacles.

Examples in Nav2: DWB, MPPI, RPP — different algorithms, same job.

### 3.14 Costmaps (global + local)
2D grids of cell costs:
- **Global costmap** — long-range, tied to `/map`
- **Local costmap** — short-range, robot-centered, updates fast  

Layers can include static map + inflation + semantic obstacles.

### 3.15 TF (transforms)
ROS’s coordinate frame tree, e.g.:
`map → odom → base_footprint → base_link → camera_optical_frame`

Everything must agree on frames or images/clouds won’t line up.

---

## 4. Modes of operation

### Mode A — Mapping (`mapping:=true`)
1. Start Gazebo  
2. Start full stack with RTAB in **mapping** mode  
3. Drive with teleop (slowly, look at walls/houses/landmarks)  
4. RTAB writes `~/ugv_vision_nav_ws/data/maps/rtabmap.db`  

### Mode B — Localization + Navigate (`mapping:=false`)
1. Start Gazebo  
2. Start full stack: RTAB **loads the same DB**, does not wipe it  
3. Nav2 on  
4. Set goal in RViz → robot plans and drives  

**Critical teaching point:**  
`mapping:=true` can delete/rebuild the DB.  
`mapping:=false` **reuses** the saved map.

---

## 5. What runs in which package (workspace map)

| Package            | Responsibility                                      |
|--------------------|-----------------------------------------------------|
| `ugv_sim`          | Gazebo world, robot SDF/URDF, bridges, sensors      |
| `ugv_data_collect` | Save camera images for training                     |
| `ugv_perception`   | UNet segmenter, semantic costmap                    |
| `ugv_slam`         | Stereo relays, EKF+VO launch, RTAB launch           |
| `ugv_nav`          | Nav2 params, full_stack launch                      |
| `scripts/`         | train UNet, auto-label, setup_env                   |

Environment helper (always source this):
```bash
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
```

---

## 6. Commands cheat-sheet (for demos)

### Mapping
```bash
# Terminal 1
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_sim sim.launch.py

# Terminal 2
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_nav full_stack.launch.py mapping:=true use_nav2:=false use_vo:=true

# Terminal 3
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Navigate on saved map
```bash
# Terminal 1 — sim (same as above)

# Terminal 2
source ~/ugv_vision_nav_ws/scripts/setup_env.sh
ros2 launch ugv_nav full_stack.launch.py mapping:=false use_nav2:=true use_vo:=true
```

Map file: `~/ugv_vision_nav_ws/data/maps/rtabmap.db`

---

## 7. How to explain it in 60 seconds (elevator pitch)

> We put cameras on a UGV in Gazebo. A **UNet** looks at the RGB image and labels road, grass, and trees. A **depth camera** tells us how far things are; that becomes a **point cloud**. **Wheel odometry, IMU, and visual odometry** are fused in an **EKF** so motion estimates are stable. **RTAB-Map** uses RGB + depth + odometry to build a map and keep track of pose without GPS. That map and the semantic costs go into **Nav2**: a **global planner** finds A→B, and a **controller** follows it with `/cmd_vel`.

---

## 8. Common student questions (short answers)

**Q: Why not only teleop?**  
A: Teleop isn’t autonomous. The goal is sense → localize → plan → act.

**Q: Why not only segmentation driving (no map)?**  
A: Reactive only oscillates / gets stuck; no memory of the world. Map + planner enables real A→B.

**Q: Why is the open field map weak?**  
A: SLAM needs visual features. Empty grass/sky has few features. Enclosed walls, furniture, textures help.

**Q: Is depth “stereo”?**  
A: In our sim, depth is a dedicated depth sensor aligned with the left camera. Stereo cameras exist for a realistic sensor suite / future work.

**Q: Do we lose info by using a 2D Nav2 map?**  
A: For a ground robot on flat terrain, 2D occupancy is what planners need. 3D views are for visualization/teaching; same RTAB database can show 3D.

**Q: What is `/cmd_vel`?**  
A: Velocity command: linear.x = forward speed, angular.z = turn rate. Final output of the stack to the base.

---

## 9. Suggested teaching order (lab sequence)

1. Show Gazebo robot + topics (`ros2 topic list`)  
2. Show RGB + depth in RViz  
3. Show segmentation overlay (road/grass/tree)  
4. Drive & build map (RTAB)  
5. Open map / 3D view from `rtabmap.db`  
6. Restart in localization mode + set Nav2 goal  
7. Discuss failure cases: textureless areas, fast spins, wrong `mapping:=true` wiping DB  

---

## 10. One diagram to draw on the board

```text
Sense → Understand → Localize/Map → Plan → Act

RGB(+Depth) → UNet + cloud → EKF + RTAB-Map → Nav2 → cmd_vel
```

That’s the entire architecture in five words students can remember.
