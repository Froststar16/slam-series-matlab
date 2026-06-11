# 2D LiDAR SLAM — From a Kalman Filter to Autonomous Navigation

![EKF SLAM](results_media/ekf_slam_map.gif)

I built this to actually understand autonomous navigation from the ground up — not just follow along with tutorials, but implement each algorithm myself, break it, fix it, and move on only once I understood what was happening. Started with a Kalman filter on a simulated robot, ended up with a robot that maps a maze and patrols it indefinitely. Here's how it went.

---

## The idea

Brian Douglas's Autonomous Navigation series got me started. The goal was to build each layer of a real navigation stack — localisation, mapping, planning, control — as standalone modules that build on each other, so the progression is visible in the code.

Seven modules later, here's what exists.

---

## What's in here

### 01 — EKF SLAM
The foundation. A differential-drive robot navigates a 2D landmark world, using a virtual LiDAR to observe them. The Extended Kalman Filter jointly tracks robot pose and every landmark position in a single growing state vector. The tricky part was data association — figuring out which observation corresponds to which landmark using a Mahalanobis distance gate. Get it wrong and the map corrupts itself immediately.

![EKF metrics](results_media/ekf_slam_metrics.gif)

### 02 — Loop Closure Detection
When the robot revisits a known area, you can correct accumulated drift by recognising the overlap. The challenge is tuning thresholds to avoid false closures — ended up with a cooldown timer and minimum match count before any correction applies.

### 03 — FastSLAM (Particle Filter)
Replaced the single Gaussian EKF with a particle filter where each particle carries its own landmark map. Handles non-linear motion better and scales more gracefully with landmark count. The comparison plot between EKF and FastSLAM trajectories is the best visualisation in the project.

### 04 — Occupancy Grid Mapping
Instead of discrete landmarks, this builds a dense grid using a log-odds inverse sensor model — every cell updated as free or occupied from LiDAR rays. The kind of map you'd actually use for navigation.

### 05 — ROS 2 Integration
Wrapped EKF SLAM in a ROS 2 Python node publishing odometry, landmarks, and path topics to RViz2. Getting DDS networking to work between Windows and WSL2 was its own adventure — full setup guide in the module folder.

### 06 — A* Navigation
The occupancy grid from Module 04 feeds into an A* planner. Click a goal, watch the robot navigate there. Two-stage path smoother (greedy shortcutting + cubic spline) turns the raw staircase path into something a real robot could follow.

![A* Navigation](results_media/nav_astar.gif)

### 07 — Full Autonomous Navigation Pipeline
Everything comes together. The robot maps a corridor maze in real time, plans paths through the centre crossing, and patrols between clicked goals indefinitely. Pure pursuit controller for smooth cornering. Dynamic replanning when the path gets blocked. The full autonomy loop running end-to-end.

![Autonomous Navigation](results_media/Wall_slam.gif)

---

## Modules at a glance

| # | Module | Algorithm | Key concept |
|---|--------|-----------|-------------|
| 01 | [EKF SLAM](01_ekf_slam/) | Extended Kalman Filter | Joint pose + map covariance |
| 02 | [Loop Closure](02_loop_closure/) | EKF + submap matching | Drift correction on revisit |
| 03 | [FastSLAM](03_fastslam/) | Particle filter + per-LM EKF | O(M·N) vs O(N²) |
| 04 | [Occupancy Grid](04_occupancy_grid/) | Log-odds inverse sensor model | Dense free/occupied map |
| 05 | [ROS 2 Integration](05_ros2_integration/) | EKF SLAM node | Live sensor pipeline |
| 06 | [A* Navigation](06_astar_navigation/) | A* + path smoothing | Click-to-navigate on SLAM map |
| 07 | [Autonomous Navigation](07_autonomous_navigation/) | Pure pursuit + dynamic replan | Full patrol pipeline |

---

## How the modules connect

```
EKF SLAM (01)
  └─ + Loop closure (02)         ← fixes unbounded drift
       └─ FastSLAM (03)          ← better with many landmarks
            └─ Occupancy grid (04) ← dense map for navigation
                 └─ ROS 2 (05)   ← real sensors + RViz2
                 └─ A* (06)      ← click a goal, watch it go
                      └─ Autonomous patrol (07) ← full loop
```

---

## Quick start

```matlab
% Module 01 — EKF SLAM
cd 01_ekf_slam
ekf_slam_main

% Module 06 — A* Navigation (interactive)
cd ../06_astar_navigation
astar_navigation_main
% → wait for map to build, then click a goal

% Module 07 — Autonomous patrol (interactive)
cd ../07_autonomous_navigation
autonomous_nav_main
% → click 3 patrol goals, press Enter, screen-record
```

```bash
# Module 05 — ROS 2 node (WSL2 + ROS 2 Humble)
cd 05_ros2_integration
# See SETUP_GUIDE.md inside for full steps
```

---

## Things I learned the hard way

**MATLAB quirks:**
- `for ~=` is invalid — `~` only works as a function output discard
- `yyaxis` silently hijacks the colour cycle and breaks legend colours — fix with explicit `gobjects` handle arrays
- `getframe()` captures blank frames with OpenGL in R2024a — use `print() → PNG → imwrite()` instead
- Coordinate row conventions aren't consistent between MATLAB's `image()` (row 1 = top = high y) and natural grid indexing (row 1 = bottom = low y) — mixing them causes path planners to check the wrong cells

**ROS 2 / WSL2:**
- MATLAB's ROS 2 bridge doesn't reliably connect over DDS from Windows to WSL2 — replaced with a Python publisher running natively inside WSL2
- A stale `CYCLONEDDS_URI` loopback entry in `.bashrc` silently blocks multicast
- Node startup order matters: static TF → EKF node → RViz2 → Python bridge

**Navigation:**
- Pre-seeding the log-odds grid with walls before any LiDAR runs is essential — otherwise the planner happily routes straight through walls on step 1
- Pure pursuit with a short lookahead (0.4m) tracks corners much better than waypoint stepping — the curvature-based speed scaling is key
- Loose data association thresholds cause false loop closures that corrupt the entire map in seconds

---

## Requirements

- MATLAB R2024a, Image Processing Toolbox
- Python 3.10+, numpy, scipy (Module 05)
- ROS 2 Humble, RViz2 (Module 05)

---

## References

1. Thrun, Burgard, Fox — *Probabilistic Robotics* (2005)
2. Montemerlo et al. — "FastSLAM: A Factored Solution to the SLAM Problem" (2002)
3. Moravec & Elfes — "High Resolution Maps from Wide Angle Sonar" (1985)
4. Brian Douglas — Autonomous Navigation series (YouTube / MATLAB)