# 2D LiDAR SLAM Series — MATLAB

A from-scratch implementation of the core algorithms in autonomous mobile robotics, built module by module in MATLAB. Started after watching Brian Douglas's Autonomous Navigation series and wanting to actually build the things being described, not just watch them.

Each module stands alone and runs independently. The later ones reuse pieces from earlier ones — which is kind of the point.

---

## Modules

| # | Module | What it does | Key result |
|---|--------|-------------|------------|
| 01 | [EKF SLAM](01_ekf_slam/) | Extended Kalman Filter simultaneous localisation and mapping | Pose error < 0.15 m after 500 steps |
| 02 | [Loop Closure](02_loop_closure/) | Detect when the robot revisits a known place and correct accumulated drift | Drift correction on return visits |
| 03 | [FastSLAM](03_fastslam/) | Particle filter SLAM — compare against EKF on the same trajectory | Side-by-side EKF vs particle filter |
| 04 | [Occupancy Grid](04_occupancy_grid/) | Build a probabilistic map of free/occupied space from LiDAR scans | Log-odds grid from Bresenham ray traces |
| 05 | [ROS 2 Integration](05_ros2_integration/) | Live pipeline in WSL2 with RViz2 visualisation | Real-time EKF SLAM in ROS 2 Humble |
| 06 | [A\* Navigation](06_astar_navigation/) | Interactive goal selection, A* planning, path smoothing, animated execution | Full navigation pipeline with spline paths |
| 07 | [Autonomous Patrol](07_autonomous_navigation/) | Pure pursuit controller, dynamic replanning, continuous multi-goal patrol | Robot patrols indefinitely between clicked goals |
| 08 | [Frontier Exploration](08_frontier_exploration/) | Robot autonomously maps an unknown environment with no human input | 88.2% coverage in 5475 steps |

---

## Results

### EKF SLAM — mapping and metrics
![EKF SLAM map](results_media/ekf_slam_map.png)
![EKF SLAM metrics](results_media/ekf_slam_metrics.png)

### A* Navigation
![A* navigation](results_media/nav_astar.gif)

### Autonomous Patrol
![Autonomous patrol](results_media/Wall_slam.gif)

### Frontier Exploration
![Frontier exploration](results_media/frontier_exploration.png)

---

## How the modules connect

```
01 EKF SLAM ──────────────────────────────────── pose estimation backbone
     │
02 Loop Closure ──────────────────────────────── drift correction on revisit
     │
03 FastSLAM ──────────────────────────────────── alternative: particle filter
     │
04 Occupancy Grid ────────────────────────────── probabilistic map from scans
     │
05 ROS 2 ─────────────────────────────────────── live pipeline, RViz2
     │
06 A* Navigation ─────────────────────────────── plan paths through the grid
     │
07 Autonomous Patrol ─────────────────────────── pure pursuit + replanning
     │
08 Frontier Exploration ──────────────────────── no human input, robot decides
```

---

## What I actually learned

The theory makes it look cleaner than it is. Some things that only became obvious by building it:

**EKF SLAM breaks badly on walls.** Point-landmark EKF works well for corner features. Apply it to straight walls and every beam along the wall creates a new "landmark" — the state vector explodes before the robot leaves the first room. Module 08 ended up dropping EKF entirely and switching to dead-reckoning + occupancy grid, which is both faster and more honest about what walls actually are.

**MATLAB scoping rules bite hard in scripts with functions.** Nested `function` blocks in a script file can't see workspace variables. Learned this the hard way multiple times — solution is either separate `.m` files or fully inlining the math. Both approaches are in use across the modules.

**ROS 2 networking on Windows is genuinely difficult.** The MATLAB ROS 2 bridge doesn't work with WSL2 due to Windows-to-WSL2 DDS multicast issues. Module 05 replaced it with a Python publisher running natively inside WSL2, which works cleanly.

**GIF export in MATLAB R2024a is broken with the default renderer.** `getframe()` silently captures blank frames. The fix is a `print() → PNG → imread → imwrite` chain. All modules that export GIFs use this.

---

## Setup

MATLAB R2024a with:
- Robotics System Toolbox
- Navigation Toolbox
- Computer Vision Toolbox
- Sensor Fusion and Tracking Toolbox
- Image Processing Toolbox

For Module 05 (ROS 2): WSL2 Ubuntu 22.04, ROS 2 Humble.

Each module runs independently — `cd` into the folder and run the main script.

---

## References

1. Thrun, Burgard, Fox — *Probabilistic Robotics* (2005)
2. Montemerlo et al. — "FastSLAM: A Factored Solution to the SLAM Problem" (2002)
3. Moravec & Elfes — "High Resolution Maps from Wide Angle Sonar" (1985)
4. Brian Douglas — Autonomous Navigation series (YouTube / MATLAB)