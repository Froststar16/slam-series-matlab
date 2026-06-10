# 2D LiDAR SLAM — From a Kalman Filter to A* Navigation

![EKF SLAM](results_media/ekf_slam_map.gif)

I built this project to actually understand how robots figure out where they are while simultaneously building a map of their surroundings — the classic SLAM problem. Started from scratch in MATLAB, ended up with a live ROS 2 pipeline running in RViz2, and then layered A* path planning on top of the whole thing. Here's how it went.

---

## The idea

I kept watching Brian Douglas's Autonomous Navigation series and wanted to go beyond just following along. So I decided to implement each algorithm myself, debug it, break it, fix it, and only move to the next one once I genuinely understood what was happening under the hood.

Six modules later, here we are.

---

## What's in here

### 01 — EKF SLAM
The foundation. A simulated differential-drive robot drives around a 2D environment with landmarks, using a virtual LiDAR to sense them. The Extended Kalman Filter jointly tracks the robot's pose and every landmark position in one big state vector that grows as new landmarks are discovered.

The tricky part was getting the data association right — figuring out which observation corresponds to which landmark using a Mahalanobis distance gate. Get it wrong and the map corrupts itself almost immediately.

![EKF metrics](results_media/ekf_slam_metrics.gif)

### 02 — Loop Closure Detection
Once the robot revisits a place it's been before, you can correct the accumulated drift by recognising the overlap between submaps. Sounds simple. In practice, tuning the thresholds to avoid false closures is genuinely fiddly — I ended up with a cooldown timer and minimum match count before any correction gets applied.

### 03 — FastSLAM (Particle Filter)
Swapped the single Gaussian EKF for a particle filter where each particle carries its own landmark map. The big win is that it handles non-linear motion better and scales more gracefully with landmark count. The comparison plot between EKF and FastSLAM trajectories is probably my favourite visualisation in the whole project.

### 04 — Occupancy Grid Mapping
Instead of tracking discrete landmarks, this module builds a dense grid of the environment using a log-odds inverse sensor model — every cell gets updated as free or occupied based on what the LiDAR rays hit. The result is the kind of map you'd actually use for navigation.

### 05 — ROS 2 Integration
Took the EKF SLAM algorithm and wrapped it in a proper ROS 2 Python node, publishing odometry, landmarks, and path topics that RViz2 can visualise in real time. Getting DDS networking to behave between Windows and WSL2 was its own adventure — there's a full setup guide in the module folder.

### 06 — A* Navigation
The payoff module. The occupancy grid from Module 04 feeds directly into an A* planner — you click a goal on the map and watch the robot navigate there. Two-stage path smoother (greedy shortcutting + cubic spline) turns the raw staircase path into something a real robot could actually follow.

![A* Navigation](results_media/nav_astar.gif)

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

---

## How the modules connect

```
EKF SLAM (01)
  └─ + Loop closure detection (02)     ← fixes unbounded drift
       └─ FastSLAM comparison (03)     ← better with many landmarks  
            └─ Occupancy grid (04)     ← dense map for navigation
                 └─ ROS 2 node (05)   ← real sensors + RViz2
                 └─ A* Navigation (06) ← click a goal, watch it go
```

---

## Quick start

```matlab
% Module 01 — EKF SLAM
cd 01_ekf_slam
ekf_slam_main

% Module 03 — FastSLAM vs EKF comparison
cd ../03_fastslam
fastslam

% Module 04 — Occupancy grid
cd ../04_occupancy_grid
occupancy_grid

% Module 06 — A* Navigation (interactive)
cd ../06_astar_navigation
astar_navigation_main
% → wait for map to build, then click a goal
```

```bash
# Module 05 — ROS 2 node (WSL2 + ROS 2 Humble)
cd 05_ros2_integration
# See the SETUP_GUIDE.md inside for full steps
ros2 run ekf_slam_py ekf_slam_node
```

---

## Things I learned the hard way

**MATLAB quirks:**
- `for ~=` is not valid — `~` only works as a function output discard, not a loop variable
- `yyaxis` silently hijacks the colour cycle and breaks legend colours — fix with explicit `gobjects` handle arrays
- `getframe()` captures blank frames with the OpenGL renderer in R2024a — use `print() → PNG → imwrite()` instead

**ROS 2 / WSL2:**
- MATLAB's ROS 2 bridge doesn't reliably connect to WSL2 over DDS — replaced with a Python publisher running natively in WSL2
- A stale `CYCLONEDDS_URI` loopback entry in `.bashrc` will silently block multicast — check for it if topics aren't flowing
- Node startup order matters: static TF → EKF node → RViz2 → Python bridge

**SLAM:**
- Loose data association thresholds cause false loop closures that corrupt the entire map in seconds
- Occupancy grids are only as good as the trajectory that built them — unknown space is grey for a reason

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