# 2D LiDAR SLAM — From a Kalman Filter to a Robot That Explores on Its Own

![EKF SLAM](results_media/ekf_slam_map.gif)

I built this project to actually understand how robots figure out where they are while simultaneously building a map of their surroundings — the classic SLAM problem. Started from scratch in MATLAB with a basic Extended Kalman Filter, ended up with a live ROS 2 pipeline, A* navigation, a full autonomous patrol loop, and finally a robot that maps an entire unknown environment with zero human input. Here's how it went.

---

## The idea

I kept watching Brian Douglas's Autonomous Navigation series and wanted to go beyond just following along. So I decided to implement each algorithm myself, debug it, break it, fix it, and only move to the next one once I genuinely understood what was happening under the hood.

Eight modules later, here we are.

---

## What's in here

### 01 — EKF SLAM
The foundation. A simulated differential-drive robot drives around a 2D environment with landmarks, using a virtual LiDAR to sense them. The Extended Kalman Filter jointly tracks the robot's pose and every landmark position in one big state vector that grows as new landmarks are discovered.

The tricky part was getting data association right — figuring out which observation corresponds to which landmark using a Mahalanobis distance gate. Get it wrong and the map corrupts itself almost immediately.

![EKF metrics](results_media/ekf_slam_metrics.gif)

### 02 — Loop Closure Detection
Once the robot revisits a place it's been before, you can correct accumulated drift by recognising the overlap between submaps. Sounds simple. In practice, tuning the thresholds to avoid false closures is genuinely fiddly — I ended up with a cooldown timer and a minimum match count before any correction gets applied. Without those, the filter "closes the loop" 200 times in a row and the map falls apart.

### 03 — FastSLAM (Particle Filter)
Swapped the single Gaussian EKF for a particle filter where each particle carries its own landmark map. The big win is that it handles non-linear motion better and scales more gracefully with landmark count. The side-by-side comparison plot between EKF and FastSLAM trajectories is one of my favourite visualisations in the whole project — you can actually *see* where the two filters diverge.

### 04 — Occupancy Grid Mapping
Instead of tracking discrete landmarks, this module builds a dense grid of the environment using a log-odds inverse sensor model — every cell gets nudged toward free or occupied based on what the LiDAR rays pass through and hit. This grid is the foundation everything from Module 06 onward is built on.

### 05 — ROS 2 Integration
Took the EKF SLAM node and ran it live inside ROS 2 Humble on WSL2, visualised in RViz2. This one fought back the hardest — MATLAB's ROS 2 bridge just doesn't talk to WSL2 reliably over DDS, so I ended up replacing it with a Python publisher running natively inside WSL2. Also discovered a stale `CYCLONEDDS_URI` setting in `.bashrc` that was silently blocking multicast the whole time. Once that was gone, everything connected first try.

### 06 — A* Navigation
Click a goal, watch the robot navigate there. The occupancy grid from Module 04 becomes the planning space, A* finds a path with an octile-distance heuristic, and a two-stage smoother (greedy shortcutting + cubic spline) turns the raw staircase path into something a real robot could actually follow.

![A* Navigation](results_media/nav_astar.gif)

### 07 — Full Autonomous Navigation Pipeline
Everything from 01–06 comes together. The robot maps a corridor maze in real time, plans paths through a tight centre crossing, and patrols indefinitely between goals I click on the map. Pure pursuit handles smooth cornering, and the path flashes red when it dynamically replans around something blocking the way. This was the first time the *full* autonomy loop ran end-to-end in real time.

![Autonomous Navigation](results_media/Wall_slam.gif)

### 08 — Frontier-Based Autonomous Exploration
The one where I stopped clicking goals entirely. The robot looks at its occupancy grid, finds **frontiers** — the boundary cells between known-free and unknown space — and picks the best one using a hybrid score (`frontier size / distance`), balancing "how much new area will this reveal" against "how far do I have to travel to get there." Then it plans a path there with A*, drives it with pure pursuit, and repeats until there's nothing left to discover.

It finished the maze at **88.2% coverage in 5475 steps** — well under a quarter of the step budget — and stopped cleanly because the remaining ~12% genuinely has no reachable entrance. No human input from start to finish.

![Frontier Exploration](results_media/frontier_exploration.png)

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
| 08 | [Frontier Exploration](08_frontier_exploration/) | Frontier detection + hybrid scoring | Robot picks its own goals |

---

## How the modules connect

```
EKF SLAM (01)
  └─ + Loop closure (02)            ← fixes unbounded drift
       └─ FastSLAM (03)             ← better with many landmarks
            └─ Occupancy grid (04)  ← dense map for navigation
                 ├─ ROS 2 (05)      ← real sensors + RViz2
                 └─ A* (06)         ← click a goal, watch it go
                      └─ Autonomous patrol (07)   ← full loop, clicked goals
                           └─ Frontier exploration (08) ← robot picks its own goals
```

Module 08 quietly drops the EKF landmark tracking that powers 01–07 — more on that below.

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

% Module 08 — Frontier exploration (fully autonomous, no input)
cd ../08_frontier_exploration
frontier_exploration_main
% → just watch
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
- Nested `function` blocks in a script file can't see script-level variables — either split into separate `.m` files or inline the math. Hit this one twice in Module 08 alone.
- Coordinate row conventions aren't consistent between MATLAB's `image()` (row 1 = top) and natural grid indexing (row 1 = bottom) — mixing them sends path planners checking the wrong cells

**ROS 2 / WSL2:**
- MATLAB's ROS 2 bridge doesn't reliably connect over DDS from Windows to WSL2 — replaced with a Python publisher running natively inside WSL2
- A stale `CYCLONEDDS_URI` loopback entry in `.bashrc` silently blocks multicast
- Node startup order matters: static TF → EKF node → RViz2 → Python bridge

**Navigation:**
- Pre-seeding the log-odds grid with walls before any LiDAR runs is essential — otherwise the planner happily routes straight through walls on step 1
- Pure pursuit with a short lookahead (0.4–0.45m) tracks corners much better than waypoint stepping — curvature-based speed scaling is key
- Loose data association thresholds cause false loop closures that corrupt the entire map in seconds

**Frontier exploration (the big one):**
- EKF point-landmark SLAM and long straight walls do **not** mix. Every beam along a wall looks like a "new" landmark, the state vector explodes before the robot leaves the first room, and the O(landmarks) update loop grinds to a crawl. Module 08 dropped EKF landmarks entirely in favour of dead-reckoning + the occupancy grid — ~30x faster and the map stopped getting wall-smear artefacts as a bonus.
- A robot that picks its own goals will eventually pick one that's unreachable, or drift its estimated position into a wall it thinks is free. Needed a stuck-counter, a blacklist with expiry, and a "spiral outward until you find a free cell" recovery before it stopped looping in place forever.
- 88% coverage on a maze like this is actually close to the ceiling — the last bit of grey is rooms with no navigable entrance within the obstacle inflation radius. The robot correctly gave up rather than spinning forever looking for frontiers that don't exist.

---

## Requirements

- MATLAB R2024a, Image Processing Toolbox, Robotics System Toolbox, Navigation Toolbox, Computer Vision Toolbox, Sensor Fusion and Tracking Toolbox
- Python 3.10+, numpy, scipy (Module 05)
- ROS 2 Humble, RViz2 (Module 05)

---

## References

1. Thrun, Burgard, Fox — *Probabilistic Robotics* (2005)
2. Montemerlo et al. — "FastSLAM: A Factored Solution to the SLAM Problem" (2002)
3. Moravec & Elfes — "High Resolution Maps from Wide Angle Sonar" (1985)
4. Brian Douglas — Autonomous Navigation series (YouTube / MATLAB)
5. Yamauchi — "A Frontier-Based Approach for Autonomous Exploration" (1997)