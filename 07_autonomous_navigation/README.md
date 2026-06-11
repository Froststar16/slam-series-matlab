# Module 07 — Full Autonomous Navigation Pipeline

The module where everything comes together. SLAM builds the map, A* plans the path, pure pursuit executes it, and the robot patrols indefinitely between goals you click on the map. It's the first time the full autonomy loop runs end-to-end in real time.

---

## What's in here

```
07_autonomous_navigation/
├── autonomous_nav_main.m    ← full pipeline (run this)
├── pure_pursuit.m           ← path tracking controller
└── README.md
```

Needs `astar_planner.m` from `06_astar_navigation/` — the script adds it to the path automatically.

---

## The pipeline

Every time step, in order:

```
1. LiDAR ray-cast     → 72 beams against maze walls
2. Occupancy grid     → log-odds update (walls pre-seeded at startup)
3. Goal manager       → cycle to next goal when within 0.45m
4. Path check         → is the current path still clear?
5. A* replanner       → fires if blocked or no path yet, flashes red
6. Pure pursuit       → lookahead point → (v, w) commands
7. Motion step        → apply control + noise
```

---

## Quick start

```matlab
cd 07_autonomous_navigation
autonomous_nav_main
```

1. Red/blue map renders — **red = wall, dark blue = free corridor**
2. Click **3 goals** in the dark blue quadrant rooms
3. Press **Enter** — start screen recording here
4. Robot patrols until step 20000

![Autonomous Navigation](../results_media/Wall_slam.gif)

**Good patrol goals** (one per quadrant, away from dividers):
- `(2.5, 7.5)` — top-left
- `(7.5, 6.5)` — top-right
- `(7.5, 2.5)` — bottom-right

---

## The maze

Simple cross-divider layout — two walls with a 2m gap each, verified connected via BFS before the MATLAB code was written:

```
+----+----+----+
|         |   |   y=5 horizontal divider: gap x=4..6
|    +    +   |
|    |    |   |   x=5 vertical divider:   gap y=4..6
+----+    +---+
|              |
+--------------+
```

All four quadrant rooms connect through the centre crossing. Corridors are ~4m wide so `INFLATE_R=2` (0.2m safety margin) leaves plenty of navigable space.

---

## Key design decisions

**Pre-seeded wall grid:** The log-odds grid is initialised with walls at `L_MAX` before any LiDAR scan runs. This means A* never plans through walls on step 1 — no waiting for the robot to discover them first.

**Local path smoother:** `path_smoother.m` from Module 06 uses a different grid row convention (row 1 = high y). Rather than fight the convention mismatch, Module 07 uses its own `smooth_path_local` + `los_free` functions that use the same `xy2grid` convention as everything else in the script. LOS checks now correctly block wall shortcuts.

**Pure pursuit over waypoint stepping:** The robot steers toward a lookahead point 0.4m ahead on the path rather than snapping to the nearest waypoint. This gives smooth, natural-looking motion through corners. Speed scales down automatically with curvature.

**Dynamic replanning:** Every 15 steps the planner checks if any cell along the current path has flipped to occupied in the latest grid update. If so, A* reruns and the path line flashes red briefly — visible in the recording.

---

## Parameters

| Parameter | Value | Effect |
|-----------|-------|--------|
| `INFLATE_R` | 2 cells | Safety margin around walls |
| `LOOKAHEAD` | 0.40 m | Pure pursuit lookahead — shorter = tighter corners |
| `V_MAX` | 0.25 m/s | Max speed — slower = less wall overshoot |
| `REPLAN_EVERY` | 15 steps | Path validity check frequency |
| `GOAL_THRESH` | 0.45 m | Distance to count as "goal reached" |
| `MAX_STEPS` | 20000 | Run length |

---

## Known limitations

- Path occasionally grazes the wall junction at the centre crossing (x=5, y=5). This is a sub-cell corner issue in A* diagonal moves through the 2m gap. Fixable with a finer grid resolution or Jump Point Search.
- Navigation uses ground truth pose — EKF localisation is the natural next step (Option B).
- Maze is static — no moving obstacles.

---

## How it connects to previous modules

```
EKF SLAM (01)         → pose + landmark estimation pattern
Occupancy Grid (04)   → same log-odds model, same grid conventions
A* Planner (06)       → astar_planner.m reused directly
Pure Pursuit (07)     → NEW: replaces waypoint stepping
Dynamic Replan (07)   → NEW: path validity check each step
Continuous Patrol (07)← NEW: goal cycling + indefinite loop
```