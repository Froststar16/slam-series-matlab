# Module 10 — Benchmark Capstone

Every previous module in this series was its own demo, with its own data,
its own random seed, its own story. This module asks the question that ties
all of them together: **if you ran all of them on the exact same problem,
which one would actually win — and by how much?**

## The shared dataset

`generate_benchmark_data.m` builds ONE scenario that every method below runs
on: a figure-eight trajectory (two 8m square loops sharing the origin as a
junction, 424 poses), 32 landmarks split across both loop regions with
deliberate overlap near the origin, noisy odometry with the same systematic
drift bias used in Module 09, and **4 loop closure events** — more than
Module 09's single closure, since the figure-eight revisits the origin three
times.

Every method gets the identical odometry stream and the identical noisy
landmark observations. The EKF-based methods (01, 02) and FastSLAM (03) do
their own data association on raw `(dx, dy)` measurements — they are NOT told
which landmark is which. The pose graph (09) uses true landmark IDs for its
observation edges, same as Module 09 — data association is treated as a
separate problem, so this isolates the optimization itself.

## The four methods

- **01 — EKF SLAM**: standard EKF SLAM, but landmarks can only be
  re-associated within a 60-pose window. The ~212-pose gap between loop A and
  loop B means landmarks near the origin seen in loop A are NOT re-associated
  when loop B revisits them — they become new, duplicate landmarks instead.
- **02 — EKF + Loop Closure**: the *identical* EKF code, with an unbounded
  re-association window. The same revisit now re-associates with loop A's
  original landmarks, and the EKF update pulls the current pose back into
  consistency.
- **03 — FastSLAM**: a 50-particle filter, each particle carrying its own
  pose hypothesis and its own per-landmark EKFs, with full (unbounded)
  per-particle re-association.
- **09 — Pose Graph SLAM**: Module 09's from-scratch Gauss-Newton solver,
  unchanged, applied to a full pose+landmark graph built from this dataset's
  odometry, observations, and all 4 loop closures.

## Results

| Method | ATE [m] | RPE trans [m] | RPE rot [deg] | Landmarks | Runtime [s] |
|---|---|---|---|---|---|
| EKF (01) | 1.860 | 0.070 | 1.303 | 54 | 0.225 |
| EKF+LC (02) | 0.328 | 0.134 | 1.553 | 46 | 0.609 |
| FastSLAM (03, 50p) | 2.809 | 0.054 | 1.358 | 61 | 15.025 |
| **PoseGraph (09)** | **0.101** | **0.030** | **0.752** | 32 | 1.572 |

![Trajectory comparison](../results_media/trajectory_comparison.png)
![ATE comparison](../results_media/ate_comparison.png)
![RPE comparison](../results_media/rpe_comparison.png)
![Runtime comparison](../results_media/runtime_comparison.png)

## What the numbers actually mean

**The headline progression**: 1.860m → 0.328m → 0.101m. Adding loop closure
re-association to plain EKF SLAM is a **5.7x** improvement. Replacing the
filter entirely with a jointly-optimized pose graph is another **3.2x** on
top of that — **18.4x** better than the plain EKF baseline, using only 1.572s
of compute.

**The ATE-vs-RPE divergence is the most interesting single result here.**
EKF+LC's ATE improves dramatically (0.328m vs EKF's 1.860m), but its RPE gets
*worse* (0.134m vs 0.070m). ATE measures global position error; RPE measures
local consistency over short windows. EKF+LC's correction happens as a single
*snap* at the moment of revisit — excellent for global consistency, but that
snap is itself a large local discontinuity relative to ground truth's smooth
motion, which RPE picks up directly. A trajectory can be **globally righter
but locally jumpier**.

Pose graph SLAM wins on *both* metrics (0.101m ATE, 0.030m RPE) because its
correction is distributed smoothly across all 424 poses by the joint
optimization — there's no single snap, just a gentle global redistribution of
error. That's the real argument for graph-based SLAM over filter-based loop
closure: not just "more accurate," but **smoother**.

## The FastSLAM particle sweep

| FastSLAM | ATE [m] | RPE trans [m] | RPE rot [deg] | Landmarks | Runtime [s] |
|---|---|---|---|---|---|
| 50 particles | 2.809 | 0.054 | 1.358 | 61 | 15.0 |
| 200 particles | 1.762 | 0.086 | 1.428 | 62 | 110.4 |

![FastSLAM particle sweep](../results_media/fastslam_particle_sweep.png)
![FastSLAM sweep trajectories](../results_media/fastslam_sweep_trajectories.png)

At 50 particles, FastSLAM has the *worst* ATE of all four methods (2.809m) —
worse even than plain EKF (01) — despite having, in principle, the same
"global re-association" capability as EKF+LC. 4x-ing the particle count to
200 improves ATE to 1.762m (now just barely better than EKF (01)), but the
landmark count barely moves (61 → 62) and **RPE gets worse, not better**
(0.054m → 0.086m), for **7.3x the runtime**.

The landmark count is the tell: if more particles were fixing the
re-association problem, the landmark count would drop toward 32 (the true
count) as particles started correctly re-using loop A's landmarks during loop
B. It doesn't. What's happening instead is that each particle propagates using
sampled odometry noise matching the *process* model, but new landmarks are
initialized using only *sensor* noise (~0.05m) — there's no margin for "this
particle's pose might already be slightly off." By the time the robot reaches
the origin crossing, the accumulated pose spread across particles (a few cm
to tens of cm) is often enough to push every particle's prediction outside the
gate, so re-association fails for essentially the whole population, and a new
landmark gets spawned everywhere. More particles means more *chances* that one
particle's accumulated drift happens to line up — which is why ATE improves
a little — but it doesn't fix the systematic mismatch between the proposal
distribution and the sensor's actual informativeness. This is the textbook
limitation of FastSLAM 1.0 that motivated later "improved proposal"
variants (Montemerlo et al.): **throwing more particles at it doesn't fix the
problem, it just averages over it, at exponential cost.**

## Computational cost, in practice

EKF (01) is the cheapest at 0.225s. EKF+LC (02) costs 2.7x more (0.609s) —
the price of searching the *entire* landmark map for re-association
candidates on every observation, instead of just a recent window. Pose graph
(09) costs 7x the baseline (1.572s) but delivers the best accuracy by a wide
margin — a genuinely good trade. FastSLAM at 50 particles costs **67x** the
baseline for the *worst* accuracy of the four; at 200 particles, **490x** the
baseline for accuracy that's still 17x worse than the pose graph. At this
problem size (424 poses, 32 landmarks), none of these are "slow" in absolute
terms — but the *relative* costs already show the asymptotic story: EKF
SLAM's per-step cost grows with the size of the association search space (01
vs 02), FastSLAM's grows linearly with particle count but apparently needs a
lot of particles to be competitive, and the pose graph's sparse Gauss-Newton
solve stays cheap even as it optimizes 424 poses and 32 landmarks jointly.

## Files

| File | What it does |
|---|---|
| `benchmark_main.m` | Orchestrates everything — run this one |
| `generate_benchmark_data.m` | The one shared dataset all four methods run on |
| `run_ekf_slam.m` | EKF SLAM core; `assoc_window` parameter covers both 01 and 02 |
| `run_fastslam.m` | FastSLAM 1.0 core, adapted to the shared dataset |
| `run_pose_graph_slam.m` | Wraps Module 09's `gauss_newton_pgo.m`, unchanged |
| `gauss_newton_pgo.m` | Copied verbatim from Module 09 |
| `compute_ate.m` | Copied from Module 09 |
| `compute_rpe.m` | New: Relative Pose Error (translation + rotation) |

## Closing thoughts on the series

Ten modules ago, this started as "implement EKF SLAM and see if I actually
understand the Kalman filter update." It ends here, with a from-scratch
nonlinear optimizer beating both the filter it was meant to replace and the
particle-filter alternative, on a benchmark built specifically to make that
comparison fair — and with the numbers explaining *why*, not just confirming
*that*. The series covers the EKF/particle-filter/pose-graph spectrum of
mainstream 2D SLAM, a full navigation stack (A*, pure pursuit, frontier
exploration), and a live ROS 2 deployment. The benchmark capstone is the
piece that turns all of that from "a collection of demos" into "a study with
a conclusion."

## References

1. Thrun, Burgard, Fox — *Probabilistic Robotics* (2005)
2. Montemerlo et al. — "FastSLAM: A Factored Solution to the SLAM Problem" (2002)
3. Grisetti, Kummerle, Stachniss, Burgard — "A Tutorial on Graph-Based SLAM" (2010)
4. Sturm et al. — "A Benchmark for the Evaluation of RGB-D SLAM Systems" (2012) — ATE/RPE definitions