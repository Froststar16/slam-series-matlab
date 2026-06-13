# Module 09 — Pose Graph SLAM (from-scratch Gauss-Newton)

So far this series has built the SLAM "front end" — EKF SLAM (01), a particle
filter comparison (03), occupancy grid mapping (04), loop closure detection
(05), and a full A* navigation pipeline (06), plus a live ROS 2 demo. What's
been missing is the "back end": the part that actually *does something* with
a detected loop closure besides flag it.

That's the whole point of this module. A pose graph turns your trajectory into
a graph: every pose is a node, every odometry step is an edge, and — crucially
— every loop closure is *also* an edge, connecting two nodes that aren't
adjacent in time but that the robot believes are physically close together.
Landmarks get nodes too, connected to the poses that observed them. Then you
ask: "what set of poses and landmark positions makes all of these edges as
consistent as possible?" That's a nonlinear least-squares problem, and solving
it is what corrects drift.

## What's actually happening here

1. **Synthetic data** (`generate_synthetic_data.m`) — a robot drives a roughly
   square 8m loop (212 poses). Odometry has a small systematic bias (it
   slightly over-rotates and over-estimates distance every step), so
   dead-reckoning alone does **not** close the loop — the estimated
   trajectory drifts to 1.33m ATE by the end. Along the way the robot makes
   668 landmark observations of 18 landmarks, and right at the end of the
   loop it genuinely revisits its starting area — that revisit becomes a
   single loop closure constraint.

2. **From-scratch Gauss-Newton** (`gauss_newton_pgo.m`) — this is the core of
   the module. Every pose (x, y, theta) and every landmark (x, y) is a variable
   in one big state vector (675 variables here). For each edge type
   (odometry/loop-closure pose-pose constraints, and pose-landmark observation
   constraints), the error function and its Jacobian are computed
   analytically — no numerical differentiation. Those get assembled into a
   sparse information matrix `H` and gradient vector `b`, and each iteration
   solves `H * dx = -b` and applies the update. Pose 1 gets a stiff prior to
   anchor the gauge freedom (a pose graph by itself has no absolute frame — it
   can be rigidly rotated/translated without changing any error, so something
   has to be pinned down).

   **Result:** converges in 5 iterations, chi2 drops from 133,474 to 1,314,
   and trajectory ATE drops from **1.331m to 0.076m** — a 17x reduction in
   drift.

3. **Animated convergence** — every Gauss-Newton iteration is rendered and
   exported as a frame, then assembled into `pose_graph_convergence.gif`. You
   can watch the drifted trajectory snap back into a closed loop and the
   landmark cloud tighten up around it over the 5 iterations.

4. **What's the landmark network actually buying you?** — re-running the same
   solver on just the pose-only sub-graph (odometry + the single loop closure,
   no landmarks at all) gives an ATE of **0.41m** — much better than the raw
   0.076m for the full graph, but still 5x worse. With only one loop closure
   edge in the whole graph, the pose-only backbone has very little to work
   with; the 668 landmark observations are doing most of the heavy lifting.
   `full_vs_pose_only_comparison.png` overlays both results against ground
   truth.

5. **Correctness check, no toolbox required** (`test_pgo_toy_graph.m`) —
   MATLAB's `optimizePoseGraph` (Navigation Toolbox) only handles pose-pose
   edges, so it can't validate the full pose+landmark graph directly, and on
   this install it isn't even available (Navigation Toolbox not licensed). The
   `validate_with_pose_graph_toolbox.m` path is still there and will run *if*
   the toolbox is present, but the primary correctness check is a standalone
   "ground truth recovery" test: a tiny noise-free graph where every edge is
   exactly consistent with a known ground truth, so the true optimum has
   chi2 = 0. The solver should recover that ground truth to numerical
   precision regardless of where it starts — this is the standard way SLAM
   codebases test their optimizers, and it doesn't depend on any toolbox.

   **Result:** converges in 4 iterations to chi2 = 1.1e-26, with pose and
   landmark errors on the order of 1e-15 — floating-point noise, i.e. the
   Jacobians are exact, not just approximately right. **PASS.**

## Results

**Before optimization** — dead-reckoned trajectory drifts away from the
ground-truth square, and estimated landmark positions (orange) are offset
from their true positions (black X):

![Before optimization](results_media/before_optimization.png)

**After optimization** — the full pose+landmark graph snaps the trajectory
back onto the ground-truth loop, and landmark estimates land almost exactly
on their true positions:

![After optimization](results_media/after_optimization.png)

**Convergence** — chi2 drops from 133,474 to 1,314 in 5 iterations:

![Chi-squared convergence](results_media/chi2_convergence.png)

**Animated convergence** — watch the drift get corrected iteration by
iteration:

![Pose graph convergence animation](results_media/pose_graph_convergence.gif)

**Full graph vs pose-only sub-graph** — landmarks vs. a single loop closure:

![Full graph vs pose-only comparison](results_media/full_vs_pose_only_comparison.png)

## Files

| File | What it does |
|---|---|
| `pose_graph_slam_main.m` | Orchestrates everything — run this one |
| `generate_synthetic_data.m` | Builds ground truth + drifted graph + edges |
| `gauss_newton_pgo.m` | The from-scratch optimizer (Jacobians live here) |
| `compute_ate.m` | RMSE between an estimated and ground-truth trajectory |
| `test_pgo_toy_graph.m` | Standalone ground-truth-recovery correctness check |
| `validate_with_pose_graph_toolbox.m` | Runs `optimizePoseGraph` for comparison, if available |
| `save_pgo_gif.m` | print()->PNG->imwrite GIF export (R2024a-safe) |

## Outputs

In addition to the images/animation shown above, console output reports ATE
(RMSE, in metres) for before/after, the pose-only sub-graph, and (if
Navigation Toolbox is available) `optimizePoseGraph`. `test_pgo_toy_graph.m`
(run separately) reports the PASS/FAIL ground-truth recovery check shown in
section 5 above, with no toolbox dependency.

## A note on the numbers

The exact ATE values depend on the random seed and the bias parameters at the
top of `generate_synthetic_data.m` (`odom_sigma`, `bias_dtheta`, `bias_scale`).
On the seed used here: 1.331m drift before optimization, 0.076m after with the
full graph, 0.41m for the pose-only sub-graph. These three numbers — drift
before/after, and the marginal contribution of the landmark network — are
exactly the kind of thing that'll get reused as a baseline in Module 10's
benchmark capstone, where EKF SLAM, FastSLAM, and this pose-graph-corrected
trajectory all get compared head-to-head on the same ATE metric.

## Debugging notes

`validate_with_pose_graph_toolbox.m` tries `poseGraph` and falls back to
`robotics.PoseGraph`; if neither is available (Navigation Toolbox not
licensed), the main script catches this and skips the toolbox comparison
without affecting any other output. `test_pgo_toy_graph.m` is the
toolbox-independent correctness check and should be run at least once to
confirm the solver recovers a known ground truth exactly.

## What's next

Module 10 is the capstone: take EKF SLAM, FastSLAM, and this pose-graph result,
run them all on the same trajectory, and compare them quantitatively with
ATE/RPE — turning six-plus separate demos into one coherent evaluation story.