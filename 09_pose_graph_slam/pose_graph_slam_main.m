%% Module 09 -- Pose Graph SLAM (from-scratch Gauss-Newton)
%
% This module takes the loop closures detected in Module 05 and actually
% *uses* them: every pose-to-pose step (odometry) and every detected
% revisit (loop closure) becomes an edge in a pose graph, and every
% landmark observation becomes an edge connecting a pose node to a
% landmark node. A from-scratch Gauss-Newton solver then jointly
% optimizes ALL of it -- poses and landmarks together -- to minimize the
% total constraint error.
%
% Pipeline:
%   1. Generate a synthetic trajectory + landmark map with ground truth,
%      plus a drifted dead-reckoned initial estimate (odometry has a
%      small systematic bias so the loop doesn't close on its own).
%   2. Run from-scratch Gauss-Newton on the FULL pose+landmark graph,
%      animating the iteration-by-iteration convergence.
%   3. Report Absolute Trajectory Error (ATE) before vs after.
%   4. Compare against a pose-only sub-graph (odometry + loop closure,
%      no landmarks) to quantify how much the landmark network
%      contributes beyond the loop closure alone. A separate standalone
%      script, test_pgo_toy_graph.m, validates the solver itself against
%      a noise-free toy graph with a known closed-form solution.

clear; clc; close all;

%% 1. Generate synthetic data ------------------------------------------------
graph = generate_synthetic_data();

N = size(graph.gt_poses,1);
M = size(graph.gt_landmarks,1);
n_odom = sum(strcmp({graph.edges.type},'odom'));
n_loop = sum(strcmp({graph.edges.type},'loop'));
n_obs  = sum(strcmp({graph.edges.type},'obs'));

fprintf('Pose graph: %d poses, %d landmarks\n', N, M);
fprintf('Edges: %d odometry, %d loop closure, %d landmark observations\n', ...
    n_odom, n_loop, n_obs);

%% 2. "Before" snapshot --------------------------------------------------------
fig = figure('Color','w','Position',[100 100 800 800]);
plot_state(graph.gt_poses, graph.gt_landmarks, graph.poses_init, graph.landmarks_init, ...
    'Before optimization (drifted dead reckoning)');
exportgraphics(fig, 'before_optimization.png');

%% 3. From-scratch Gauss-Newton on the FULL pose+landmark graph -----------------
max_iters = 20;
tol = 1e-4;

result_full = gauss_newton_pgo(graph.poses_init, graph.landmarks_init, graph.edges, max_iters, tol);

n_iters = numel(result_full.poses_history) - 1;
fprintf('\nFull graph optimization: %d iterations\n', n_iters);
fprintf('  chi2: %.3f -> %.3f\n', result_full.chi2_history(1), result_full.chi2_history(end));

%% 4. Animate iteration-by-iteration convergence --------------------------------
frame_dir = 'frames_pgo';
if ~exist(frame_dir,'dir'); mkdir(frame_dir); end

for it = 1:numel(result_full.poses_history)
    clf(fig);
    plot_state(graph.gt_poses, graph.gt_landmarks, ...
        result_full.poses_history{it}, result_full.landmarks_history{it}, ...
        sprintf('Gauss-Newton iteration %d / %d', it-1, n_iters));
    drawnow;
    print(fig, fullfile(frame_dir, sprintf('frame_%04d.png', it)), '-dpng', '-r100');
end

save_pgo_gif(frame_dir, 'pose_graph_convergence.gif', 0.3);

%% 5. "After" snapshot ------------------------------------------------------------
clf(fig);
plot_state(graph.gt_poses, graph.gt_landmarks, ...
    result_full.poses_final, result_full.landmarks_final, ...
    'After optimization (full pose + landmark graph)');
exportgraphics(fig, 'after_optimization.png');

%% 6. ATE before / after -----------------------------------------------------------
ate_before = compute_ate(graph.poses_init(:,1:2), graph.gt_poses(:,1:2));
ate_after  = compute_ate(result_full.poses_final(:,1:2), graph.gt_poses(:,1:2));

fprintf('\nTrajectory ATE (RMSE):\n');
fprintf('  before optimization: %.3f m\n', ate_before);
fprintf('  after optimization:  %.3f m\n', ate_after);

%% 7. Pose-only sub-graph: how much do the landmarks actually help? -----------------
% Re-run the from-scratch solver on JUST the odometry + loop-closure edges
% (no landmarks/observations) to quantify how much of the correction in
% step 3 came from the landmark network vs. the single loop closure alone.
pose_edges = graph.edges(~strcmp({graph.edges.type},'obs'));
result_pose_only = gauss_newton_pgo(graph.poses_init, zeros(0,2), pose_edges, max_iters, tol);

ate_pose_only = compute_ate(result_pose_only.poses_final(:,1:2), graph.gt_poses(:,1:2));

fprintf('\nPose-only sub-graph (odometry + %d loop closure, no landmarks):\n', n_loop);
fprintf('  ATE: %.4f m  (vs %.4f m for the full pose+landmark graph)\n', ate_pose_only, ate_after);

clf(fig);
plot_comparison(graph.gt_poses, result_full.poses_final, result_pose_only.poses_final);
exportgraphics(fig, 'full_vs_pose_only_comparison.png');

% Optional bonus: compare against MATLAB's optimizePoseGraph if Navigation
% Toolbox is available (it isn't on every install -- see
% test_pgo_toy_graph.m for a toolbox-free correctness check).
try
    poses_matlab = validate_with_pose_graph_toolbox(graph.poses_init, pose_edges);
    ate_matlab = compute_ate(poses_matlab(:,1:2), graph.gt_poses(:,1:2));
    diff_rmse  = compute_ate(result_pose_only.poses_final(:,1:2), poses_matlab(:,1:2));
    fprintf('  optimizePoseGraph ATE: %.4f m (difference from from-scratch GN: %.4f m)\n', ...
        ate_matlab, diff_rmse);
catch ME
    fprintf('  optimizePoseGraph unavailable (%s)\n', ME.identifier);
    fprintf('  -- run test_pgo_toy_graph.m for an independent correctness check instead\n');
end

%% 8. Chi-square convergence curve --------------------------------------------------
figure('Color','w');
plot(0:n_iters, result_full.chi2_history, '-o', 'LineWidth', 1.5);
xlabel('Gauss-Newton iteration');
ylabel('Total weighted error (\chi^2)');
title('Pose graph optimization convergence');
grid on;
exportgraphics(gcf, 'chi2_convergence.png');

fprintf('\nDone. Outputs:\n');
fprintf('  before_optimization.png\n');
fprintf('  after_optimization.png\n');
fprintf('  pose_graph_convergence.gif\n');
fprintf('  full_vs_pose_only_comparison.png\n');
fprintf('  chi2_convergence.png\n');
fprintf('\n(Run test_pgo_toy_graph.m separately for a ground-truth-recovery check of the solver.)\n');

%% ---------- local helper functions ----------

function plot_state(gt_poses, gt_landmarks, est_poses, est_landmarks, title_str)
hold on; axis equal; grid on;
plot(gt_poses(:,1), gt_poses(:,2), '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.5);
plot(gt_landmarks(:,1), gt_landmarks(:,2), 'kx', 'MarkerSize',8, 'LineWidth',1.5);
plot(est_poses(:,1), est_poses(:,2), '-', 'Color',[0 0.45 0.74], 'LineWidth',2);
if ~isempty(est_landmarks)
    plot(est_landmarks(:,1), est_landmarks(:,2), 'o', 'Color',[0.85 0.33 0.1], ...
        'MarkerFaceColor',[0.85 0.33 0.1], 'MarkerSize',5);
end
plot(est_poses(1,1), est_poses(1,2), 'g^', 'MarkerSize',10, 'MarkerFaceColor','g');
legend({'ground truth path','ground truth landmarks','estimated path', ...
    'estimated landmarks','start'}, 'Location','best');
xlabel('x [m]'); ylabel('y [m]');
title(title_str);
xlim([-2 10]); ylim([-2 10]);
hold off;
end

function plot_comparison(gt_poses, full_poses, pose_only_poses)
hold on; axis equal; grid on;
plot(gt_poses(:,1), gt_poses(:,2), '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.5);
plot(full_poses(:,1), full_poses(:,2), '-', 'Color',[0 0.45 0.74], 'LineWidth',2.5);
plot(pose_only_poses(:,1), pose_only_poses(:,2), ':', 'Color',[0.85 0.33 0.1], 'LineWidth',2.5);
legend({'ground truth','full pose + landmark graph','pose-only (odom + loop closure)'}, ...
    'Location','best');
xlabel('x [m]'); ylabel('y [m]');
title('Effect of landmark constraints on trajectory correction');
hold off;
end