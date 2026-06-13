%% Module 10 -- Benchmark Capstone
%
% Head-to-head comparison of four SLAM approaches from this series, all
% run on ONE shared synthetic dataset (a figure-eight trajectory with
% multiple loop closures):
%
%   01  EKF SLAM              -- no cross-loop landmark re-association
%   02  EKF + Loop Closure     -- same EKF, unbounded re-association window
%   03  FastSLAM               -- particle filter, per-particle landmark EKFs
%   09  Pose Graph SLAM         -- from-scratch Gauss-Newton, full graph
%
% Metrics: ATE (global trajectory accuracy), RPE (local drift rate,
% translation + rotation), landmark count, and wall-clock runtime.
%
% A bonus section sweeps FastSLAM's particle count (50 vs 200) to turn
% the single FastSLAM data point into an accuracy-vs-compute tradeoff.

clear; clc; close all;
RPE_DELTA = 10;   % pose-index window for RPE (~1.5m of travel)

%% 1. Shared dataset ----------------------------------------------------------------
data = generate_benchmark_data();
N = size(data.gt_poses,1);
fprintf('Benchmark dataset: %d poses, %d landmarks, %d loop closures, %d observations\n\n', ...
    N, size(data.gt_landmarks,1), numel(data.loop_closures), size(data.observations,1));

%% 2. Run all four methods -----------------------------------------------------------
fprintf('Running EKF SLAM (01)...\n');
result_01 = run_ekf_slam(data, 60);

fprintf('Running EKF + Loop Closure (02)...\n');
result_02 = run_ekf_slam(data, Inf);

fprintf('Running FastSLAM (03, 50 particles)...\n');
result_03 = run_fastslam(data, 50);

fprintf('Running Pose Graph SLAM (09)...\n');
result_09 = run_pose_graph_slam(data);

results = {result_01, result_02, result_03, result_09};
labels  = {'EKF (01)', 'EKF+LC (02)', 'FastSLAM (03)', 'PoseGraph (09)'};
colors  = {[0.85 0.10 0.10], [0.10 0.30 0.85], [0.85 0.10 0.85], [0.10 0.60 0.20]};

%% 3. Metrics table -------------------------------------------------------------------
n_methods = numel(results);
ate_vals  = zeros(1,n_methods);
rpet_vals = zeros(1,n_methods);
rper_vals = zeros(1,n_methods);
lm_vals   = zeros(1,n_methods);
time_vals = zeros(1,n_methods);

fprintf('\n%-16s %8s %9s %10s %10s %8s\n', 'Method','ATE[m]','RPEt[m]','RPEr[deg]','Landmarks','Time[s]');
fprintf('%s\n', repmat('-',1,64));
for i = 1:n_methods
    r = results{i};
    ate_vals(i)  = compute_ate(r.poses(:,1:2), data.gt_poses(:,1:2));
    [rt, rr]     = compute_rpe(r.poses, data.gt_poses, RPE_DELTA);
    rpet_vals(i) = rt;
    rper_vals(i) = rad2deg(rr);
    lm_vals(i)   = r.num_landmarks;
    time_vals(i) = r.runtime_s;
    fprintf('%-16s %8.3f %9.3f %10.3f %10d %8.3f\n', ...
        labels{i}, ate_vals(i), rpet_vals(i), rper_vals(i), lm_vals(i), time_vals(i));
end

%% 4. Trajectory overlay ----------------------------------------------------------------
fig = figure('Color','w','Position',[100 100 800 800]);
hold on; axis equal; grid on;
plot(data.gt_poses(:,1), data.gt_poses(:,2), '--', 'Color',[0.5 0.5 0.5], 'LineWidth',2);
legend_labels = {'ground truth'};
for i = 1:n_methods
    plot(results{i}.poses(:,1), results{i}.poses(:,2), '-', 'Color', colors{i}, 'LineWidth',1.5);
    legend_labels{end+1} = labels{i}; %#ok<SAGROW>
end
legend(legend_labels, 'Location','southoutside', 'Orientation','horizontal');
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory comparison: all methods vs ground truth');
exportgraphics(fig, 'trajectory_comparison.png');

%% 5. ATE bar chart ------------------------------------------------------------------------
figure('Color','w');
b = bar(ate_vals, 'FaceColor','flat');
for i = 1:n_methods, b.CData(i,:) = colors{i}; end
xticks(1:n_methods); xticklabels(labels); xtickangle(15);
ylabel('ATE (RMSE) [m]');
title('Absolute Trajectory Error');
grid on;
for i = 1:n_methods
    text(i, ate_vals(i)+0.04*max(ate_vals), sprintf('%.3f', ate_vals(i)), ...
        'HorizontalAlignment','center');
end
exportgraphics(gcf, 'ate_comparison.png');

%% 6. RPE bar charts (translation + rotation side by side) ---------------------------------
figure('Color','w','Position',[100 100 900 420]);

subplot(1,2,1);
b = bar(rpet_vals, 'FaceColor','flat');
for i = 1:n_methods, b.CData(i,:) = colors{i}; end
xticks(1:n_methods); xticklabels(labels); xtickangle(15);
ylabel('RPE translation (RMSE) [m]');
title(sprintf('RPE - translation (\\Delta=%d poses)', RPE_DELTA));
grid on;

subplot(1,2,2);
b = bar(rper_vals, 'FaceColor','flat');
for i = 1:n_methods, b.CData(i,:) = colors{i}; end
xticks(1:n_methods); xticklabels(labels); xtickangle(15);
ylabel('RPE rotation (RMSE) [deg]');
title(sprintf('RPE - rotation (\\Delta=%d poses)', RPE_DELTA));
grid on;

exportgraphics(gcf, 'rpe_comparison.png');

%% 7. Runtime bar chart (log scale) ---------------------------------------------------------
figure('Color','w');
b = bar(time_vals, 'FaceColor','flat');
for i = 1:n_methods, b.CData(i,:) = colors{i}; end
set(gca,'YScale','log');
xticks(1:n_methods); xticklabels(labels); xtickangle(15);
ylabel('Runtime [s] (log scale)');
title('Computational cost');
grid on;
for i = 1:n_methods
    text(i, time_vals(i)*1.3, sprintf('%.2fs', time_vals(i)), ...
        'HorizontalAlignment','center');
end
exportgraphics(gcf, 'runtime_comparison.png');

%% 8. Bonus: FastSLAM particle sweep (50 vs 200) ---------------------------------------------
fprintf('\nFastSLAM particle sweep: 50 (already run) vs 200 (running now, this is the slow one)...\n');
result_03_200 = run_fastslam(data, 200);

ate_sweep  = [ate_vals(3), compute_ate(result_03_200.poses(:,1:2), data.gt_poses(:,1:2))];
[rt200, rr200] = compute_rpe(result_03_200.poses, data.gt_poses, RPE_DELTA);
rpet_sweep = [rpet_vals(3), rt200];
rper_sweep = [rper_vals(3), rad2deg(rr200)];
time_sweep = [time_vals(3), result_03_200.runtime_s];
lm_sweep   = [lm_vals(3), result_03_200.num_landmarks];

fprintf('\n%-16s %8s %9s %10s %10s %8s\n', 'FastSLAM','ATE[m]','RPEt[m]','RPEr[deg]','Landmarks','Time[s]');
fprintf('%s\n', repmat('-',1,64));
fprintf('%-16s %8.3f %9.3f %10.3f %10d %8.3f\n', '50 particles',  ate_sweep(1), rpet_sweep(1), rper_sweep(1), lm_sweep(1), time_sweep(1));
fprintf('%-16s %8.3f %9.3f %10.3f %10d %8.3f\n', '200 particles', ate_sweep(2), rpet_sweep(2), rper_sweep(2), lm_sweep(2), time_sweep(2));

figure('Color','w','Position',[100 100 1000 420]);

subplot(1,3,1);
bar(ate_sweep, 'FaceColor',[0.85 0.10 0.85]);
xticks(1:2); xticklabels({'50','200'});
ylabel('ATE [m]'); title('FastSLAM ATE vs particle count'); grid on;
for i=1:2, text(i, ate_sweep(i)+0.05, sprintf('%.3f',ate_sweep(i)), 'HorizontalAlignment','center'); end

subplot(1,3,2);
bar(rpet_sweep, 'FaceColor',[0.85 0.10 0.85]);
xticks(1:2); xticklabels({'50','200'});
ylabel('RPE trans [m]'); title('FastSLAM RPE vs particle count'); grid on;

subplot(1,3,3);
bar(time_sweep, 'FaceColor',[0.85 0.10 0.85]);
set(gca,'YScale','log');
xticks(1:2); xticklabels({'50','200'});
ylabel('Runtime [s] (log)'); title('FastSLAM runtime vs particle count'); grid on;
for i=1:2, text(i, time_sweep(i)*1.3, sprintf('%.1fs',time_sweep(i)), 'HorizontalAlignment','center'); end

sgtitle('FastSLAM particle-count sweep');
exportgraphics(gcf, 'fastslam_particle_sweep.png');

% trajectory comparison for the sweep
figure('Color','w','Position',[100 100 700 700]);
hold on; axis equal; grid on;
plot(data.gt_poses(:,1), data.gt_poses(:,2), '--', 'Color',[0.5 0.5 0.5], 'LineWidth',2);
plot(result_03.poses(:,1), result_03.poses(:,2), '-', 'Color',[0.85 0.4 0.85], 'LineWidth',1.5);
plot(result_03_200.poses(:,1), result_03_200.poses(:,2), '-', 'Color',[0.5 0.0 0.5], 'LineWidth',1.5);
legend('ground truth','FastSLAM (50)','FastSLAM (200)','Location','southoutside','Orientation','horizontal');
title('FastSLAM trajectory: 50 vs 200 particles');
exportgraphics(gcf, 'fastslam_sweep_trajectories.png');

%% 9. Summary --------------------------------------------------------------------------------
fprintf('\nDone. Outputs:\n');
fprintf('  trajectory_comparison.png\n');
fprintf('  ate_comparison.png\n');
fprintf('  rpe_comparison.png\n');
fprintf('  runtime_comparison.png\n');
fprintf('  fastslam_particle_sweep.png\n');
fprintf('  fastslam_sweep_trajectories.png\n');

[~, best_ate_i]  = min(ate_vals);
[~, best_rpe_i]  = min(rpet_vals);
[~, best_time_i] = min(time_vals);
fprintf('\nBest ATE:     %s (%.3f m)\n', labels{best_ate_i}, ate_vals(best_ate_i));
fprintf('Best RPE:     %s (%.3f m)\n', labels{best_rpe_i}, rpet_vals(best_rpe_i));
fprintf('Fastest:      %s (%.3f s)\n', labels{best_time_i}, time_vals(best_time_i));