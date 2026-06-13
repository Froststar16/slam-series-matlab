%% Module 10 - sanity check #2 (adds Pose Graph SLAM + RPE, all 4 methods)
% Run this from inside the 10_benchmark_capstone folder.

clear; clc; close all;

data = generate_benchmark_data();
fprintf('Poses: %d, Landmarks: %d, Loop closures: %d, Observations: %d\n', ...
    size(data.gt_poses,1), size(data.gt_landmarks,1), ...
    numel(data.loop_closures), size(data.observations,1));

result_01 = run_ekf_slam(data, 60);
result_02 = run_ekf_slam(data, Inf);
result_03 = run_fastslam(data, 50);
result_09 = run_pose_graph_slam(data);

results = {result_01, result_02, result_03, result_09};
labels  = {'EKF (01)', 'EKF+LC (02)', 'FastSLAM (03)', 'PoseGraph (09)'};

fprintf('\n%-16s %8s %8s %8s %10s %8s\n', 'Method','ATE[m]','RPEt[m]','RPEr[deg]','Landmarks','Time[s]');
for i = 1:numel(results)
    r = results{i};
    ate = compute_ate(r.poses(:,1:2), data.gt_poses(:,1:2));
    [rpe_t, rpe_r] = compute_rpe(r.poses, data.gt_poses, 10);
    fprintf('%-16s %8.3f %8.3f %8.3f %10d %8.3f\n', ...
        labels{i}, ate, rpe_t, rad2deg(rpe_r), r.num_landmarks, r.runtime_s);
end

figure; hold on; axis equal; grid on;
plot(data.gt_poses(:,1), data.gt_poses(:,2), '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.5);
colors = {'r','b','m',[0 0.6 0.2]};
for i = 1:numel(results)
    plot(results{i}.poses(:,1), results{i}.poses(:,2), '-', 'Color', colors{i});
end
legend(['ground truth', labels]);
title('Module 10 sanity check - all 4 methods');