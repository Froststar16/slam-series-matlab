%% Module 10 - sanity check #1 (data + EKF 01/02 + FastSLAM 03)
% Run this from inside the 10_benchmark_capstone folder.

clear; clc; close all;

data = generate_benchmark_data();
fprintf('Poses: %d, Landmarks: %d, Loop closures: %d, Observations: %d\n', ...
    size(data.gt_poses,1), size(data.gt_landmarks,1), ...
    numel(data.loop_closures), size(data.observations,1));

result_01 = run_ekf_slam(data, 60);     % Module 01: no cross-loop correction
result_02 = run_ekf_slam(data, Inf);    % Module 02: EKF + loop closure
result_03 = run_fastslam(data, 50);     % Module 03: FastSLAM, 50 particles

ate_01 = compute_ate(result_01.poses(:,1:2), data.gt_poses(:,1:2));
ate_02 = compute_ate(result_02.poses(:,1:2), data.gt_poses(:,1:2));
ate_03 = compute_ate(result_03.poses(:,1:2), data.gt_poses(:,1:2));

fprintf('EKF (01):        ATE=%.3fm, landmarks=%d, runtime=%.3fs\n', ate_01, result_01.num_landmarks, result_01.runtime_s);
fprintf('EKF+LC (02):     ATE=%.3fm, landmarks=%d, runtime=%.3fs\n', ate_02, result_02.num_landmarks, result_02.runtime_s);
fprintf('FastSLAM (03):   ATE=%.3fm, landmarks=%d, runtime=%.3fs\n', ate_03, result_03.num_landmarks, result_03.runtime_s);

figure; hold on; axis equal; grid on;
plot(data.gt_poses(:,1), data.gt_poses(:,2), '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1.5);
plot(result_01.poses(:,1), result_01.poses(:,2), 'r-');
plot(result_02.poses(:,1), result_02.poses(:,2), 'b-');
plot(result_03.poses(:,1), result_03.poses(:,2), 'm-');
legend('ground truth','EKF (01)','EKF+LC (02)','FastSLAM (03)');
title('Module 10 sanity check');