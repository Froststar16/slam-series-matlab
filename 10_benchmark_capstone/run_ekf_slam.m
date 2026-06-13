function result = run_ekf_slam(data, assoc_window)
%RUN_EKF_SLAM  EKF SLAM with a configurable landmark re-association window.
%
%   result = RUN_EKF_SLAM(data, assoc_window)
%
%   data           struct from generate_benchmark_data
%   assoc_window   maximum age (in poses) for a landmark to be considered
%                  as a re-association candidate.
%                    - Inf            -> "Module 02: EKF + Loop Closure".
%                                        Revisited landmarks (e.g. loop B
%                                        re-observing loop A's landmarks
%                                        near the origin) are
%                                        re-associated, and the EKF update
%                                        pulls the current pose back into
%                                        consistency with the earlier map.
%                    - finite (e.g. 60) -> "Module 01: EKF SLAM". Far
%                                        revisits are outside the window,
%                                        so re-observed landmarks become
%                                        NEW landmarks instead -- no
%                                        cross-loop correction.
%
%   Both modes run the IDENTICAL prediction/update math on the IDENTICAL
%   data; only the association-candidate filter differs. This isolates
%   the "does re-association on revisit happen" effect as a single knob.
%
%   result.poses          Nx3 estimated trajectory
%   result.landmarks      Kx2 estimated landmark positions (K landmarks created)
%   result.runtime_s      wall-clock seconds for the main loop
%   result.num_landmarks  K

N = size(data.gt_poses,1);
obs  = data.observations;          % [pose_idx, true_id, dx, dy]
odom = data.odom;

Q      = diag(data.params.odom_sigma.^2);
R_meas = diag(data.params.obs_sigma.^2);
GATE   = 9.21;   % chi-square 99% threshold for 2 DoF

% State vector: [x; y; theta; lx1; ly1; lx2; ly2; ...]
x = data.gt_poses(1,:)';   % pose 1 anchored exactly, as in Module 09
P = zeros(3,3);

landmark_first_seen = zeros(0,1);
poses_est = zeros(N,3);
poses_est(1,:) = x(1:3)';

tic;
for k = 1:N
    if k > 1
        u = odom(k-1,:);
        [x, P] = ekf_predict(x, P, u, Q);
        poses_est(k,:) = x(1:3)';
    end

    rows = obs(obs(:,1)==k, :);
    for r = 1:size(rows,1)
        z = rows(r,3:4);
        [x, P, landmark_first_seen] = ekf_update(x, P, z, R_meas, ...
            landmark_first_seen, k, assoc_window, GATE);
        poses_est(k,:) = x(1:3)';
    end
end
runtime_s = toc;

M_est = (numel(x)-3)/2;
landmarks_est = reshape(x(4:end), 2, M_est)';

result.poses         = poses_est;
result.landmarks     = landmarks_est;
result.runtime_s     = runtime_s;
result.num_landmarks = M_est;

end

%% ---------- local helper functions ----------

function [x, P] = ekf_predict(x, P, u, Q)
% Motion update: x_pose <- compose_pose(x_pose, u). Landmarks unaffected.
theta = x(3);
c = cos(theta); s = sin(theta);

dx_ = c*u(1) - s*u(2);
dy_ = s*u(1) + c*u(2);
x(1) = x(1) + dx_;
x(2) = x(2) + dy_;
x(3) = wrap_angle(x(3) + u(3));

n = numel(x);
F = eye(n);
F(1:3,1:3) = [1 0 (-s*u(1)-c*u(2)); 0 1 (c*u(1)-s*u(2)); 0 0 1];

Gu = [c -s 0; s c 0; 0 0 1];

P = F*P*F';
P(1:3,1:3) = P(1:3,1:3) + Gu*Q*Gu';
P = (P+P')/2;
end

function [x, P, landmark_first_seen] = ekf_update(x, P, z, R_meas, ...
    landmark_first_seen, k, assoc_window, GATE)
% Data association (nearest Mahalanobis distance, gated) followed by
% either a standard EKF landmark update or a new-landmark augmentation.

theta = x(3);
c = cos(theta); s = sin(theta);
RT = [c s; -s c];          % R(theta)'
dRT_dtheta = [-s c; -c -s];

M_est = (numel(x)-3)/2;

best_d2    = inf;
best_idx   = -1;
best_innov = [];
best_H     = [];
best_S     = [];

for l = 1:M_est
    if isfinite(assoc_window) && (k - landmark_first_seen(l)) > assoc_window
        continue
    end

    lm = x(3+2*l-1 : 3+2*l);
    d  = (lm - x(1:2));
    pred  = (RT*d)';
    innov = z - pred;

    Hpose = [-RT, (dRT_dtheta*d)];   % 2x3
    Hlm   = RT;                       % 2x2

    H = zeros(2, numel(x));
    H(:,1:3) = Hpose;
    H(:,3+2*l-1:3+2*l) = Hlm;

    S  = H*P*H' + R_meas;
    d2 = (innov/S)*innov';

    if d2 < best_d2
        best_d2    = d2;
        best_idx   = l;
        best_innov = innov;
        best_H     = H;
        best_S     = S;
    end
end

if best_idx > 0 && best_d2 < GATE
    % ---- re-associate with existing landmark ----
    K = (P*best_H')/best_S;
    x = x + K*best_innov';
    x(3) = wrap_angle(x(3));

    n = numel(x);
    P = (eye(n) - K*best_H)*P;
    P = (P+P')/2;
else
    % ---- new landmark: augment state and covariance ----
    R_global = [c -s; s c];   % rotates robot-frame z into the global frame
    lm_pos = x(1:2) + R_global*z';   % 2x1 column

    dR_dtheta = [-s -c; c -s];
    Jp = [1 0 (dR_dtheta(1,:)*z'); 0 1 (dR_dtheta(2,:)*z')];   % 2x3, d(lm)/d(pose)
    Jz = R_global;                                              % 2x2, d(lm)/d(z)

    P_cross = Jp*P(1:3,:);                       % 2 x n
    P_ll    = Jp*P(1:3,1:3)*Jp' + Jz*R_meas*Jz'; % 2x2

    x = [x; lm_pos];
    P = [P, P_cross'; P_cross, P_ll];

    landmark_first_seen(end+1,1) = k;
end
end

function a = wrap_angle(a)
a = mod(a+pi,2*pi)-pi;
end