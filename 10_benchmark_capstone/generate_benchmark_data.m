function data = generate_benchmark_data()
%GENERATE_BENCHMARK_DATA  Shared figure-eight benchmark scenario for Module 10.
%
%   data = GENERATE_BENCHMARK_DATA() builds ONE synthetic scenario that all
%   four methods (EKF SLAM, EKF + Loop Closure, FastSLAM, Pose Graph SLAM)
%   run on, so their results are directly comparable.
%
%   The trajectory is a figure-eight: two 8m square loops sharing the
%   origin as a junction. The robot passes through the origin three times
%   (start, end of loop A, end of loop B), giving multiple genuine
%   revisit/loop-closure opportunities -- more than Module 09's single
%   loop closure. Landmarks are split across both loop regions with
%   deliberate overlap near the origin, so loop B re-observes some of
%   loop A's landmarks.
%
%   Returns a struct:
%     gt_poses       Nx3   ground truth [x y theta]
%     gt_landmarks   Mx2   ground truth landmark positions
%     odom           (N-1)x3  noisy odometry [dx dy dtheta]; odom(k,:) is
%                    the motion from pose k to pose k+1, expressed in
%                    pose k's frame (same convention as Module 09)
%     observations   Px4   [pose_idx, landmark_id, dx_meas, dy_meas] --
%                    noisy landmark observations in the robot's frame.
%                    landmark_id is the TRUE landmark index, available
%                    here for evaluation and for the pose-graph method
%                    (which, as in Module 09, uses known correspondences).
%                    The EKF-based methods do NOT use landmark_id -- they
%                    perform their own data association on (dx,dy).
%     loop_closures  struct array .i .j .z -- ground-truth-derived loop
%                    closure constraints (i = earlier pose, j = later
%                    pose), same format as Module 09
%     params         struct of the noise parameters used to generate this
%                    data (odom_sigma, obs_sigma, loop_sigma, sensor_range)

rng(11);

%% ---- Ground truth: figure-eight (two 8m square loops sharing the origin) ----
step_len = 0.15;
loopA_corners = [0 0; 8 0; 8 8; 0 8; 0 0];
loopB_corners = [0 0; 0 -8; -8 -8; -8 0; 0 0];

poses_A = build_loop(loopA_corners, step_len);
poses_B = build_loop(loopB_corners, step_len);
gt_poses = [poses_A; poses_B];
N = size(gt_poses,1);

%% ---- Landmarks: one cluster per loop region, overlapping near the origin ----
M1 = 16;
M2 = 16;
landmarks_A = [-1 + 10*rand(M1,1), -1 + 10*rand(M1,1)];   % roughly [-1,9]^2
landmarks_B = [-9 + 10*rand(M2,1), -9 + 10*rand(M2,1)];   % roughly [-9,1]^2
gt_landmarks = [landmarks_A; landmarks_B];
M = size(gt_landmarks,1);

%% ---- Odometry: same systematic-bias drift model as Module 09 ----
odom_sigma  = [0.01 0.01 0.01];
bias_dtheta = 0.0025;
bias_scale  = 1.01;

odom = zeros(N-1,3);
for k = 1:N-1
    dT = relative_pose(gt_poses(k,:), gt_poses(k+1,:));
    z = dT + odom_sigma .* randn(1,3);
    z(1) = z(1) * bias_scale;
    z(3) = z(3) + bias_dtheta;
    odom(k,:) = z;
end

%% ---- Landmark observations: range-limited, noisy ----
obs_sigma    = [0.05 0.05];
sensor_range = 3.0;

observations = zeros(0,4);
for k = 1:N
    for l = 1:M
        if norm(gt_landmarks(l,:) - gt_poses(k,1:2)) <= sensor_range
            z_true = global_to_local(gt_poses(k,:), gt_landmarks(l,:));
            z_meas = z_true + obs_sigma .* randn(1,2);
            observations(end+1,:) = [k, l, z_meas]; %#ok<AGROW>
        end
    end
end

%% ---- Loop closures: ground-truth revisit detection (as in Module 09) ----
loop_sigma = [0.03 0.03 0.01];
min_gap    = 40;
lc_radius  = 0.3;

loop_closures = struct('i',{},'j',{},'z',{});
for k = 1:N
    for j = 1:(k-min_gap)
        if j < 1, continue; end
        if norm(gt_poses(k,1:2) - gt_poses(j,1:2)) < lc_radius
            z_true = relative_pose(gt_poses(j,:), gt_poses(k,:));
            z = z_true + loop_sigma .* randn(1,3);
            loop_closures(end+1) = struct('i',j,'j',k,'z',z); %#ok<AGROW>
            break
        end
    end
end

%% ---- Package ----
params.odom_sigma   = odom_sigma;
params.obs_sigma    = obs_sigma;
params.loop_sigma   = loop_sigma;
params.sensor_range = sensor_range;

data.gt_poses      = gt_poses;
data.gt_landmarks  = gt_landmarks;
data.odom          = odom;
data.observations  = observations;
data.loop_closures = loop_closures;
data.params        = params;

end

%% ---------- local helper functions ----------

function poses = build_loop(corners, step_len)
poses = [];
for c = 1:size(corners,1)-1
    p0 = corners(c,:);
    p1 = corners(c+1,:);
    seg_len = norm(p1-p0);
    n_steps = round(seg_len/step_len);
    heading = atan2(p1(2)-p0(2), p1(1)-p0(1));
    for s = 0:n_steps-1
        pt = p0 + (p1-p0)*(s/n_steps);
        poses = [poses; pt(1) pt(2) heading]; %#ok<AGROW>
    end
end
end

function z = relative_pose(pa, pb)
c = cos(pa(3)); s = sin(pa(3));
d = pb(1:2)-pa(1:2);
z = [c*d(1)+s*d(2), -s*d(1)+c*d(2), wrap_angle(pb(3)-pa(3))];
end

function zl = global_to_local(p, pt)
c = cos(p(3)); s = sin(p(3));
d = pt - p(1:2);
zl = [c*d(1)+s*d(2), -s*d(1)+c*d(2)];
end

function a = wrap_angle(a)
a = mod(a+pi,2*pi)-pi;
end