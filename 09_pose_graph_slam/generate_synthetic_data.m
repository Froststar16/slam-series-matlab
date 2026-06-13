function graph = generate_synthetic_data()
%GENERATE_SYNTHETIC_DATA Build a synthetic pose+landmark graph for Module 09.
%
%   graph = GENERATE_SYNTHETIC_DATA() returns a struct with fields:
%       gt_poses        Nx3   ground truth [x y theta] for each pose node
%       gt_landmarks    Mx2   ground truth [x y] for each landmark node
%       poses_init      Nx3   drifted initial pose estimates (dead-reckoned)
%       landmarks_init  Mx2   drifted initial landmark estimates
%       edges           struct array, one entry per constraint:
%                           .type   'odom' | 'loop' | 'obs'
%                           .i      index of first node (always a pose)
%                           .j      index of second node (pose for odom/loop,
%                                   landmark for obs)
%                           .z      measurement
%                                     - 'odom'/'loop' -> [dx dy dtheta]
%                                     - 'obs'          -> [dx dy] in robot frame
%                           .omega  information matrix (3x3 or 2x2)
%
%   The trajectory is a roughly-square 8m loop. Odometry has a small
%   systematic bias (in both heading and forward distance) so the
%   dead-reckoned estimate fails to close the loop -- this is the
%   "drift" that pose graph optimization will correct. Landmarks are
%   observed whenever they fall within sensor range, and a single
%   loop-closure constraint is injected near the end of the trajectory
%   where the robot genuinely revisits its starting area.

rng(7);  % reproducible

%% ---- 1. Ground-truth trajectory: a roughly-square 8m loop ----
side_len = 8;     % metres
step_len = 0.15;  % metres per pose
corners = [0 0; side_len 0; side_len side_len; 0 side_len; 0 0];

gt_poses = [];
for c = 1:4
    p0 = corners(c,:);
    p1 = corners(c+1,:);
    seg_len = norm(p1 - p0);
    n_steps = round(seg_len / step_len);
    heading = atan2(p1(2)-p0(2), p1(1)-p0(1));
    for s = 0:n_steps-1
        pt = p0 + (p1-p0) * (s / n_steps);
        gt_poses = [gt_poses; pt(1) pt(2) heading]; %#ok<AGROW>
    end
end
N = size(gt_poses,1);

%% ---- 2. Ground-truth landmarks scattered around the loop ----
M = 18;
gt_landmarks = [ -1 + (side_len+2)*rand(M,1), -1 + (side_len+2)*rand(M,1) ];

%% ---- 3. Simulate noisy odometry (with a small systematic bias -> drift) ----
odom_sigma  = [0.01 0.01 0.01];   % [dx dy dtheta] noise std
bias_dtheta = 0.0025;             % rad/step heading bias -> loop fails to close
bias_scale  = 1.01;               % 1% over-estimate of forward distance

edges = struct('type',{},'i',{},'j',{},'z',{},'omega',{});

poses_init = zeros(N,3);
poses_init(1,:) = gt_poses(1,:);   % anchor: pose 1 is known exactly

odom_omega = diag(1 ./ (odom_sigma.^2));

for k = 1:N-1
    dT = relative_pose(gt_poses(k,:), gt_poses(k+1,:));
    z_meas = dT + odom_sigma .* randn(1,3);
    z_meas(1) = z_meas(1) * bias_scale;
    z_meas(3) = z_meas(3) + bias_dtheta;

    edges(end+1) = struct('type','odom','i',k,'j',k+1,'z',z_meas,'omega',odom_omega); %#ok<AGROW>

    poses_init(k+1,:) = compose_pose(poses_init(k,:), z_meas);
end

%% ---- 4. Simulate landmark observations (range-limited) ----
obs_sigma    = [0.05 0.05];
sensor_range = 3.0;
obs_omega    = diag(1 ./ (obs_sigma.^2));

landmarks_init = nan(M,2);
for k = 1:N
    for l = 1:M
        d = norm(gt_landmarks(l,:) - gt_poses(k,1:2));
        if d <= sensor_range
            z_true = global_to_local(gt_poses(k,:), gt_landmarks(l,:));
            z_meas = z_true + obs_sigma .* randn(1,2);
            edges(end+1) = struct('type','obs','i',k,'j',l,'z',z_meas,'omega',obs_omega); %#ok<AGROW>

            if any(isnan(landmarks_init(l,:)))
                landmarks_init(l,:) = local_to_global(poses_init(k,:), z_meas);
            end
        end
    end
end

% any landmark never observed -> seed near ground truth (shouldn't normally happen)
unseen = any(isnan(landmarks_init),2);
if any(unseen)
    landmarks_init(unseen,:) = gt_landmarks(unseen,:) + 0.5*randn(sum(unseen),2);
end

%% ---- 5. Loop closure: detect genuine revisits from ground truth ----
loop_sigma = [0.03 0.03 0.01];
loop_omega = diag(1 ./ (loop_sigma.^2));
min_gap    = 30;   % ignore trivially-close neighbours
lc_radius  = 0.3;  % metres

for k = 1:N
    for j = 1:(k-min_gap)
        if j < 1, continue; end
        if norm(gt_poses(k,1:2) - gt_poses(j,1:2)) < lc_radius
            z_true = relative_pose(gt_poses(j,:), gt_poses(k,:));
            z_meas = z_true + loop_sigma .* randn(1,3);
            edges(end+1) = struct('type','loop','i',j,'j',k,'z',z_meas,'omega',loop_omega); %#ok<AGROW>
            break  % one loop closure per pose is plenty
        end
    end
end

graph.gt_poses       = gt_poses;
graph.gt_landmarks   = gt_landmarks;
graph.poses_init     = poses_init;
graph.landmarks_init = landmarks_init;
graph.edges          = edges;

end

%% ---------- local helper functions ----------

function z = relative_pose(pa, pb)
% Pose of b expressed in frame a -> [dx dy dtheta]
c = cos(pa(3)); s = sin(pa(3));
d = pb(1:2) - pa(1:2);
z = [ c*d(1)+s*d(2), -s*d(1)+c*d(2), wrap_angle(pb(3)-pa(3)) ];
end

function pb = compose_pose(pa, z)
% pose b = pose a "plus" relative measurement z (inverse of relative_pose)
c = cos(pa(3)); s = sin(pa(3));
dx = c*z(1) - s*z(2);
dy = s*z(1) + c*z(2);
pb = [ pa(1)+dx, pa(2)+dy, wrap_angle(pa(3)+z(3)) ];
end

function zl = global_to_local(p, pt)
c = cos(p(3)); s = sin(p(3));
d = pt - p(1:2);
zl = [ c*d(1)+s*d(2), -s*d(1)+c*d(2) ];
end

function pt = local_to_global(p, zl)
c = cos(p(3)); s = sin(p(3));
pt = p(1:2) + [ c*zl(1)-s*zl(2), s*zl(1)+c*zl(2) ];
end

function a = wrap_angle(a)
a = mod(a+pi, 2*pi) - pi;
end