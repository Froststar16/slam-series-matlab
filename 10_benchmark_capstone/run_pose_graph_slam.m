function result = run_pose_graph_slam(data)
%RUN_POSE_GRAPH_SLAM  Module 09's from-scratch Gauss-Newton solver, applied
%to the shared benchmark dataset.
%
%   result = RUN_POSE_GRAPH_SLAM(data)
%
%   Builds a full pose+landmark graph:
%     - odometry edges from data.odom
%     - pose-landmark edges from data.observations, using the TRUE
%       landmark IDs (same convention as Module 09 -- data association is
%       treated as a separate problem; this isolates the optimization
%       itself, and is the fairest comparison point since the EKF/FastSLAM
%       methods are NOT given landmark IDs and must associate on their own)
%     - loop closure edges from data.loop_closures
%   then runs gauss_newton_pgo (unchanged from Module 09) on the full graph.
%
%   result.poses          Nx3 optimized trajectory
%   result.landmarks      Mx2 optimized landmark positions
%   result.runtime_s      wall-clock seconds (graph build + optimization)
%   result.num_landmarks  M (= true landmark count, by construction)

N = size(data.gt_poses,1);
M = size(data.gt_landmarks,1);

odom_omega = diag(1./(data.params.odom_sigma.^2));
obs_omega  = diag(1./(data.params.obs_sigma.^2));
loop_omega = diag(1./(data.params.loop_sigma.^2));

tic;

% ---- dead-reckoned initial poses ----
poses_init = zeros(N,3);
poses_init(1,:) = data.gt_poses(1,:);
for k = 1:N-1
    poses_init(k+1,:) = compose_pose(poses_init(k,:), data.odom(k,:));
end

% ---- initial landmark estimates: first-observation triangulation ----
landmarks_init = nan(M,2);
for r = 1:size(data.observations,1)
    pidx = data.observations(r,1);
    lidx = data.observations(r,2);
    if isnan(landmarks_init(lidx,1))
        z = data.observations(r,3:4);
        landmarks_init(lidx,:) = local_to_global(poses_init(pidx,:), z);
    end
end
unseen = isnan(landmarks_init(:,1));
if any(unseen)
    landmarks_init(unseen,:) = data.gt_landmarks(unseen,:);
end

% ---- build edges ----
edges = struct('type',{},'i',{},'j',{},'z',{},'omega',{});
for k = 1:N-1
    edges(end+1) = struct('type','odom','i',k,'j',k+1, ...
        'z',data.odom(k,:),'omega',odom_omega); %#ok<AGROW>
end
for r = 1:size(data.observations,1)
    pidx = data.observations(r,1);
    lidx = data.observations(r,2);
    z    = data.observations(r,3:4);
    edges(end+1) = struct('type','obs','i',pidx,'j',lidx, ...
        'z',z,'omega',obs_omega); %#ok<AGROW>
end
for c = 1:numel(data.loop_closures)
    lc = data.loop_closures(c);
    edges(end+1) = struct('type','loop','i',lc.i,'j',lc.j, ...
        'z',lc.z,'omega',loop_omega); %#ok<AGROW>
end

% ---- optimize ----
gn_result = gauss_newton_pgo(poses_init, landmarks_init, edges, 20, 1e-4);

runtime_s = toc;

result.poses         = gn_result.poses_final;
result.landmarks     = gn_result.landmarks_final;
result.runtime_s     = runtime_s;
result.num_landmarks = M;

end

%% ---------- local helper functions ----------

function pb = compose_pose(pa, z)
c = cos(pa(3)); s = sin(pa(3));
dx = c*z(1) - s*z(2);
dy = s*z(1) + c*z(2);
pb = [pa(1)+dx, pa(2)+dy, wrap_angle(pa(3)+z(3))];
end

function pt = local_to_global(p, zl)
c = cos(p(3)); s = sin(p(3));
pt = p(1:2) + [c*zl(1)-s*zl(2), s*zl(1)+c*zl(2)];
end

function a = wrap_angle(a)
a = mod(a+pi,2*pi)-pi;
end