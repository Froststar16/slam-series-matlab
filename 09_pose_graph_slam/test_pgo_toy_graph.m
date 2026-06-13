function test_pgo_toy_graph()
%TEST_PGO_TOY_GRAPH  Ground-truth recovery test for gauss_newton_pgo.
%
%   TEST_PGO_TOY_GRAPH() builds a tiny, noise-free pose+landmark graph
%   where every single edge is exactly consistent with a known ground
%   truth (an 8-pose square loop with a loop-closure edge back to pose 1,
%   plus 3 landmarks each observed from two poses). Because every
%   constraint is simultaneously satisfiable, the true global optimum has
%   chi2 = 0 and IS the ground truth.
%
%   The initial estimate is the ground truth plus small random noise.
%   gauss_newton_pgo is run on this graph, and the result is checked
%   against ground truth to numerical precision.
%
%   This is the standard "ground truth recovery" sanity check used to
%   validate graph optimizers, and it has no Navigation Toolbox
%   dependency -- useful since optimizePoseGraph may not be available on
%   every install (see validate_with_pose_graph_toolbox.m).

rng(42);

%% ---- Ground truth: an 8-pose square loop (2m side, 1m steps) ----
gt_poses = [
    0 0  0     ;
    1 0  0     ;
    2 0  pi/2  ;
    2 1  pi/2  ;
    2 2  pi    ;
    1 2  pi    ;
    0 2 -pi/2  ;
    0 1 -pi/2  ;
];
N = size(gt_poses,1);

gt_landmarks = [1 1; 1.5 0.5; 0.5 1.5];
M = size(gt_landmarks,1);

omega_pose = diag([1e4 1e4 1e4]);
omega_obs  = diag([1e4 1e4]);

edges = struct('type',{},'i',{},'j',{},'z',{},'omega',{});

%% ---- Odometry edges: exact relative pose between consecutive nodes ----
for k = 1:N-1
    edges(end+1) = struct('type','odom','i',k,'j',k+1, ...
        'z', relative_pose(gt_poses(k,:), gt_poses(k+1,:)), 'omega', omega_pose); %#ok<AGROW>
end

%% ---- One loop-closure edge: pose N back to pose 1, exact ----
edges(end+1) = struct('type','loop','i',N,'j',1, ...
    'z', relative_pose(gt_poses(N,:), gt_poses(1,:)), 'omega', omega_pose); 

%% ---- Landmark observations: each landmark seen from two poses, exact ----
obs_pairs = [1 1; 5 1; 2 2; 4 2; 7 3; 3 3];  % [pose_idx landmark_idx]
for e = 1:size(obs_pairs,1)
    pidx = obs_pairs(e,1);
    lidx = obs_pairs(e,2);
    z = global_to_local(gt_poses(pidx,:), gt_landmarks(lidx,:));
    edges(end+1) = struct('type','obs','i',pidx,'j',lidx,'z',z,'omega',omega_obs); %#ok<AGROW>
end

%% ---- Perturbed initial estimate ----
poses_init = gt_poses + 0.05*[zeros(1,3); randn(N-1,3)];  % pose 1 stays exact (anchored)
landmarks_init = gt_landmarks + 0.1*randn(M,2);

%% ---- Run the from-scratch solver ----
result = gauss_newton_pgo(poses_init, landmarks_init, edges, 30, 1e-12);

pose_err     = max(abs(result.poses_final(:,1:2) - gt_poses(:,1:2)), [], 'all');
theta_err    = max(abs(wrap_angle(result.poses_final(:,3) - gt_poses(:,3))));
landmark_err = max(abs(result.landmarks_final - gt_landmarks), [], 'all');
chi2_final   = result.chi2_history(end);

fprintf('Ground-truth recovery test (gauss_newton_pgo)\n');
fprintf('  iterations:               %d\n', numel(result.poses_history)-1);
fprintf('  final chi2:               %.3e   (expect ~0)\n', chi2_final);
fprintf('  max pose position error:  %.3e m\n', pose_err);
fprintf('  max pose heading error:   %.3e rad\n', theta_err);
fprintf('  max landmark error:       %.3e m\n', landmark_err);

tol_check = 1e-4;
if chi2_final < 1e-6 && pose_err < tol_check && theta_err < tol_check && landmark_err < tol_check
    fprintf('  PASS -- solver recovers ground truth to within %.0e\n', tol_check);
else
    fprintf('  FAIL -- check Jacobian signs in gauss_newton_pgo.m\n');
end

end

%% ---------- local helper functions (mirrors generate_synthetic_data.m) ----------

function z = relative_pose(pa, pb)
c = cos(pa(3)); s = sin(pa(3));
d = pb(1:2) - pa(1:2);
z = [ c*d(1)+s*d(2), -s*d(1)+c*d(2), wrap_angle(pb(3)-pa(3)) ];
end

function zl = global_to_local(p, pt)
c = cos(p(3)); s = sin(p(3));
d = pt - p(1:2);
zl = [ c*d(1)+s*d(2), -s*d(1)+c*d(2) ];
end

function a = wrap_angle(a)
a = mod(a+pi, 2*pi) - pi;
end