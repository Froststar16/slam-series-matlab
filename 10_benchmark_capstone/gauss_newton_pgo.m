function result = gauss_newton_pgo(poses_init, landmarks_init, edges, max_iters, tol)
%GAUSS_NEWTON_PGO  From-scratch Gauss-Newton pose+landmark graph optimizer.
%
%   result = GAUSS_NEWTON_PGO(poses_init, landmarks_init, edges, max_iters, tol)
%
%   poses_init      Nx3  initial [x y theta] for each pose node
%   landmarks_init  Mx2  initial [x y] for each landmark node (can be 0x2)
%   edges           struct array from generate_synthetic_data:
%                       .type 'odom'|'loop'|'obs', .i, .j, .z, .omega
%   max_iters       maximum Gauss-Newton iterations
%   tol             convergence threshold on norm(dx)
%
%   result.poses_history       cell array, {iter+1} -> Nx3 pose estimates
%   result.landmarks_history   cell array, {iter+1} -> Mx2 landmark estimates
%   result.chi2_history        1x(iters+1) total weighted error per iteration
%   result.poses_final         Nx3   final pose estimates
%   result.landmarks_final     Mx2   final landmark estimates
%
%   State vector layout: [pose_1 (3); pose_2 (3); ...; pose_N (3);
%                          landmark_1 (2); ...; landmark_M (2)]
%   Pose 1 is anchored with a stiff prior to remove the 3-DOF gauge
%   freedom (the whole graph can otherwise be rigidly transformed
%   without changing any edge residual).

N = size(poses_init,1);
M = size(landmarks_init,1);
dim = 3*N + 2*M;

poses     = poses_init;
landmarks = landmarks_init;

poses_history     = {poses};
landmarks_history = {landmarks};
chi2_history       = compute_chi2(poses, landmarks, edges);

LAMBDA_PRIOR = 1e9;   % anchors pose 1, removes gauge freedom
LAMBDA_DAMP  = 1e-6;  % tiny LM-style damping for numerical stability

for iter = 1:max_iters
    I_idx = []; J_idx = []; V_val = [];
    b = zeros(dim, 1);

    for e = 1:numel(edges)
        edge = edges(e);
        switch edge.type
            case {'odom','loop'}
                idx_i = (3*(edge.i-1)+1):(3*edge.i);
                idx_j = (3*(edge.j-1)+1):(3*edge.j);

                [err, Ji, Jj] = pose_pose_error_jacobian(poses(edge.i,:), poses(edge.j,:), edge.z);

                Hii = Ji'*edge.omega*Ji;
                Hjj = Jj'*edge.omega*Jj;
                Hij = Ji'*edge.omega*Jj;

                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_i, idx_i, Hii);
                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_j, idx_j, Hjj);
                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_i, idx_j, Hij);
                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_j, idx_i, Hij');

                b(idx_i) = b(idx_i) + Ji'*edge.omega*err';
                b(idx_j) = b(idx_j) + Jj'*edge.omega*err';

            case 'obs'
                idx_p = (3*(edge.i-1)+1):(3*edge.i);
                idx_l = (3*N + 2*(edge.j-1)+1):(3*N + 2*edge.j);

                [err, Jp, Jl] = pose_landmark_error_jacobian(poses(edge.i,:), landmarks(edge.j,:), edge.z);

                Hpp = Jp'*edge.omega*Jp;
                Hll = Jl'*edge.omega*Jl;
                Hpl = Jp'*edge.omega*Jl;

                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_p, idx_p, Hpp);
                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_l, idx_l, Hll);
                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_p, idx_l, Hpl);
                [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, idx_l, idx_p, Hpl');

                b(idx_p) = b(idx_p) + Jp'*edge.omega*err';
                b(idx_l) = b(idx_l) + Jl'*edge.omega*err';
        end
    end

    % anchor pose 1 (removes 3-DOF gauge freedom)
    [I_idx,J_idx,V_val] = add_block(I_idx,J_idx,V_val, 1:3, 1:3, LAMBDA_PRIOR*eye(3));

    H = sparse(I_idx, J_idx, V_val, dim, dim) + LAMBDA_DAMP*speye(dim);

    dx = -(H \ b);

    for k = 1:N
        idx = (3*(k-1)+1):(3*k);
        poses(k,:) = poses(k,:) + dx(idx)';
        poses(k,3) = wrap_angle(poses(k,3));
    end
    for l = 1:M
        idx = (3*N + 2*(l-1)+1):(3*N + 2*l);
        landmarks(l,:) = landmarks(l,:) + dx(idx)';
    end

    poses_history{end+1}     = poses;     %#ok<AGROW>
    landmarks_history{end+1} = landmarks; %#ok<AGROW>
    chi2_history(end+1)      = compute_chi2(poses, landmarks, edges); %#ok<AGROW>

    if norm(dx) < tol
        break
    end
end

result.poses_history     = poses_history;
result.landmarks_history = landmarks_history;
result.chi2_history       = chi2_history;
result.poses_final        = poses;
result.landmarks_final    = landmarks;

end

%% ---------- local helper functions ----------

function [I,J,V] = add_block(I,J,V, rows, cols, block)
% Accumulate a dense block into (I,J,V) triplet lists for later sparse().
[rr,cc] = ndgrid(rows, cols);
I = [I; rr(:)];
J = [J; cc(:)];
V = [V; block(:)];
end

function chi2 = compute_chi2(poses, landmarks, edges)
chi2 = 0;
for e = 1:numel(edges)
    edge = edges(e);
    switch edge.type
        case {'odom','loop'}
            err = pose_pose_error_jacobian(poses(edge.i,:), poses(edge.j,:), edge.z);
        case 'obs'
            err = pose_landmark_error_jacobian(poses(edge.i,:), landmarks(edge.j,:), edge.z);
    end
    chi2 = chi2 + err*edge.omega*err';
end
end

function [err, Ji, Jj] = pose_pose_error_jacobian(pi, pj, z)
% Residual of a pose-pose (odometry / loop-closure) constraint.
% z = [dx dy dtheta] is the measured pose of j expressed in i's frame.
ci = cos(pi(3)); si = sin(pi(3));
Ri_T = [ ci si; -si ci ];   % R(theta_i)'

d = pj(1:2) - pi(1:2);
pred_t  = (Ri_T * d')';
pred_th = wrap_angle(pj(3) - pi(3));

err = [pred_t(1)-z(1), pred_t(2)-z(2), wrap_angle(pred_th - z(3))];

if nargout > 1
    dRiT_dtheta = [-si ci; -ci -si];
    dpred_t_dthetai = (dRiT_dtheta * d')';   % 1x2

    Ji = [ -Ri_T,             dpred_t_dthetai' ;
            0,  0,           -1               ];
    Jj = [  Ri_T,             [0;0]            ;
            0,  0,            1               ];
end
end

function [err, Jp, Jl] = pose_landmark_error_jacobian(p, l, z)
% Residual of a pose-landmark observation constraint.
% z = [dx dy] is the measured landmark position in the robot's frame.
c = cos(p(3)); s = sin(p(3));
Rp_T = [ c s; -s c ];   % R(theta_p)'

d = l - p(1:2);
pred = (Rp_T * d')';

err = pred - z;

if nargout > 1
    dRpT_dtheta = [-s c; -c -s];
    dpred_dtheta = (dRpT_dtheta * d')';   % 1x2

    Jp = [ -Rp_T, dpred_dtheta' ];   % 2x3
    Jl = Rp_T;                        % 2x2
end
end

function a = wrap_angle(a)
a = mod(a+pi, 2*pi) - pi;
end