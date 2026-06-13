function [rpe_trans, rpe_rot] = compute_rpe(est_poses, gt_poses, delta)
%COMPUTE_RPE  Relative Pose Error (RMSE) over a fixed pose-index delta.
%
%   [rpe_trans, rpe_rot] = COMPUTE_RPE(est_poses, gt_poses, delta)
%
%   For every pair of poses (i, i+delta), compares the relative transform
%   in the ESTIMATED trajectory against the relative transform in GROUND
%   TRUTH over that same window. rpe_trans is the RMSE of the
%   translational difference (metres); rpe_rot is the RMSE of the
%   rotational difference (radians).
%
%   Unlike ATE, RPE doesn't get "fixed" by a single large correction (e.g.
%   a loop closure snapping the whole trajectory back into place) -- it
%   measures local drift RATE over short windows, which is a useful
%   complement: a method can have good ATE (globally consistent) but
%   still have poor RPE (locally noisy/jittery), or vice versa.
%
%   Default delta = 10 poses (~1.5m of travel at this dataset's step size).

if nargin < 3, delta = 10; end

N = size(est_poses,1);
trans_errs = zeros(N-delta,1);
rot_errs   = zeros(N-delta,1);

for i = 1:(N-delta)
    rel_gt  = relative_pose(gt_poses(i,:),  gt_poses(i+delta,:));
    rel_est = relative_pose(est_poses(i,:), est_poses(i+delta,:));

    trans_errs(i) = norm(rel_est(1:2) - rel_gt(1:2));
    rot_errs(i)   = abs(wrap_angle(rel_est(3) - rel_gt(3)));
end

rpe_trans = sqrt(mean(trans_errs.^2));
rpe_rot   = sqrt(mean(rot_errs.^2));

end

%% ---------- local helper functions ----------

function z = relative_pose(pa, pb)
c = cos(pa(3)); s = sin(pa(3));
d = pb(1:2)-pa(1:2);
z = [c*d(1)+s*d(2), -s*d(1)+c*d(2), wrap_angle(pb(3)-pa(3))];
end

function a = wrap_angle(a)
a = mod(a+pi,2*pi)-pi;
end