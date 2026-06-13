function rmse = compute_ate(est_xy, gt_xy)
%COMPUTE_ATE  Root-mean-square Absolute Trajectory Error (translation only).
%
%   rmse = COMPUTE_ATE(est_xy, gt_xy) where both inputs are Nx2 [x y]
%   arrays of the same length, expressed in the same frame (here pose 1
%   is anchored at the origin for both, so no alignment step is needed).

errors = sqrt(sum((est_xy - gt_xy).^2, 2));
rmse = sqrt(mean(errors.^2));

end