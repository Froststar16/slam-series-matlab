function [v, w, lookahead_pt] = pure_pursuit(pose, path, L, v_max)
% PURE_PURSUIT  Pure pursuit path tracking controller.
%
%   [v, w, lookahead_pt] = pure_pursuit(pose, path, L, v_max)
%
%   Finds the lookahead point on the path at distance L ahead of the
%   robot, then computes the curvature needed to reach it. Returns
%   linear velocity v and angular velocity w.
%
%   Inputs:
%     pose   — [x, y, theta] robot pose
%     path   — Nx2 world-coord waypoints
%     L      — lookahead distance (metres)
%     v_max  — maximum linear speed (m/s)
%
%   Outputs:
%     v            — linear velocity command (m/s)
%     w            — angular velocity command (rad/s)
%     lookahead_pt — [x, y] of the lookahead point (for visualisation)
%
%   Algorithm:
%     1. Find the closest point on the path to the robot.
%     2. Walk forward along the path until cumulative distance >= L.
%     3. Interpolate the exact lookahead point.
%     4. Compute heading error and curvature → (v, w).

    rx = pose(1);  ry = pose(2);  rth = pose(3);
    n  = size(path, 1);

    % ── Step 1: find closest path point ──────────────────────
    dists = sqrt((path(:,1)-rx).^2 + (path(:,2)-ry).^2);
    [~, closest] = min(dists);

    % ── Step 2: walk forward to find lookahead point ──────────
    arc = 0;
    lookahead_pt = path(end,:);   % default to goal if path is short
    found = false;

    for i = closest : n-1
        seg_len = norm(path(i+1,:) - path(i,:));
        if arc + seg_len >= L
            % Interpolate within this segment
            t = (L - arc) / seg_len;
            lookahead_pt = path(i,:) + t * (path(i+1,:) - path(i,:));
            found = true;
            break;
        end
        arc = arc + seg_len;
    end

    if ~found
        lookahead_pt = path(end,:);
    end

    % ── Step 3: compute curvature ─────────────────────────────
    % Transform lookahead into robot frame
    dx   = lookahead_pt(1) - rx;
    dy   = lookahead_pt(2) - ry;
    % Actual distance to lookahead (may differ from L at path end)
    ld   = max(0.01, sqrt(dx^2 + dy^2));

    % x-component in robot frame (lateral offset)
    ex   = -sin(rth)*dx + cos(rth)*dy;   % lateral error

    % Pure pursuit curvature: kappa = 2*ex / ld^2
    kappa = 2 * ex / (ld^2);

    % ── Step 4: velocity commands ─────────────────────────────
    % Slow down when curvature is high (tight corners)
    v = v_max / (1 + 2.0*abs(kappa));
    v = max(0.05, min(v_max, v));
    w = v * kappa;

    % Clamp angular velocity
    w = max(-2.0, min(2.0, w));
end