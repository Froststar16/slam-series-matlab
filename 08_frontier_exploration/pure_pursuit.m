function [x_new, y_new, th_new] = pure_pursuit(x, y, th, path_xy, ...
                                                 lookahead, v_max, dt)
% PURE_PURSUIT  One step of pure-pursuit path tracking.
%
%   [x_new, y_new, th_new] = pure_pursuit(x, y, th, path_xy,
%                                          lookahead, v_max, dt)
%
%   Inputs:
%     x, y, th   – current pose (world coords, radians)
%     path_xy    – Kx2 array of [x y] waypoints
%     lookahead  – lookahead distance (metres)
%     v_max      – maximum forward speed (m/s)
%     dt         – time step (s)
%
%   Outputs:
%     x_new, y_new, th_new – updated pose
%
%   Speed scales down with curvature so the robot slows at corners.
%   If path_xy has only one point the robot drives straight toward it.

if isempty(path_xy), x_new=x; y_new=y; th_new=th; return; end

% ── find lookahead point ─────────────────────────────────────────────────
pos = [x, y];
K   = size(path_xy, 1);

% find the furthest waypoint within lookahead distance that is ahead of us
target = path_xy(end, :);   % default: final waypoint

for k = 1:K-1
    % parametric point on segment k→k+1
    A = path_xy(k, :);
    B = path_xy(k+1, :);
    AB = B - A;
    AP = pos - A;
    t  = dot(AP, AB) / (dot(AB, AB) + 1e-9);
    t  = max(0, min(1, t));
    closest = A + t * AB;
    d = norm(pos - closest);
    if d <= lookahead
        target = B;   % aim past this segment
    end
end

% ── compute steering ─────────────────────────────────────────────────────
dx   = target(1) - x;
dy   = target(2) - y;
dist = sqrt(dx^2 + dy^2);

% desired heading
th_des = atan2(dy, dx);
err    = atan2(sin(th_des - th), cos(th_des - th));   % wrapped

% curvature → speed scaling  (high curvature = slow down)
curvature = abs(err) / (lookahead + 1e-6);
v = v_max * max(0.3, 1 - 1.5 * curvature);
v = min(v, dist / dt);   % don't overshoot final waypoint

% max turn rate 2 rad/s
omega = max(-2, min(2, 3.0 * err));

% ── integrate unicycle model ─────────────────────────────────────────────
th_new = th + omega * dt;
th_new = atan2(sin(th_new), cos(th_new));
x_new  = x + v * cos(th_new) * dt;
y_new  = y + v * sin(th_new) * dt;

end