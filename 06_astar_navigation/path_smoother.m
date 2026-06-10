function smooth = path_smoother(raw_path, grid, res, N)
% PATH_SMOOTHER  Two-stage path smoother: shortcut pruning + spline.
%
%   smooth = path_smoother(raw_path, grid, res, N)
%
%   Stage 1 — Greedy shortcutting:
%     Walk from the start. For each waypoint, try to jump as far ahead
%     as possible while the straight-line chord stays in free space.
%     Line-of-sight check uses Bresenham-style grid sampling.
%
%   Stage 2 — Cubic spline interpolation:
%     Fit a parametric cubic spline through the pruned waypoints and
%     re-sample at a fixed arc-length density. Each re-sampled point is
%     clamped back to free space if the spline dips into an obstacle.
%
%   Inputs:
%     raw_path  — Px2 world-coord waypoints [x, y]
%     grid      — NxN binary occupancy/inflated grid (1=obstacle)
%     res       — metres per cell
%     N         — grid side length (cells)
%
%   Output:
%     smooth    — Qx2 world-coord smoothed waypoints

    % ── Stage 1: greedy shortcutting ─────────────────────────
    pruned = raw_path(1,:);
    i = 1;
    while i < size(raw_path,1)
        % Try to reach as far ahead as possible
        j = size(raw_path,1);
        while j > i+1
            if los_clear(raw_path(i,:), raw_path(j,:), grid, res, N)
                break;
            end
            j = j - 1;
        end
        pruned = [pruned; raw_path(j,:)]; %#ok
        i = j;
    end
    % Always include the exact goal
    if ~isequal(pruned(end,:), raw_path(end,:))
        pruned = [pruned; raw_path(end,:)];
    end

    if size(pruned,1) < 3
        % Not enough points for spline — return pruned as-is
        smooth = pruned;
        return;
    end

    % ── Stage 2: parametric cubic spline ─────────────────────
    % Parametrize by cumulative chord length
    diffs  = diff(pruned);
    dists  = sqrt(sum(diffs.^2, 2));
    t_knot = [0; cumsum(dists)];
    total  = t_knot(end);

    % Resample density: one point every ~2 cells
    n_out  = max(50, round(total / (res*2)));
    t_fine = linspace(0, total, n_out)';

    % Fit independent cubic splines for x and y
    pp_x = spline(t_knot, pruned(:,1));
    pp_y = spline(t_knot, pruned(:,2));
    xs   = ppval(pp_x, t_fine);
    ys   = ppval(pp_y, t_fine);

    smooth = [xs, ys];

    % ── Clamp spline points that landed in obstacles ──────────
    for k = 1:size(smooth,1)
        ci    = world2cell(smooth(k,1), smooth(k,2), res, N);
        ci_r  = N - ci(2) + 1;
        ci_c  = ci(1);
        ci_r  = max(1, min(N, ci_r));
        ci_c  = max(1, min(N, ci_c));
        if grid(ci_r, ci_c)
            % Snap to nearest pruned waypoint
            diffs2 = sum((pruned - smooth(k,:)).^2, 2);
            [~, nn] = min(diffs2);
            smooth(k,:) = pruned(nn,:);
        end
    end
end

% ── Line-of-sight check (Bresenham sampling) ─────────────────
function clear = los_clear(p1, p2, grid, res, N)
    n_samples = max(10, round(norm(p2-p1)/res * 2));
    clear = true;
    for s = 0:n_samples
        t  = s / n_samples;
        px = p1(1)*(1-t) + p2(1)*t;
        py = p1(2)*(1-t) + p2(2)*t;
        ci = world2cell(px, py, res, N);
        r  = N - ci(2) + 1;
        c  = ci(1);
        r  = max(1, min(N, r));
        c  = max(1, min(N, c));
        if grid(r, c)
            clear = false;
            return;
        end
    end
end

% ── World → cell conversion ───────────────────────────────────
function ci = world2cell(wx, wy, res, n)
    ci = [max(1,min(n, floor(wx/res)+1)), ...
          max(1,min(n, floor(wy/res)+1))];
end