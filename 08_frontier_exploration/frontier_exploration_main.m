% =========================================================================
%  MODULE 08 — FRONTIER-BASED AUTONOMOUS EXPLORATION
%  2D LiDAR SLAM Series  |  Froststar16
% =========================================================================
%
%  Robot autonomously explores an unknown environment by navigating to
%  frontiers — boundaries between known-free and unknown space.
%
%  Pose estimation: dead-reckoning (odometry) only.
%  Mapping:         log-odds occupancy grid via Bresenham ray tracing.
%  Planning:        8-connected A* on inflated grid + pure pursuit.
%  Frontier select: hybrid score = frontier_size / distance_to_centroid.
%  Termination:     no frontiers remain  OR  MAX_STEPS reached.
%
%  Why no EKF landmarks?
%    Walls are line features, not point landmarks. Using them as point
%    landmarks floods the EKF with hundreds of closely-spaced "new"
%    landmarks along every straight wall, saturating the budget in the
%    first room and making the O(n_lm) update inner loop extremely slow.
%    Dead-reckoning + a good occupancy grid is the right model here;
%    the grid itself provides all the structure needed for frontier
%    exploration and A* planning.
%
%  Dependencies (all in this folder):
%    frontier_detector.m    – frontier detection + clustering
%    astar_planner.m        – 8-connected A* path planner
%    pure_pursuit.m         – pure-pursuit path tracker
%    save_exploration_gif.m – R2024a-compatible GIF export
%
% =========================================================================

clear; close all; clc;
rng(42);

%% ── PARAMETERS ───────────────────────────────────────────────────────────

% Simulation
DT            = 0.1;       % time step (s)
MAX_STEPS     = 25000;     % step budget
V_ROBOT       = 0.22;      % nominal forward speed (m/s)

% Dead-reckoning noise (odometry)
Q_SIG_V       = 0.015;     % linear velocity noise std (m/s)
Q_SIG_W       = 0.02;      % angular velocity noise std (rad/s)

% LiDAR
LIDAR_RANGE   = 5.0;       % max range (m)
LIDAR_NBEAMS  = 60;        % number of beams
LIDAR_NOISE   = 0.04;      % range noise std (m)

% Occupancy grid
GRID_RES      = 0.15;      % metres per cell
GRID_SIZE     = 20.0;      % world extent (metres), square
GRID_ORIGIN   = [0, 0];    % world coords of grid bottom-left corner
LO_FREE       = -0.4;      % log-odds decrement (free)
LO_OCC        = 1.2;       % log-odds increment (occupied)
LO_MIN        = -5;
LO_MAX        = 5;
OCC_TH        = 0.6;       % log-odds above this → occupied for planner
INFLATE_R     = 2;         % obstacle inflation radius (cells)

% Frontier exploration
FRONTIER_INTERVAL = 40;    % recompute frontiers every N steps
MIN_FRONTIER_SIZE = 3;     % ignore tiny fragments (cells)
SCORE_DIST_MIN    = 0.5;   % floor on distance to avoid /0 (m)
GOAL_THRESH       = 0.50;  % distance to count as "reached goal" (m)
REPLAN_EVERY      = 20;    % check path validity every N steps
STUCK_LIMIT       = 40;    % steps without progress → pick new frontier
BLACKLIST_RADIUS  = 3;     % grid cells — blacklist zone around failed target
BLACKLIST_TTL     = 500;   % steps before a blacklisted cell expires

% Pure pursuit
LOOKAHEAD         = 0.45;  % lookahead distance (m)

% Visualisation
VIS_EVERY         = 5;     % redraw every N steps
GIF_EVERY         = 10;    % save GIF frame every N steps
GIF_ENABLED       = true;
GIF_FILE          = 'exploration.gif';
GIF_DELAY         = 0.08;

%% ── WORLD DEFINITION ─────────────────────────────────────────────────────
WALLS = [
    % outer boundary
     0,  0, 20,  0;
    20,  0, 20, 20;
    20, 20,  0, 20;
     0, 20,  0,  0;
    % internal walls
     5,  0,  5,  8;
     5,  9,  5, 14;
     5, 15,  5, 20;
    10,  6, 10, 20;
    10,  0, 10,  4;
    15,  0, 15,  6;
    15,  8, 15, 13;
    15, 15, 15, 20;
     5, 14, 10, 14;
    10,  4, 15,  4;
    15,  6, 20,  6;
    15, 13, 20, 13;
];

%% ── GRID SETUP ───────────────────────────────────────────────────────────
NC_grid  = round(GRID_SIZE / GRID_RES);
NR_grid  = NC_grid;
log_odds = zeros(NR_grid, NC_grid);   % 0 = unknown

%% ── ROBOT INITIAL STATE ──────────────────────────────────────────────────
x0 = 2.5;  y0 = 2.5;  th0 = pi/4;
x  = x0;   y  = y0;   th  = th0;

%% ── LIDAR BEAM ANGLES ────────────────────────────────────────────────────
beam_angles = linspace(-pi, pi, LIDAR_NBEAMS + 1);
beam_angles(end) = [];

%% ── FIGURE SETUP ─────────────────────────────────────────────────────────
fig = figure('Name', 'Module 08 — Frontier Exploration', ...
             'Position', [60 60 850 800], 'Color', 'k');

ax = axes('Parent', fig, 'Color', 'k');
axis(ax, 'equal');
axis(ax, [0 GRID_SIZE 0 GRID_SIZE]);
xlabel(ax, 'x (m)', 'Color', 'w');
ylabel(ax, 'y (m)', 'Color', 'w');
title(ax, 'Frontier Exploration — building map...', 'Color', 'w', 'FontSize', 11);
ax.XColor = 'w';  ax.YColor = 'w';
hold(ax, 'on');

h_grid = imagesc(ax, [GRID_RES/2, GRID_SIZE - GRID_RES/2], ...
                     [GRID_RES/2, GRID_SIZE - GRID_RES/2], ...
                     zeros(NR_grid, NC_grid));
colormap(ax, gray);  caxis(ax, [-1 1]);
set(ax, 'YDir', 'normal');

for w = 1:size(WALLS, 1)
    plot(ax, WALLS(w,[1 3]), WALLS(w,[2 4]), 'c-', 'LineWidth', 1.2);
end

h_trail  = plot(ax, x0, y0, '-', 'Color', [1 0.6 0], 'LineWidth', 1.0);
trail_x  = x0;   trail_y  = y0;

h_robot  = plot(ax, x0, y0, 'wo', 'MarkerFaceColor', 'w', ...
                'MarkerSize', 8, 'LineWidth', 1.5);
h_path   = plot(ax, NaN, NaN, '--', 'Color', [0.4 0.8 0.4], 'LineWidth', 1.2);
h_fronts = plot(ax, NaN, NaN, 'd', 'Color', 'c', ...
                'MarkerFaceColor', 'none', 'MarkerSize', 8, 'LineWidth', 1.5);
h_target = plot(ax, NaN, NaN, 'p', 'Color', 'm', ...
                'MarkerFaceColor', 'm', 'MarkerSize', 14);

h_cov  = text(ax, 0.5,  19.3, 'Coverage: 0.0%', 'Color', 'y', ...
              'FontSize', 10, 'FontWeight', 'bold');
h_step = text(ax, 10.0, 19.3, 'Step: 0', 'Color', [0.7 0.7 0.7], ...
              'FontSize', 9, 'HorizontalAlignment', 'center');
drawnow;

%% ── EXPLORATION STATE ────────────────────────────────────────────────────
current_path      = [];
path_step_idx     = 1;
target_world      = [];
steps_no_prog     = 0;
last_dist_to_goal = inf;
frontiers         = struct('centroid_w',{}, 'centroid_rc',{}, 'size',{});
frontier_blacklist = zeros(0,3);   % [grid_r, grid_c, expiry_step]

gif_frame_idx    = 1;
exploration_done = false;
stop_reason      = '';

%% ── MAIN LOOP ─────────────────────────────────────────────────────────────
for step = 1:MAX_STEPS

    % ── 1. MOTION + DEAD-RECKONING ───────────────────────────────────────
    if ~isempty(current_path) && path_step_idx <= size(current_path,1)
        remaining = current_path(path_step_idx:end, :);
        [x, y, th] = pure_pursuit(x, y, th, remaining, LOOKAHEAD, V_ROBOT, DT);

        while path_step_idx < size(current_path,1)
            if norm([x,y] - current_path(path_step_idx,:)) < GRID_RES * 0.8
                path_step_idx = path_step_idx + 1;
            else
                break;
            end
        end
    else
        th = th + 0.05;
        x  = x  + 0.01 * cos(th);
        y  = y  + 0.01 * sin(th);
    end

    % odometry noise
    x  = x  + Q_SIG_V * randn * DT;
    y  = y  + Q_SIG_V * randn * DT;
    th = th + Q_SIG_W * randn * DT;
    th = atan2(sin(th), cos(th));

    x = max(0.3, min(GRID_SIZE - 0.3, x));
    y = max(0.3, min(GRID_SIZE - 0.3, y));

    trail_x(end+1) = x;
    trail_y(end+1) = y;

    % ── 2. LIDAR SCAN + OCCUPANCY GRID UPDATE ────────────────────────────
    % robot grid cell
    rob_c = max(1, min(NC_grid, floor((x - GRID_ORIGIN(1)) / GRID_RES) + 1));
    rob_r = max(1, min(NR_grid, NR_grid - floor((y - GRID_ORIGIN(2)) / GRID_RES)));

    for b = 1:LIDAR_NBEAMS
        abs_angle = th + beam_angles(b);
        dx_ray = cos(abs_angle);
        dy_ray = sin(abs_angle);

        % ray cast against all walls
        scan_range = LIDAR_RANGE;
        for wi = 1:size(WALLS,1)
            x1=WALLS(wi,1); y1=WALLS(wi,2); x2=WALLS(wi,3); y2=WALLS(wi,4);
            dxw=x2-x1; dyw=y2-y1;
            denom = dx_ray*dyw - dy_ray*dxw;
            if abs(denom) < 1e-9, continue; end
            tt = ((x1-x)*dyw - (y1-y)*dxw) / denom;
            uu = ((x1-x)*dy_ray - (y1-y)*dx_ray) / denom;
            if tt > 0.01 && tt < scan_range && uu >= 0 && uu <= 1
                scan_range = tt;
            end
        end

        hit_wall  = (scan_range < LIDAR_RANGE * 0.98);
        scan_range = max(0, scan_range + LIDAR_NOISE * randn);

        % beam endpoint grid cell
        end_x = x + scan_range * dx_ray;
        end_y = y + scan_range * dy_ray;
        end_c = max(1, min(NC_grid, floor((end_x - GRID_ORIGIN(1)) / GRID_RES) + 1));
        end_r = max(1, min(NR_grid, NR_grid - floor((end_y - GRID_ORIGIN(2)) / GRID_RES)));

        % Bresenham ray trace — mark free cells along ray, occupied at endpoint
        br=rob_r; bc=rob_c; br2=end_r; bc2=end_c;
        bdr=abs(br2-br); bdc=abs(bc2-bc);
        bsr=sign(br2-br); bsc=sign(bc2-bc);
        berr=bdr-bdc;
        while true
            if ~(br==br2 && bc==bc2)   % free (not the endpoint)
                log_odds(br,bc) = max(LO_MIN, log_odds(br,bc) + LO_FREE);
            end
            if br==br2 && bc==bc2, break; end
            be2=2*berr;
            if be2 > -bdc, berr=berr-bdc; br=br+bsr; end
            if be2 <  bdr, berr=berr+bdr; bc=bc+bsc; end
        end
        if hit_wall
            log_odds(end_r,end_c) = min(LO_MAX, log_odds(end_r,end_c) + LO_OCC);
        end
    end

    % ── 3. FRONTIER SELECTION ─────────────────────────────────────────────
    recompute_frontiers = (mod(step, FRONTIER_INTERVAL) == 0);

    if ~isempty(target_world)
        dist_to_goal = norm([x, y] - target_world);
        if dist_to_goal < GOAL_THRESH
            recompute_frontiers = true;
            target_world = [];
            current_path = [];
        end
        if dist_to_goal >= last_dist_to_goal - 0.01
            steps_no_prog = steps_no_prog + 1;
        else
            steps_no_prog = 0;
        end
        last_dist_to_goal = dist_to_goal;
        if steps_no_prog >= STUCK_LIMIT
            recompute_frontiers = true;
            steps_no_prog = 0;
            current_path  = [];
            if ~isempty(target_world)
                bl_c = max(1,min(NC_grid, floor((target_world(1)-GRID_ORIGIN(1))/GRID_RES)+1));
                bl_r = max(1,min(NR_grid, NR_grid-floor((target_world(2)-GRID_ORIGIN(2))/GRID_RES)));
                frontier_blacklist(end+1,:) = [bl_r, bl_c, step + BLACKLIST_TTL];
            end
        end
    else
        recompute_frontiers = true;
    end

    if recompute_frontiers
        frontiers = frontier_detector(log_odds, GRID_RES, GRID_ORIGIN);
        keep = arrayfun(@(f) f.size >= MIN_FRONTIER_SIZE, frontiers);
        frontiers = frontiers(keep);

        if isempty(frontiers)
            exploration_done = true;
            stop_reason = 'All frontiers explored — map complete!';
            break;
        end

        % expire old blacklist entries
        if ~isempty(frontier_blacklist)
            frontier_blacklist = frontier_blacklist(frontier_blacklist(:,3) > step, :);
        end

        % hybrid score: size / distance  (0 if blacklisted)
        scores = zeros(1, numel(frontiers));
        for fi = 1:numel(frontiers)
            fc_c = max(1,min(NC_grid, floor((frontiers(fi).centroid_w(1)-GRID_ORIGIN(1))/GRID_RES)+1));
            fc_r = max(1,min(NR_grid, NR_grid-floor((frontiers(fi).centroid_w(2)-GRID_ORIGIN(2))/GRID_RES)));
            is_bl = false;
            for bli = 1:size(frontier_blacklist,1)
                if abs(fc_r-frontier_blacklist(bli,1)) <= BLACKLIST_RADIUS && ...
                   abs(fc_c-frontier_blacklist(bli,2)) <= BLACKLIST_RADIUS
                    is_bl = true; break;
                end
            end
            if is_bl, continue; end
            dist_f = max(SCORE_DIST_MIN, norm([x,y] - frontiers(fi).centroid_w));
            scores(fi) = frontiers(fi).size / dist_f;
        end

        % if all blacklisted, clear and retry
        if max(scores) == 0
            frontier_blacklist = zeros(0,3);
            for fi = 1:numel(frontiers)
                dist_f = max(SCORE_DIST_MIN, norm([x,y] - frontiers(fi).centroid_w));
                scores(fi) = frontiers(fi).size / dist_f;
            end
        end

        [~, best_fi] = max(scores);
        target_world = frontiers(best_fi).centroid_w;

        % A* to target
        occ_grid = imdilate(log_odds > OCC_TH, strel('disk', INFLATE_R));

        % find nearest free cell to robot (drift may have put us inside a wall)
        s_c0 = max(1,min(NC_grid, floor((x-GRID_ORIGIN(1))/GRID_RES)+1));
        s_r0 = max(1,min(NR_grid, NR_grid-floor((y-GRID_ORIGIN(2))/GRID_RES)));
        [s_r, s_c] = nearest_free_cell(occ_grid, s_r0, s_c0, NR_grid, NC_grid);

        g_c = max(1,min(NC_grid, floor((target_world(1)-GRID_ORIGIN(1))/GRID_RES)+1));
        g_r = max(1,min(NR_grid, NR_grid-floor((target_world(2)-GRID_ORIGIN(2))/GRID_RES)));
        % also ensure goal is free
        [g_r, g_c] = nearest_free_cell(occ_grid, g_r, g_c, NR_grid, NC_grid);

        path_rc = astar_planner(occ_grid, [s_r s_c], [g_r g_c]);

        if isempty(path_rc)
            [~, sort_idx] = sort(scores, 'descend');
            found = false;
            for fi2 = 2:min(5, numel(sort_idx))
                target_world = frontiers(sort_idx(fi2)).centroid_w;
                g_c2 = max(1,min(NC_grid, floor((target_world(1)-GRID_ORIGIN(1))/GRID_RES)+1));
                g_r2 = max(1,min(NR_grid, NR_grid-floor((target_world(2)-GRID_ORIGIN(2))/GRID_RES)));
                path_rc = astar_planner(occ_grid, [s_r s_c], [g_r2 g_c2]);
                if ~isempty(path_rc), found=true; break; end
            end
            if ~found, current_path=[]; target_world=[]; end
        end

        if ~isempty(path_rc)
            current_path = zeros(size(path_rc,1), 2);
            for pi2 = 1:size(path_rc,1)
                pr=path_rc(pi2,1); pc=path_rc(pi2,2);
                current_path(pi2,1) = GRID_ORIGIN(1) + (pc-0.5)*GRID_RES;
                current_path(pi2,2) = GRID_ORIGIN(2) + (NR_grid-pr+0.5)*GRID_RES;
            end
            path_step_idx     = 1;
            last_dist_to_goal = norm([x,y] - target_world);
            steps_no_prog     = 0;
        end
    end

    % periodic path validity / replan
    if mod(step,REPLAN_EVERY)==0 && ~isempty(current_path) && ~isempty(target_world)
        occ_grid  = imdilate(log_odds > OCC_TH, strel('disk', INFLATE_R));
        rem_path  = current_path(path_step_idx:end,:);
        blocked   = false;
        for pi2 = 1:size(rem_path,1)
            rc_c2 = max(1,min(NC_grid, floor((rem_path(pi2,1)-GRID_ORIGIN(1))/GRID_RES)+1));
            rc_r2 = max(1,min(NR_grid, NR_grid-floor((rem_path(pi2,2)-GRID_ORIGIN(2))/GRID_RES)));
            if occ_grid(rc_r2,rc_c2), blocked=true; break; end
        end
        if blocked
            s_c2_0 = max(1,min(NC_grid, floor((x-GRID_ORIGIN(1))/GRID_RES)+1));
            s_r2_0 = max(1,min(NR_grid, NR_grid-floor((y-GRID_ORIGIN(2))/GRID_RES)));
            [s_r2, s_c2] = nearest_free_cell(occ_grid, s_r2_0, s_c2_0, NR_grid, NC_grid);
            g_c2 = max(1,min(NC_grid, floor((target_world(1)-GRID_ORIGIN(1))/GRID_RES)+1));
            g_r2 = max(1,min(NR_grid, NR_grid-floor((target_world(2)-GRID_ORIGIN(2))/GRID_RES)));
            [g_r2, g_c2] = nearest_free_cell(occ_grid, g_r2, g_c2, NR_grid, NC_grid);
            new_rc = astar_planner(occ_grid, [s_r2 s_c2], [g_r2 g_c2]);
            if ~isempty(new_rc)
                current_path = zeros(size(new_rc,1),2);
                for pi2=1:size(new_rc,1)
                    pr=new_rc(pi2,1); pc=new_rc(pi2,2);
                    current_path(pi2,1) = GRID_ORIGIN(1)+(pc-0.5)*GRID_RES;
                    current_path(pi2,2) = GRID_ORIGIN(2)+(NR_grid-pr+0.5)*GRID_RES;
                end
                path_step_idx = 1;
            end
        end
    end

    % ── 4. VISUALISATION ─────────────────────────────────────────────────
    if mod(step, VIS_EVERY) == 0
        set(h_grid,  'CData', flipud(max(-1, min(1, log_odds/LO_MAX))));
        set(h_trail, 'XData', trail_x, 'YData', trail_y);
        set(h_robot, 'XData', x, 'YData', y);

        if ~isempty(current_path)
            rp = current_path(path_step_idx:end,:);
            set(h_path, 'XData', rp(:,1), 'YData', rp(:,2));
        else
            set(h_path, 'XData', NaN, 'YData', NaN);
        end

        if ~isempty(frontiers)
            fc = vertcat(frontiers.centroid_w);
            set(h_fronts, 'XData', fc(:,1), 'YData', fc(:,2));
        else
            set(h_fronts, 'XData', NaN, 'YData', NaN);
        end

        if ~isempty(target_world)
            set(h_target, 'XData', target_world(1), 'YData', target_world(2));
        else
            set(h_target, 'XData', NaN, 'YData', NaN);
        end

        known_cells = sum(abs(log_odds(:)) > 0.1);
        cov_pct     = 100 * known_cells / (NR_grid * NC_grid);
        set(h_cov,  'String', sprintf('Coverage: %.1f%%', cov_pct));
        set(h_step, 'String', sprintf('Step: %d / %d', step, MAX_STEPS));
        title(ax, sprintf('Frontier Exploration  |  step %d  |  coverage: %.1f%%', ...
              step, cov_pct), 'Color', 'w', 'FontSize', 10);

        drawnow limitrate;

        if GIF_ENABLED && mod(step, GIF_EVERY) == 0
            save_exploration_gif(fig, GIF_FILE, GIF_DELAY, gif_frame_idx);
            gif_frame_idx = gif_frame_idx + 1;
        end
    end

end  % main loop

%% ── TERMINATION ──────────────────────────────────────────────────────────
if ~exploration_done
    stop_reason = sprintf('Step budget (%d steps) reached.', MAX_STEPS);
end

known_cells = sum(abs(log_odds(:)) > 0.1);
cov_pct     = 100 * known_cells / (NR_grid * NC_grid);

set(h_grid,   'CData', flipud(max(-1, min(1, log_odds/LO_MAX))));
set(h_trail,  'XData', trail_x, 'YData', trail_y);
set(h_robot,  'XData', x, 'YData', y);
set(h_fronts, 'XData', NaN, 'YData', NaN);
set(h_target, 'XData', NaN, 'YData', NaN);
set(h_path,   'XData', NaN, 'YData', NaN);
title(ax, sprintf('Done! %s  |  Final coverage: %.1f%%', stop_reason, cov_pct), ...
      'Color', [0.4 1 0.4], 'FontSize', 10);
drawnow;

if GIF_ENABLED
    save_exploration_gif(fig, GIF_FILE, 1.5, gif_frame_idx);
end

fprintf('\n=== Module 08 Complete ===\n');
fprintf('Stop reason : %s\n', stop_reason);
fprintf('Steps run   : %d\n', step);
fprintf('Coverage    : %.1f%%\n', cov_pct);
fprintf('GIF saved   : %s\n', GIF_FILE);