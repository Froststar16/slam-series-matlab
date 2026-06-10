% =============================================================
%  Module 06 — Occupancy Grid + A* Navigation
%  Pipeline: EKF SLAM → Occupancy Grid → A* → Smooth → Execute
%
%  Usage:
%    1. Run this script.
%    2. Wait for SLAM + grid to build (progress shown in title).
%    3. Click a goal point on the map when prompted.
%    4. Watch the robot follow the smoothed A* path.
%
%  Requirements: MATLAB R2024a, Image Processing Toolbox
% =============================================================

clear; clc; close all;
rng(42);

% ── SLAM parameters ──────────────────────────────────────────
N_STEPS      = 400;
DT           = 0.1;
MAX_RANGE    = 4.0;
N_BEAMS      = 36;
SIGMA_R      = 0.1;
SIGMA_B      = deg2rad(2);
Q_EKF        = diag([SIGMA_R^2, SIGMA_B^2]);
R_EKF        = diag([0.02^2, deg2rad(1)^2]);
ASSOC_GATE   = 1.5;

% ── World ────────────────────────────────────────────────────
WORLD_SIZE = 10.0;
LANDMARKS  = [ 2,2; 2,7; 5,5; 7,2; 7,8;
               3,4; 6,3; 4,7; 8,5; 1,8 ];

% ── Occupancy grid parameters ────────────────────────────────
GRID_RES   = 0.1;
L_FREE     = -0.5;
L_OCC      = 1.5;
L_MIN      = -5;
L_MAX      = 5;
OCC_THRESH = 0.5;
INFLATE_R  = 2;

% ── Animation ────────────────────────────────────────────────
EXEC_SPEED = 0.03;

% =============================================================
%  PART 1 — EKF SLAM
% =============================================================
fprintf('=== Phase 1/3 : Running EKF SLAM ===\n');

x_true = [1; 1; 0];
x_est  = x_true;
P      = zeros(3,3);
lm_map = zeros(0,1);   % always a column vector
n_lm   = 0;

traj_true = x_true(1:2);
traj_est  = x_est(1:2);

theta_path = linspace(0, 4*pi, N_STEPS);
path_x = 4 + 3.5*sin(theta_path);
path_y = 5 + 2.5*sin(theta_path/2);

for k = 1:N_STEPS

    % ── control ──────────────────────────────────────────────
    ddx   = path_x(k) - x_true(1);
    ddy   = path_y(k) - x_true(2);
    d_des = sqrt(ddx^2 + ddy^2);
    h_des = atan2(ddy, ddx);
    v_cmd = min(0.4, d_des / DT);
    w_cmd = wrap_angle(h_des - x_true(3)) / DT;
    w_cmd = max(-1.5, min(1.5, w_cmd));
    u_cmd = [v_cmd; w_cmd];

    % ── true motion ───────────────────────────────────────────
    x_true    = motion_model(x_true, u_cmd, DT) + ...
                [randn*0.01; randn*0.01; randn*deg2rad(0.5)];
    x_true(3) = wrap_angle(x_true(3));

    % ── EKF predict ───────────────────────────────────────────
    [x_pred, Fmat, Vmat] = ekf_predict(x_est(1:3), P(1:3,1:3), u_cmd, DT);
    n_state = 3 + 2*n_lm;
    P_full  = zeros(n_state);
    P_full(1:3,1:3) = P(1:3,1:3);
    for ii = 1:n_lm
        idx = 3 + (ii-1)*2 + (1:2);
        P_full(idx,idx) = P(idx,idx);
        P_full(1:3,idx) = P(1:3,idx);
        P_full(idx,1:3) = P(idx,1:3);
    end
    P_full(1:3,1:3) = Fmat*P_full(1:3,1:3)*Fmat' + Vmat*R_EKF*Vmat';
    x_aug = [x_pred; lm_map];   % lm_map is always column

    % ── LiDAR scan ────────────────────────────────────────────
    bearings = linspace(-pi, pi, N_BEAMS);
    obs_list = [];
    for b = 1:N_BEAMS
        ang_b = wrap_angle(x_true(3) + bearings(b));
        for lm = 1:size(LANDMARKS,1)
            ddx2 = LANDMARKS(lm,1) - x_true(1);
            ddy2 = LANDMARKS(lm,2) - x_true(2);
            rng_lm = sqrt(ddx2^2 + ddy2^2);
            if rng_lm < MAX_RANGE
                bear_lm = wrap_angle(atan2(ddy2,ddx2) - x_true(3));
                if abs(wrap_angle(bear_lm - bearings(b))) < deg2rad(5)
                    obs_list = [obs_list; ...
                        rng_lm + randn*SIGMA_R, ...
                        bear_lm + randn*SIGMA_B, lm]; %#ok<AGROW>
                    break;
                end
            end
        end
    end

    % ── EKF update ────────────────────────────────────────────
    for o = 1:size(obs_list,1)
        r_obs  = obs_list(o,1);
        b_obs  = obs_list(o,2);
        min_md = inf;
        best_id = -1;

        for ii = 1:n_lm
            lx_i = x_aug(3+(ii-1)*2+1);
            ly_i = x_aug(3+(ii-1)*2+2);
            ddx2 = lx_i - x_aug(1);
            ddy2 = ly_i - x_aug(2);
            r_p  = sqrt(ddx2^2 + ddy2^2);
            b_p  = wrap_angle(atan2(ddy2,ddx2) - x_aug(3));
            innov2 = [r_obs - r_p; wrap_angle(b_obs - b_p)];
            H2     = obs_jacobian(x_aug, ii);
            S2     = H2*P_full*H2' + Q_EKF;
            md2    = innov2' * (S2 \ innov2);
            if md2 < min_md
                min_md  = md2;
                best_id = ii;
            end
        end

        if n_lm == 0 || min_md > ASSOC_GATE^2
            % --- initialise new landmark ---
            n_lm   = n_lm + 1;
            rob_x  = x_aug(1);
            rob_y  = x_aug(2);
            rob_th = x_aug(3);
            new_lx = rob_x + r_obs * cos(wrap_angle(rob_th + b_obs));
            new_ly = rob_y + r_obs * sin(wrap_angle(rob_th + b_obs));
            new_lm = [new_lx; new_ly];        % 2×1 column
            lm_map = [lm_map; new_lm];        % grow column vector
            x_aug  = [x_aug; new_lm];         % append to state
            n_state = numel(x_aug);
            P_new  = zeros(n_state);
            P_new(1:n_state-2, 1:n_state-2) = P_full;
            P_new(end-1:end,   end-1:end)   = eye(2)*5;
            P_full = P_new;
        else
            % --- update existing landmark ---
            lx_b  = x_aug(3+(best_id-1)*2+1);
            ly_b  = x_aug(3+(best_id-1)*2+2);
            ddx2  = lx_b - x_aug(1);
            ddy2  = ly_b - x_aug(2);
            r_p   = sqrt(ddx2^2 + ddy2^2);
            b_p   = wrap_angle(atan2(ddy2,ddx2) - x_aug(3));
            innov2 = [r_obs - r_p; wrap_angle(b_obs - b_p)];
            H2     = obs_jacobian(x_aug, best_id);
            S2     = H2*P_full*H2' + Q_EKF;
            K2     = P_full * H2' / S2;
            x_aug  = x_aug + K2*innov2;
            x_aug(3) = wrap_angle(x_aug(3));
            P_full = (eye(n_state) - K2*H2) * P_full;
        end
    end

    x_est  = x_aug;
    P      = P_full;
    lm_map = x_aug(4:end);          % stays a column vector
    traj_true(:,end+1) = x_true(1:2); 
    traj_est(:,end+1)  = x_est(1:2);  
end
fprintf('  SLAM complete. Landmarks found: %d\n', n_lm);

% =============================================================
%  PART 2 — OCCUPANCY GRID
% =============================================================
fprintf('=== Phase 2/3 : Building occupancy grid ===\n');

N_CELLS  = round(WORLD_SIZE / GRID_RES);
log_grid = zeros(N_CELLS, N_CELLS);

for k = 1:5:size(traj_true,2)
    pose = [traj_true(1,k); traj_true(2,k)];
    for b = 1:N_BEAMS*3
        ang_map = (b / (N_BEAMS*3)) * 2*pi;
        hit_r   = MAX_RANGE;
        for lm = 1:size(LANDMARKS,1)
            ddx2   = LANDMARKS(lm,1) - pose(1);
            ddy2   = LANDMARKS(lm,2) - pose(2);
            r_lm   = sqrt(ddx2^2 + ddy2^2);
            bear_m = atan2(ddy2, ddx2);
            if r_lm < MAX_RANGE && abs(wrap_angle(bear_m - ang_map)) < deg2rad(4)
                hit_r = min(hit_r, r_lm);
            end
        end
        % free cells along ray
        n_steps = floor(hit_r / GRID_RES);
        for s = 1:n_steps
            px = pose(1) + s*GRID_RES*cos(ang_map);
            py = pose(2) + s*GRID_RES*sin(ang_map);
            ci = world2cell(px, py, GRID_RES, N_CELLS);
            if ci(1)>=1 && ci(1)<=N_CELLS && ci(2)>=1 && ci(2)<=N_CELLS
                log_grid(ci(2),ci(1)) = ...
                    clamp(log_grid(ci(2),ci(1)) + L_FREE, L_MIN, L_MAX);
            end
        end
        % occupied hit cell
        if hit_r < MAX_RANGE
            px = pose(1) + hit_r*cos(ang_map);
            py = pose(2) + hit_r*sin(ang_map);
            ci = world2cell(px, py, GRID_RES, N_CELLS);
            if ci(1)>=1 && ci(1)<=N_CELLS && ci(2)>=1 && ci(2)<=N_CELLS
                log_grid(ci(2),ci(1)) = ...
                    clamp(log_grid(ci(2),ci(1)) + L_OCC, L_MIN, L_MAX);
            end
        end
    end
end

binary_grid   = log_grid > OCC_THRESH;
se            = strel('disk', INFLATE_R);
inflated_grid = imdilate(binary_grid, se);

fprintf('  Grid built: %dx%d cells (%.1fm x %.1fm)\n', ...
    N_CELLS, N_CELLS, WORLD_SIZE, WORLD_SIZE);

% =============================================================
%  PART 3 — INTERACTIVE GOAL SELECTION + A* PLANNING
% =============================================================
fprintf('=== Phase 3/3 : Interactive goal selection ===\n');

fig = figure('Name','Module 06 - A* Navigation','NumberTitle','off',...
    'Color',[0.08 0.08 0.12],'Position',[80 80 900 820]);

ax = axes('Parent',fig,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w');
hold(ax,'on'); axis(ax,'equal');
xlim(ax,[0 WORLD_SIZE]); ylim(ax,[0 WORLD_SIZE]);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');

% Render occupancy grid
grid_img = zeros(N_CELLS, N_CELLS, 3);
for row = 1:N_CELLS
    for col = 1:N_CELLS
        prob = 1 / (1 + exp(-log_grid(row,col)));
        if prob > 0.6
            grid_img(row,col,:) = [0.85, 0.20, 0.20];
        elseif prob < 0.4
            grid_img(row,col,:) = [0.15, 0.18, 0.22];
        else
            grid_img(row,col,:) = [0.35, 0.35, 0.38];
        end
    end
end
image(ax, [0 WORLD_SIZE], [0 WORLD_SIZE], flipud(grid_img));

% Landmarks + trajectory (excluded from legend)
plot(ax, LANDMARKS(:,1), LANDMARKS(:,2), ...
    'o','MarkerSize',10,'MarkerFaceColor',[1 0.8 0],...
    'MarkerEdgeColor',[1 0.5 0],'LineWidth',1.5,...
    'HandleVisibility','off');
plot(ax, traj_true(1,:), traj_true(2,:), '-',...
    'Color',[0.4 0.9 0.4],'LineWidth',1.2,...
    'HandleVisibility','off');

% Robot marker at start (excluded from legend)
start_world = traj_true(1:2, 1)';
h_robot = plot(ax, start_world(1), start_world(2), ...
    'o','MarkerSize',14,'MarkerFaceColor',[0.3 0.6 1],...
    'MarkerEdgeColor','w','LineWidth',2,...
    'HandleVisibility','off');

title(ax, {'\color{white}Map built - Click a GOAL point in free space', ...
    '\color[rgb]{0.5 0.8 0.5}Green=trajectory   \color[rgb]{0.85 0.2 0.2}Red=obstacle   \color[rgb]{0.3 0.6 1}Blue=robot'}, ...
    'FontSize',11,'FontName','Courier');
drawnow;

% ── User clicks goal ─────────────────────────────────────────
% world2cell returns [col, row] i.e. [x-index, y-index].
% inflated_grid is indexed as (row, col) where row=1 is the TOP
% of the image (high Y). So: grid_row = N_CELLS - y_cell + 1.
% We also check a 5x5 neighbourhood so clicks near obstacle
% edges are safely rejected.
CLICK_PAD = 3;   % reject if any cell within this radius is occupied
valid_goal = false;
goal_world = [];
while ~valid_goal
    [gx, gy] = ginput(1);
    gx = clamp(gx, 0.1, WORLD_SIZE-0.1);
    gy = clamp(gy, 0.1, WORLD_SIZE-0.1);
    gc_tmp  = world2cell(gx, gy, GRID_RES, N_CELLS);
    gc_col  = gc_tmp(1);                        % x → column
    gc_row  = N_CELLS - gc_tmp(2) + 1;         % y → row (flipped)
    gc_col  = max(1, min(N_CELLS, gc_col));
    gc_row  = max(1, min(N_CELLS, gc_row));

    % Check padded neighbourhood
    r_lo = max(1, gc_row - CLICK_PAD);
    r_hi = min(N_CELLS, gc_row + CLICK_PAD);
    c_lo = max(1, gc_col - CLICK_PAD);
    c_hi = min(N_CELLS, gc_col + CLICK_PAD);
    neighbourhood = inflated_grid(r_lo:r_hi, c_lo:c_hi);

    if any(neighbourhood(:))
        title(ax, '\color{red}Too close to an obstacle - click in open free space',...
            'FontSize',12,'FontName','Courier');
        drawnow;
    else
        valid_goal = true;
        goal_world = [gx, gy];
    end
end

plot(ax, goal_world(1), goal_world(2), ...
    'p','MarkerSize',22,'MarkerFaceColor',[1 0.4 0.8],...
    'MarkerEdgeColor','w','LineWidth',2);
title(ax, '\color{white}Goal set - Running A* planner...',...
    'FontSize',12,'FontName','Courier');
drawnow;

% =============================================================
%  A* PLANNING
% =============================================================
sc_tmp = world2cell(start_world(1), start_world(2), GRID_RES, N_CELLS);
sc_r   = N_CELLS - sc_tmp(2) + 1;
sc_c   = sc_tmp(1);

gc_tmp2 = world2cell(goal_world(1), goal_world(2), GRID_RES, N_CELLS);
gc_r    = N_CELLS - gc_tmp2(2) + 1;
gc_c    = gc_tmp2(1);

[path_cells, found] = astar_planner(inflated_grid, [sc_r sc_c], [gc_r gc_c]);

if ~found
    title(ax, '\color{red}A* could not find a path. Try a different goal.',...
        'FontSize',12,'FontName','Courier');
    fprintf('A* failed - no valid path to goal.\n');
    return;
end

% Cell indices → world coordinates
path_world = zeros(size(path_cells,1), 2);
for i = 1:size(path_cells,1)
    path_world(i,1) = (path_cells(i,2) - 0.5) * GRID_RES;
    path_world(i,2) = (N_CELLS - path_cells(i,1) + 0.5) * GRID_RES;
end

h_raw = plot(ax, path_world(:,1), path_world(:,2), '--',...
    'Color',[1 0.6 0],'LineWidth',1.2,...
    'HandleVisibility','on');
drawnow;
title(ax, '\color{white}A* found - Smoothing path...',...
    'FontSize',12,'FontName','Courier');
pause(0.4);

% =============================================================
%  PATH SMOOTHING
% =============================================================
smooth_path = path_smoother(path_world, inflated_grid, GRID_RES, N_CELLS);

h_smooth = plot(ax, smooth_path(:,1), smooth_path(:,2), '-',...
    'Color',[0.3 0.9 1],'LineWidth',2.5,...
    'HandleVisibility','on');

% Explicit gobjects handle array — prevents auto-capture of other plot lines
leg_handles = gobjects(2,1);
leg_handles(1) = h_raw;
leg_handles(2) = h_smooth;
legend(ax, leg_handles, {'Raw A*','Smoothed path'}, ...
    'TextColor','w','Color',[0.15 0.15 0.2],...
    'EdgeColor',[0.4 0.4 0.4],'Location','northwest','FontSize',9);

title(ax, '\color{white}Smooth path ready - Executing...',...
    'FontSize',12,'FontName','Courier');
drawnow; pause(0.5);

% =============================================================
%  ROBOT EXECUTION ANIMATION
% =============================================================
h_trace = plot(ax, smooth_path(1,1), smooth_path(1,2), '-',...
    'Color',[0.3 0.6 1],'LineWidth',2,...
    'HandleVisibility','off');

n_wp    = size(smooth_path,1);
trace_x = smooth_path(1,1);
trace_y = smooth_path(1,2);

for i = 2:n_wp
    trace_x(end+1) = smooth_path(i,1); 
    trace_y(end+1) = smooth_path(i,2); 
    set(h_trace, 'XData', trace_x, 'YData', trace_y);
    set(h_robot, 'XData', smooth_path(i,1), 'YData', smooth_path(i,2));
    title(ax, sprintf('\\color{white}Executing path  [%.0f%%]', 100*i/n_wp),...
        'FontSize',12,'FontName','Courier');
    drawnow;
    pause(EXEC_SPEED);
end

set(h_robot,'MarkerFaceColor',[0.3 1 0.4],'MarkerSize',16);
dist_total = sum(sqrt(sum(diff(smooth_path).^2, 2)));
title(ax, sprintf('\\color[rgb]{0.3 1 0.4}Goal reached!  Path length: %.2f m', dist_total),...
    'FontSize',13,'FontName','Courier','FontWeight','bold');

fprintf('\n=== Navigation complete ===\n');
fprintf('  Raw A* waypoints : %d\n', size(path_world,1));
fprintf('  Smooth waypoints : %d\n', n_wp);
fprintf('  Path length      : %.3f m\n', dist_total);

% =============================================================
%  GIF EXPORT
% =============================================================
fprintf('\nSaving animation GIF...\n');
save_nav_gif(fig, smooth_path, h_robot, h_trace, ax, EXEC_SPEED, 'nav_astar.gif');
fprintf('  Saved -> nav_astar.gif\n');

% =============================================================
%  LOCAL HELPER FUNCTIONS
% =============================================================

function xn = motion_model(xv, uv, dt)
    xn = xv + dt * [uv(1)*cos(xv(3)); uv(1)*sin(xv(3)); uv(2)];
end

function [xp, Fout, Vout] = ekf_predict(xv, ~, uv, dt)
    vc = uv(1); wc = uv(2); th = xv(3);
    xp    = xv + dt*[vc*cos(th); vc*sin(th); wc];
    xp(3) = wrap_angle(xp(3));
    Fout  = eye(3) + dt*[-vc*sin(th), 0, 0;
                          vc*cos(th), 0, 0;
                          0,          0, 0];
    Vout  = dt*[cos(th), 0; sin(th), 0; 0, 1];
end

function H = obs_jacobian(xv, lm_idx)
    ns  = numel(xv);
    lxj = xv(3+(lm_idx-1)*2+1);
    lyj = xv(3+(lm_idx-1)*2+2);
    ddx = lxj - xv(1);
    ddy = lyj - xv(2);
    r2  = ddx^2 + ddy^2;
    rv  = sqrt(r2);
    H   = zeros(2, ns);
    H(1,1) = -ddx/rv;  H(1,2) = -ddy/rv;
    H(2,1) =  ddy/r2;  H(2,2) = -ddx/r2;  H(2,3) = -1;
    H(1, 3+(lm_idx-1)*2+1) =  ddx/rv;
    H(1, 3+(lm_idx-1)*2+2) =  ddy/rv;
    H(2, 3+(lm_idx-1)*2+1) = -ddy/r2;
    H(2, 3+(lm_idx-1)*2+2) =  ddx/r2;
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function v = clamp(v, lo, hi)
    v = max(lo, min(hi, v));
end

function ci = world2cell(wx, wy, res, n)
    ci = [max(1, min(n, floor(wx/res)+1)), ...
          max(1, min(n, floor(wy/res)+1))];
end