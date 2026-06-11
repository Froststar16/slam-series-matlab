% =============================================================
%  Module 07a — Full Autonomous Navigation Pipeline (MATLAB)
%
%  Pipeline: LiDAR -> Occupancy Grid -> A* -> Pure Pursuit -> Move
%
%  Usage:
%    1. Run script. Red=wall, dark blue=free.
%    2. Click 3 goals in dark blue space, press Enter.
%    3. Screen-record. Robot patrols indefinitely.
%
%  Needs on MATLAB path: astar_planner.m, path_smoother.m (Module 06)
% =============================================================

clear; clc; close all;
rng(42);

% Add Module 06 to path (astar_planner.m + path_smoother.m live there)
% Folder structure:
%   slam-series-matlab/
%     06_astar_navigation/   <- astar_planner.m, path_smoother.m
%     07_autonomous_navigation/
%       autonomous_nav_main.m  <- this file
this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, '..', '06_astar_navigation'));

% ── Parameters ────────────────────────────────────────────────
DT           = 0.08;
MAX_RANGE    = 3.5;
N_BEAMS      = 72;
SIGMA_R      = 0.04;
GRID_RES     = 0.10;
L_FREE       = -0.3;
L_OCC        = 1.5;
L_MIN        = -5;   L_MAX = 5;
OCC_THRESH   = 0.5;
INFLATE_R    = 2;    % 2 cells = 0.2m margin — enough clearance without choking corridors
LOOKAHEAD    = 0.40; % shorter = tighter corner tracking
V_MAX        = 0.25; % slower = less overshoot at corners
GOAL_THRESH  = 0.45;
REPLAN_EVERY = 15;   % replan more often to catch drift early
N_GOALS      = 3;
GOAL_PAD     = 4;
MAX_STEPS    = 8000; % more steps for full patrol recording

% =============================================================
%  MAZE — verified connected, all corridors ~4m wide
% =============================================================
S = 10.0;
WALLS = [
    0,0, S,0;   S,0, S,S;   S,S, 0,S;   0,S, 0,0;
    0,5, 4,5;   6,5, S,5;   % horizontal divider, gap x=4..6
    5,0, 5,4;   5,6, 5,S;   % vertical divider,   gap y=4..6
];
N_CELLS = round(S / GRID_RES);

% Rasterise walls
wall_grid = rasterise_walls(WALLS, GRID_RES, N_CELLS);
se        = strel('disk', INFLATE_R);
wall_inf  = imdilate(wall_grid, se);

% Pre-seed log-odds: walls at L_MAX so A* respects them from step 1.
% Free cells start at a slightly negative value so the smoother's
% LOS check also treats unseen space as passable (not blocked).
log_grid = zeros(N_CELLS, N_CELLS);
log_grid(wall_grid) = L_MAX;

% =============================================================
%  FIGURE + GOAL SELECTION
% =============================================================
fig = figure('Name','Module 07 — Autonomous Navigation',...
    'NumberTitle','off','Color',[0.06 0.06 0.09],...
    'Position',[50 50 920 860]);
ax = axes('Parent',fig,'Color',[0.06 0.06 0.09],...
    'XColor','w','YColor','w','FontName','Courier');
hold(ax,'on'); axis(ax,'equal');
xlim(ax,[0 S]); ylim(ax,[0 S]);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');

% Static background for goal selection
static_img = make_static_img(wall_inf, N_CELLS);
image(ax,[0 S],[0 S],flipud(static_img));
draw_walls(ax, WALLS);
title(ax,{'\color{white}Click 3 goals in DARK BLUE free corridors, then press Enter',...
    '\color[rgb]{0.85 0.2 0.2}Red=wall   \color[rgb]{0.35 0.65 1}Dark blue=free'},...
    'FontSize',11,'FontName','Courier');
drawnow;

fprintf('Click %d goals in dark blue corridors, then press Enter.\n', N_GOALS);
goals = zeros(N_GOALS, 2);
gi = 0;
while gi < N_GOALS
    [gx,gy] = ginput(1);
    if isempty(gx); break; end
    gx = max(0.3, min(S-0.3, gx));
    gy = max(0.3, min(S-0.3, gy));
    [gc,gr] = xy2grid(gx, gy, GRID_RES, N_CELLS);
    rlo=max(1,gr-GOAL_PAD); rhi=min(N_CELLS,gr+GOAL_PAD);
    clo=max(1,gc-GOAL_PAD); chi=min(N_CELLS,gc+GOAL_PAD);
    if any(any(wall_inf(rlo:rhi, clo:chi)))
        title(ax,'\color{red}Too close to a wall — click in open dark blue space',...
            'FontSize',11,'FontName','Courier');
        drawnow; continue;
    end
    gi = gi+1;
    goals(gi,:) = [gx, gy];
    plot(ax,gx,gy,'p','MarkerSize',22,...
        'MarkerFaceColor',gcol(gi),'MarkerEdgeColor','w','LineWidth',1.5,...
        'HandleVisibility','off');
    text(ax,gx+0.2,gy+0.2,sprintf('G%d',gi),'Color','w',...
        'FontSize',10,'FontName','Courier','FontWeight','bold',...
        'HandleVisibility','off');
    title(ax,sprintf('\\color{white}Goal %d/%d set — click next or press Enter',gi,N_GOALS),...
        'FontSize',11,'FontName','Courier');
    drawnow;
end
title(ax,'\color[rgb]{0.3 1 0.4}Goals set — press Enter to start patrol!',...
    'FontSize',12,'FontName','Courier','FontWeight','bold');
waitforbuttonpress;

% =============================================================
%  ROBOT STATE
% =============================================================
x_true      = [1.5; 1.5; pi/4];
goal_idx    = 1;
path_pts    = zeros(0,2);
replan_flag = true;
goals_reached = 0;

% Plot handles
h_img  = [];
h_path = plot(ax,nan,nan,'-','LineWidth',2.5,'Color',gcol(1),...
    'HandleVisibility','off');
h_lh   = plot(ax,nan,nan,'o','MarkerSize',10,...
    'MarkerFaceColor',[1 1 0.2],'MarkerEdgeColor',[0.6 0.6 0],...
    'HandleVisibility','off');
h_rob  = plot(ax,x_true(1),x_true(2),'o','MarkerSize',14,...
    'MarkerFaceColor',[0.25 0.55 1],'MarkerEdgeColor','w','LineWidth',2,...
    'HandleVisibility','off');
h_hdg  = plot(ax,[x_true(1),x_true(1)+0.4*cos(x_true(3))],...
                 [x_true(2),x_true(2)+0.4*sin(x_true(3))],'-',...
    'Color',[0.25 0.55 1],'LineWidth',2.5,'HandleVisibility','off');
h_traj = plot(ax,x_true(1),x_true(2),'-',...
    'Color',[0.3 0.9 0.3],'LineWidth',1.2,'HandleVisibility','off');
traj_x = x_true(1);
traj_y = x_true(2);

fprintf('\n=== Patrol started — screen-record now! ===\n');
fprintf('Suggested goals: (1.5,8.5)  (8.5,8.5)  (8.5,1.5)\n\n');

% =============================================================
%  MAIN LOOP
% =============================================================
for step = 1:MAX_STEPS

    % ── 1. LiDAR ─────────────────────────────────────────────
    bearings = linspace(-pi, pi*(1-1/N_BEAMS), N_BEAMS);
    rng_scan = zeros(1,N_BEAMS);
    for b = 1:N_BEAMS
        ang = wrapToPi(x_true(3) + bearings(b));
        rng_scan(b) = raycast(x_true(1),x_true(2),ang,WALLS,MAX_RANGE) ...
                      + randn*SIGMA_R;
    end

    % ── 2. Grid update ────────────────────────────────────────
    for b = 1:N_BEAMS
        ang   = wrapToPi(x_true(3) + bearings(b));
        hit_r = max(0, min(MAX_RANGE, rng_scan(b)));
        n_free = floor(hit_r / GRID_RES);
        for s = 1:n_free
            px = x_true(1)+s*GRID_RES*cos(ang);
            py = x_true(2)+s*GRID_RES*sin(ang);
            [cc,rr] = xy2grid(px,py,GRID_RES,N_CELLS);
            if valid_cell(cc,rr,N_CELLS)
                log_grid(rr,cc) = max(L_MIN, log_grid(rr,cc)+L_FREE);
            end
        end
        if rng_scan(b) < MAX_RANGE - SIGMA_R*2
            px = x_true(1)+hit_r*cos(ang);
            py = x_true(2)+hit_r*sin(ang);
            [cc,rr] = xy2grid(px,py,GRID_RES,N_CELLS);
            if valid_cell(cc,rr,N_CELLS)
                log_grid(rr,cc) = min(L_MAX, log_grid(rr,cc)+L_OCC);
            end
        end
    end

    % ── 3. Planning grid (LiDAR + pre-seeded walls) ───────────
    occ_bin   = log_grid > OCC_THRESH;
    plan_grid = imdilate(occ_bin, se);

    % ── 4. Goal management ────────────────────────────────────
    goal = goals(goal_idx,:);
    d2g  = norm(x_true(1:2)' - goal);
    if d2g < GOAL_THRESH
        goals_reached = goals_reached + 1;
        fprintf('  [step %d] Reached Goal %d! (total reached: %d) -> Goal %d\n',...
            step, goal_idx, goals_reached, mod(goal_idx,N_GOALS)+1);
        goal_idx    = mod(goal_idx,N_GOALS)+1;
        goal        = goals(goal_idx,:);
        d2g         = norm(x_true(1:2)'-goal);
        replan_flag = true;
        set(h_path,'Color',gcol(goal_idx));
    end

    % ── 5. Replanning ─────────────────────────────────────────
    if replan_flag || mod(step,REPLAN_EVERY)==0
        blocked = is_path_blocked(path_pts,plan_grid,GRID_RES,N_CELLS);
        if replan_flag || blocked
            if blocked && ~replan_flag
                fprintf('  [step %d] Path blocked — replanning\n', step);
                set(h_path,'Color',[1 0.15 0.15]); drawnow; pause(0.1);
                set(h_path,'Color',gcol(goal_idx));
            end
            [path_pts,ok] = plan_astar(x_true(1:2)',goal,...
                plan_grid,GRID_RES,N_CELLS);
            if ok
                set(h_path,'XData',path_pts(:,1),'YData',path_pts(:,2));
            else
                fprintf('  [step %d] No path to Goal %d\n',step,goal_idx);
                path_pts = zeros(0,2);
                set(h_path,'XData',nan,'YData',nan);
            end
            replan_flag = false;
        end
    end

    % ── 6. Pure pursuit ───────────────────────────────────────
    if size(path_pts,1) >= 2
        [v_cmd,w_cmd,lh] = pure_pursuit(x_true',path_pts,LOOKAHEAD,V_MAX);
        set(h_lh,'XData',lh(1),'YData',lh(2));
    else
        v_cmd=0; w_cmd=0;
        set(h_lh,'XData',nan,'YData',nan);
    end

    % ── 7. Motion step ────────────────────────────────────────
    x_true(1) = x_true(1) + DT*v_cmd*cos(x_true(3)) + randn*0.005;
    x_true(2) = x_true(2) + DT*v_cmd*sin(x_true(3)) + randn*0.005;
    x_true(3) = wrapToPi(x_true(3) + DT*w_cmd + randn*deg2rad(0.3));

    % ── 8. Visualise every 5 steps ────────────────────────────
    if mod(step,5) == 0
        gimg = make_grid_img(log_grid,N_CELLS);
        if isempty(h_img) || ~isvalid(h_img)
            h_img = image(ax,[0 S],[0 S],flipud(gimg));
            uistack(h_img,'bottom');
            draw_walls(ax,WALLS);
            for g = 1:N_GOALS
                plot(ax,goals(g,1),goals(g,2),'p','MarkerSize',22,...
                    'MarkerFaceColor',gcol(g),'MarkerEdgeColor','w',...
                    'LineWidth',1.5,'HandleVisibility','off');
                text(ax,goals(g,1)+0.2,goals(g,2)+0.2,sprintf('G%d',g),...
                    'Color','w','FontSize',10,'FontName','Courier',...
                    'FontWeight','bold','HandleVisibility','off');
            end
        else
            set(h_img,'CData',flipud(gimg));
        end

        set(h_rob,'XData',x_true(1),'YData',x_true(2));
        set(h_hdg,...
            'XData',[x_true(1), x_true(1)+0.4*cos(x_true(3))],...
            'YData',[x_true(2), x_true(2)+0.4*sin(x_true(3))]);
        traj_x(end+1) = x_true(1); 
        traj_y(end+1) = x_true(2); 
        set(h_traj,'XData',traj_x,'YData',traj_y);
        if size(path_pts,1) >= 2
            set(h_path,'XData',path_pts(:,1),'YData',path_pts(:,2));
        end

        title(ax,sprintf(['\\color{white}Patrolling  '...
            '\\color[rgb]{%.2f %.2f %.2f}Goal %d  '...
            '\\color{white}dist=%.2fm  reached=%d  step=%d'],...
            gcol(goal_idx),goal_idx,d2g,goals_reached,step),...
            'FontSize',11,'FontName','Courier');
        drawnow limitrate;
    end
end

fprintf('\n=== Patrol complete. Goals reached: %d in %d steps ===\n',...
    goals_reached, MAX_STEPS);

% =============================================================
%  HELPER FUNCTIONS
% =============================================================

function draw_walls(ax, walls)
    for w = 1:size(walls,1)
        plot(ax,walls(w,[1,3]),walls(w,[2,4]),'-',...
            'Color',[1 1 1],'LineWidth',2.5,'HandleVisibility','off');
    end
end

function img = make_static_img(wall_inf, n)
% Render static map: red = inflated wall, dark blue = free space.
% Explicitly fill every cell so MATLAB figure background doesn't bleed through.
    img = zeros(n, n, 3);
    % Fill all cells as free (dark blue) first
    img(:,:,1) = 0.10;
    img(:,:,2) = 0.14;
    img(:,:,3) = 0.22;
    % Overwrite wall cells with red
    wall_r = repmat(wall_inf, [1 1 3]);
    wall_colour = cat(3, 0.72*ones(n,n), 0.10*ones(n,n), 0.10*ones(n,n));
    img(wall_r) = wall_colour(wall_r);
end

function img = make_grid_img(lg, n)
    prob = 1./(1+exp(-lg));
    img  = zeros(n,n,3);
    free = prob<0.40; occ = prob>0.60; unk = ~free&~occ;
    img(:,:,1) = 0.10*free + 0.75*occ + 0.20*unk;
    img(:,:,2) = 0.14*free + 0.10*occ + 0.20*unk;
    img(:,:,3) = 0.22*free + 0.10*occ + 0.26*unk;
end

function g = rasterise_walls(walls, res, n)
    g = false(n,n);
    for w = 1:size(walls,1)
        x1=walls(w,1); y1=walls(w,2); x2=walls(w,3); y2=walls(w,4);
        ns = ceil(norm([x2-x1,y2-y1])/res)*8;
        for s = 0:ns
            t = s/max(ns,1);
            px=x1+t*(x2-x1); py=y1+t*(y2-y1);
            [cc,rr] = xy2grid(px,py,res,n);
            if cc>=1&&cc<=n&&rr>=1&&rr<=n; g(rr,cc)=true; end
        end
    end
end

function r = raycast(ox,oy,angle,walls,max_r)
    r=max_r; dx=cos(angle); dy=sin(angle);
    for w = 1:size(walls,1)
        x1=walls(w,1); y1=walls(w,2); x2=walls(w,3); y2=walls(w,4);
        wx=x2-x1; wy=y2-y1; den=dx*wy-dy*wx;
        if abs(den)<1e-10; continue; end
        tx=x1-ox; ty=y1-oy;
        t=(tx*wy-ty*wx)/den; u=(tx*dy-ty*dx)/den;
        if t>1e-6&&t<r&&u>=0&&u<=1; r=t; end
    end
end

function blocked = is_path_blocked(path,grid,res,n)
    blocked = false;
    if isempty(path); return; end
    for i = 1:size(path,1)
        [cc,rr] = xy2grid(path(i,1),path(i,2),res,n);
        cc=max(1,min(n,cc)); rr=max(1,min(n,rr));
        if grid(rr,cc); blocked=true; return; end
    end
end

function [spath,found] = plan_astar(start,goal,grid,res,n)
    spath=[]; found=false;
    [sc,sr] = xy2grid(start(1),start(2),res,n);
    [gc,gr] = xy2grid(goal(1), goal(2), res,n);
    % Flip to image-row convention (row1=top=high-y) for astar_planner
    sr_img = n-sr+1;  gr_img = n-gr+1;
    sr_img=max(1,min(n,sr_img)); sc=max(1,min(n,sc));
    gr_img=max(1,min(n,gr_img)); gc=max(1,min(n,gc));
    % Clear radius around start and goal
    g2=grid; R=5;
    for dr=-R:R
        for dc=-R:R
            if dr^2+dc^2<=R^2
                g2(max(1,min(n,sr_img+dr)),max(1,min(n,sc+dc)))=false;
                g2(max(1,min(n,gr_img+dr)),max(1,min(n,gc+dc)))=false;
            end
        end
    end
    [cells,found] = astar_planner(g2,[sr_img sc],[gr_img gc]);
    if ~found; return; end
    raw = zeros(size(cells,1),2);
    for i = 1:size(cells,1)
        raw(i,1) = (cells(i,2)-0.5)*res;       % col -> x
        raw(i,2) = (n-cells(i,1)+0.5)*res;     % img_row -> y
    end
    % Use local smoother with correct grid orientation (row1=low y)
    spath = smooth_path_local(raw, grid, res, n);
end

function spath = smooth_path_local(raw, grid, res, n)
% Two-stage smoother using the same row1=low-y grid convention as xy2grid.
% Stage 1: greedy shortcutting via LOS check on grid.
% Stage 2: resample at fixed spacing for pure pursuit compatibility.
    if size(raw,1) < 2; spath = raw; return; end

    % Stage 1 — greedy shortcutting
    pruned = raw(1,:);
    i = 1;
    while i < size(raw,1)
        j = size(raw,1);
        while j > i+1
            if los_free(raw(i,:), raw(j,:), grid, res, n)
                break;
            end
            j = j-1;
        end
        pruned(end+1,:) = raw(j,:); %#ok<AGROW>
        i = j;
    end

    % Stage 2 — resample at ~0.2m spacing so pure pursuit has enough points
    SPACING = 0.2;
    spath = pruned(1,:);
    for k = 1:size(pruned,1)-1
        seg_len = norm(pruned(k+1,:) - pruned(k,:));
        n_pts   = max(1, floor(seg_len / SPACING));
        for t = 1:n_pts
            frac = t / n_pts;
            spath(end+1,:) = pruned(k,:)*(1-frac) + pruned(k+1,:)*frac; %#ok<AGROW>
        end
    end
    spath(end+1,:) = pruned(end,:); 
end

function ok = los_free(p1, p2, grid, res, n)
% LOS check using row1=low-y convention (matches xy2grid).
    n_samp = max(10, round(norm(p2-p1)/res * 3));
    ok = true;
    for s = 0:n_samp
        t  = s / n_samp;
        px = p1(1)*(1-t) + p2(1)*t;
        py = p1(2)*(1-t) + p2(2)*t;
        [cc,rr] = xy2grid(px, py, res, n);
        cc = max(1,min(n,cc));
        rr = max(1,min(n,rr));
        if grid(rr,cc)   % row=rr matches low-y convention directly
            ok = false; return;
        end
    end
end


function c = gcol(idx)
    p = [0.30 0.88 1.00; 1.00 0.58 0.15; 0.45 1.00 0.30; 1.00 0.30 0.80];
    c = p(mod(idx-1,size(p,1))+1,:);
end

function [col,row] = xy2grid(wx,wy,res,n)
    col = max(1,min(n,floor(wx/res)+1));
    row = max(1,min(n,floor(wy/res)+1));
end

function ok = valid_cell(c,r,n)
    ok = c>=1&&c<=n&&r>=1&&r<=n;
end