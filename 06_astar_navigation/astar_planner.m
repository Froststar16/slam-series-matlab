function [path, found] = astar_planner(grid, start, goal)
% ASTAR_PLANNER  A* search on a binary occupancy grid.
%
%   [path, found] = astar_planner(grid, start, goal)
%
%   Inputs:
%     grid   — NxN binary matrix (1 = occupied / inflated obstacle)
%     start  — [row, col] start cell
%     goal   — [row, col] goal cell
%
%   Outputs:
%     path   — Px2 matrix of [row, col] waypoints (start → goal)
%     found  — true if a valid path was found
%
%   Notes:
%     • 8-connected neighbourhood (diagonal moves cost √2).
%     • Heuristic: octile distance (admissible for 8-connectivity).
%     • Tie-breaking: slight h-bias to prefer paths that head toward goal.

    [N, M]   = size(grid);
    found    = false;
    path     = [];

    % Validate start/goal
    if grid(start(1),start(2)) || grid(goal(1),goal(2))
        warning('astar_planner: start or goal cell is occupied.');
        return;
    end
    if isequal(start, goal)
        path = start; found = true; return;
    end

    % ── Data structures ──────────────────────────────────────
    INF     = 1e9;
    g_cost  = INF * ones(N, M);
    f_cost  = INF * ones(N, M);
    parent  = zeros(N, M, 2);    % [row, col] of parent cell
    in_open = false(N, M);
    closed  = false(N, M);

    g_cost(start(1),start(2)) = 0;
    f_cost(start(1),start(2)) = octile(start, goal);
    in_open(start(1),start(2)) = true;

    % Open list as [f, g, row, col] rows — use a simple sorted array.
    % Good enough for grids up to ~500×500.
    open_list = [f_cost(start(1),start(2)), 0, start(1), start(2)];

    % 8-connected neighbours and their costs
    moves   = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    m_costs = [sqrt(2);1;sqrt(2);1;1;sqrt(2);1;sqrt(2)];

    % ── Main loop ────────────────────────────────────────────
    while ~isempty(open_list)
        % Pop node with smallest f (first row — keep list sorted)
        cur  = open_list(1, 3:4);
        open_list(1,:) = [];
        cr = cur(1);  cc = cur(2);

        if closed(cr,cc); continue; end
        closed(cr,cc)  = true;
        in_open(cr,cc) = false;

        % Goal check
        if cr == goal(1) && cc == goal(2)
            path  = reconstruct(parent, start, goal);
            found = true;
            return;
        end

        % Expand neighbours
        for n = 1:8
            nr = cr + moves(n,1);
            nc = cc + moves(n,2);
            if nr<1||nr>N||nc<1||nc>M; continue; end
            if closed(nr,nc) || grid(nr,nc);   continue; end

            % Diagonal safety: block if both axis-neighbours are occupied
            if abs(moves(n,1))==1 && abs(moves(n,2))==1
                if grid(cr,nc) && grid(nr,cc); continue; end
            end

            tentative_g = g_cost(cr,cc) + m_costs(n);
            if tentative_g < g_cost(nr,nc)
                g_cost(nr,nc) = tentative_g;
                parent(nr,nc,:) = [cr, cc];
                % Tie-breaking: tiny nudge toward goal in h
                h = octile([nr,nc], goal) * (1 + 1e-4);
                f = tentative_g + h;
                f_cost(nr,nc) = f;
                % Insert into open list (insertion sort by f)
                new_row = [f, tentative_g, nr, nc];
                if isempty(open_list)
                    open_list = new_row;
                else
                    idx = find(open_list(:,1) > f, 1);
                    if isempty(idx)
                        open_list = [open_list; new_row];
                    else
                        open_list = [open_list(1:idx-1,:); ...
                                     new_row; ...
                                     open_list(idx:end,:)];
                    end
                end
                in_open(nr,nc) = true;
            end
        end
    end
    % If we exit the loop, no path exists
end

% ── Octile distance heuristic ────────────────────────────────
function h = octile(a, b)
    dx = abs(a(1)-b(1));
    dy = abs(a(2)-b(2));
    h  = max(dx,dy) + (sqrt(2)-1)*min(dx,dy);
end

% ── Reconstruct path by tracing parents ──────────────────────
function path = reconstruct(parent, start, goal)
    path = goal;
    cur  = goal;
    while ~isequal(cur, start)
        p   = squeeze(parent(cur(1),cur(2),:))';
        path = [p; path]; %#ok
        cur  = p;
    end
end