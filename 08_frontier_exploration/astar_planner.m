function path = astar_planner(grid_occ, start_rc, goal_rc)
% ASTAR_PLANNER  8-connected A* on a binary occupancy grid.
%
%   path = astar_planner(grid_occ, start_rc, goal_rc)
%
%   grid_occ  – NR x NC logical/double (1 = obstacle, 0 = free)
%   start_rc  – [row col] start cell
%   goal_rc   – [row col] goal  cell
%
%   path      – Kx2 array of [row col] cells from start to goal,
%               or [] if no path found.
%
%   Uses octile heuristic with 1e-4 tie-breaking.
%   Diagonal moves are blocked if both edge-sharing neighbours are occupied
%   (corner-clipping prevention).

[NR, NC] = size(grid_occ);

% clamp start/goal inside grid
start_rc = max([1 1], min([NR NC], start_rc));
goal_rc  = max([1 1], min([NR NC], goal_rc));

if grid_occ(start_rc(1), start_rc(2)) || grid_occ(goal_rc(1), goal_rc(2))
    path = []; return;
end

if isequal(start_rc, goal_rc)
    path = start_rc; return;
end

% ── data structures ──────────────────────────────────────────────────────
INF = 1e9;
g   = INF(ones(NR, NC));   
g   = inf(NR, NC);
f   = inf(NR, NC);
parent = zeros(NR, NC, 2);   % parent [row col] for each cell
in_closed = false(NR, NC);

sr = start_rc(1);  sc = start_rc(2);
gr = goal_rc(1);   gc = goal_rc(2);

g(sr, sc) = 0;
f(sr, sc) = heuristic(sr, sc, gr, gc);

% min-heap open list as Nx3 [f g r c] — we use a simple sorted array
% (good enough for grids up to ~200x200 in MATLAB)
open = [f(sr,sc), 0, sr, sc];

% 8-connected neighbours: [dr dc is_diagonal]
moves = [-1 -1 1; -1 0 0; -1 1 1;
          0 -1 0;          0 1 0;
          1 -1 1;  1 0 0;  1 1 1];

while ~isempty(open)
    % pop node with smallest f
    [~, idx] = min(open(:,1));
    cur = open(idx, :);
    open(idx, :) = [];

    cr = cur(3);  cc_col = cur(4);

    if in_closed(cr, cc_col), continue; end
    in_closed(cr, cc_col) = true;

    if cr == gr && cc_col == gc
        path = reconstruct(parent, start_rc, goal_rc);
        return;
    end

    for m = 1:8
        dr = moves(m,1);  dc = moves(m,2);  is_diag = moves(m,3);
        nr = cr + dr;     nc2 = cc_col + dc;

        if nr < 1 || nr > NR || nc2 < 1 || nc2 > NC, continue; end
        if grid_occ(nr, nc2),  continue; end
        if in_closed(nr, nc2), continue; end

        % corner-clipping prevention for diagonals
        if is_diag && grid_occ(cr + dr, cc_col) && grid_occ(cr, cc_col + dc)
            continue;
        end

        step_cost = 1 + (is_diag * (sqrt(2) - 1));
        new_g = g(cr, cc_col) + step_cost;

        if new_g < g(nr, nc2)
            g(nr, nc2) = new_g;
            h = heuristic(nr, nc2, gr, gc);
            new_f = new_g + h * (1 + 1e-4);
            f(nr, nc2) = new_f;
            parent(nr, nc2, :) = [cr, cc_col];
            open(end+1, :) = [new_f, new_g, nr, nc2]; %#ok<AGROW>
        end
    end
end

path = [];  % no path found

end

% ── helpers ──────────────────────────────────────────────────────────────
function h = heuristic(r, c, gr, gc)
    dr = abs(r - gr);  dc = abs(c - gc);
    h  = max(dr, dc) + (sqrt(2)-1) * min(dr, dc);
end

function path = reconstruct(parent, start_rc, goal_rc)
    path = goal_rc;
    cur  = goal_rc;
    while ~isequal(cur, start_rc)
        p = squeeze(parent(cur(1), cur(2), :))';
        path = [p; path]; %#ok<AGROW>
        cur  = p;
    end
end