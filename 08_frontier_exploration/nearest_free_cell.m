function [fr, fc] = nearest_free_cell(occ_grid, r, c, NR, NC)
% NEAREST_FREE_CELL  Find the nearest unoccupied cell to (r,c).
%
%   [fr, fc] = nearest_free_cell(occ_grid, r, c, NR, NC)
%
%   Spirals outward from (r,c) in a square shell pattern until it finds
%   a cell where occ_grid == 0.  Returns the original cell if it is
%   already free, or if no free cell is found within 15 cells.
%
%   Used to recover a valid A* start/goal when dead-reckoning drift has
%   walked the robot's estimated position into an inflated obstacle cell.

MAX_RADIUS = 15;

if ~occ_grid(r, c)
    fr = r;  fc = c;
    return;
end

for radius = 1:MAX_RADIUS
    % iterate over the square shell at this radius
    for dr = -radius:radius
        for dc = -radius:radius
            % only process cells on the shell boundary
            if abs(dr) ~= radius && abs(dc) ~= radius, continue; end
            nr = r + dr;
            nc = c + dc;
            if nr < 1 || nr > NR || nc < 1 || nc > NC, continue; end
            if ~occ_grid(nr, nc)
                fr = nr;  fc = nc;
                return;
            end
        end
    end
end

% fallback: return original (A* will fail gracefully with [])
fr = r;  fc = c;
end