function frontiers = frontier_detector(log_odds, res, origin)
% FRONTIER_DETECTOR  Detect, cluster, and score frontier segments.
%
%   frontiers = frontier_detector(log_odds, res, origin)
%
%   Inputs:
%     log_odds  – NR x NC occupancy log-odds grid
%     res       – metres per cell (scalar)
%     origin    – [x0 y0] world coords of grid cell (1,1)
%
%   Output:
%     frontiers – struct array, each element:
%       .centroid_w   [x y] world coords of segment centroid
%       .centroid_rc  [r c] grid coords of centroid
%       .size         number of frontier cells in segment
%
%   A frontier cell is a FREE cell (log_odds < FREE_TH) that has at least
%   one UNKNOWN neighbour (abs(log_odds) < UNK_TH).  Contiguous frontier
%   cells are grouped by connected-components (4-connected).
%
%   Called every FRONTIER_INTERVAL steps from the main script.

FREE_TH = -0.1;   % log-odds below this → free
UNK_TH  =  0.1;   % |log-odds| below this → unknown

[NR, NC] = size(log_odds);

% ── 1. mark frontier cells ──────────────────────────────────────────────
is_free = log_odds < FREE_TH;
is_unk  = abs(log_odds) < UNK_TH;

% dilate unknown mask by 1 to find free cells touching unknown
se = strel('square', 3);
unk_dilated = imdilate(is_unk, se);

frontier_mask = is_free & unk_dilated;

% remove border pixels (avoids edge artefacts)
frontier_mask(1,:) = false;  frontier_mask(end,:) = false;
frontier_mask(:,1) = false;  frontier_mask(:,end) = false;

if ~any(frontier_mask(:))
    frontiers = struct('centroid_w', {}, 'centroid_rc', {}, 'size', {});
    return;
end

% ── 2. connected-component clustering (4-connected) ────────────────────
CC = bwconncomp(frontier_mask, 4);
n_seg = CC.NumObjects;

if n_seg == 0
    frontiers = struct('centroid_w', {}, 'centroid_rc', {}, 'size', {});
    return;
end

% ── 3. compute centroid + size for each segment ─────────────────────────
frontiers(n_seg) = struct('centroid_w', [], 'centroid_rc', [], 'size', []);

for k = 1:n_seg
    idx = CC.PixelIdxList{k};
    [rr, cc] = ind2sub([NR NC], idx);

    % centroid in grid coords
    rc = round(mean(rr));
    c_col = round(mean(cc));
    rc = max(1, min(NR, rc));
    c_col = max(1, min(NC, c_col));

    % convert to world coords
    wx = origin(1) + (c_col - 0.5) * res;
    wy = origin(2) + (NR - rc + 0.5) * res;   % flip row→y

    frontiers(k).centroid_w  = [wx, wy];
    frontiers(k).centroid_rc = [rc, c_col];
    frontiers(k).size        = numel(idx);
end

end