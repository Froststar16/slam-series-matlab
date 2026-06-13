function poses_matlab = validate_with_pose_graph_toolbox(poses_init, edges)
%VALIDATE_WITH_POSE_GRAPH_TOOLBOX  Optimize the pose-only sub-graph
%(odometry + loop-closure edges, no landmarks) with MATLAB's built-in
%optimizePoseGraph, for comparison against the from-scratch Gauss-Newton
%result on the same edge set.
%
%   poses_matlab = VALIDATE_WITH_POSE_GRAPH_TOOLBOX(poses_init, edges)
%
%   edges should contain ONLY 'odom' and 'loop' type entries (filter out
%   'obs' edges before calling this -- the Navigation Toolbox pose graph
%   has no concept of landmark nodes).
%
%   NOTE: this relies on the Navigation Toolbox poseGraph / addRelativePose
%   / optimizePoseGraph API. If the exact call signatures differ slightly
%   from what's below in your R2024a install, this is the file to debug --
%   the surrounding pipeline doesn't depend on its internals.

N = size(poses_init,1);

% poseGraph (Navigation Toolbox) vs robotics.PoseGraph (older Robotics
% System Toolbox naming) -- try both before giving up.
pg = [];
try
    pg = poseGraph;   % 2D pose graph, node 1 = origin
catch
end
if isempty(pg)
    try
        pg = robotics.PoseGraph;
    catch
        error('validate_with_pose_graph_toolbox:noToolbox', ...
            ['Neither poseGraph nor robotics.PoseGraph could be constructed. ' ...
             'Navigation Toolbox / Robotics System Toolbox may not be installed ' ...
             'or licensed on this MATLAB. Run "ver" to check installed toolboxes.']);
    end
end

% Sequential odometry edges create nodes 2..N in order.
odom_edges = edges(strcmp({edges.type},'odom'));  % already in order i = 1..N-1
for e = 1:numel(odom_edges)
    z = odom_edges(e).z;
    addRelativePose(pg, [z(1) z(2) z(3)], odom_edges(e).omega);
end

% Loop-closure edges connect two EXISTING nodes -> pass [fromID toID].
loop_edges = edges(strcmp({edges.type},'loop'));
for e = 1:numel(loop_edges)
    z = loop_edges(e).z;
    addRelativePose(pg, [z(1) z(2) z(3)], loop_edges(e).omega, ...
        [loop_edges(e).i, loop_edges(e).j]);
end

optPG = optimizePoseGraph(pg);

poses_matlab = nodeEstimates(optPG);   % Nx3 [x y theta]

if size(poses_matlab,1) ~= N
    warning('validate_with_pose_graph_toolbox:nodeCountMismatch', ...
        'optimizePoseGraph returned %d nodes, expected %d.', size(poses_matlab,1), N);
end

end