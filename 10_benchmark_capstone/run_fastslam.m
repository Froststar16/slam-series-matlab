function result = run_fastslam(data, n_particles)
%RUN_FASTSLAM  FastSLAM 1.0 -- particle filter with per-particle landmark EKFs.
%
%   result = RUN_FASTSLAM(data, n_particles)
%
%   data          struct from generate_benchmark_data
%   n_particles   number of particles (default 50, matching Module 03)
%
%   Each particle carries its own pose hypothesis (propagated with its own
%   sampled odometry noise) and its own set of per-landmark 2x2 EKFs
%   (mu, Sigma), conditioned on that particle's pose. Data association is
%   per-particle, nearest-Mahalanobis-distance with a chi-square gate, and
%   has NO time-window restriction -- like Module 02, a revisit naturally
%   re-associates with old landmarks. The correction mechanism is
%   different though: here it shows up as particle weight differences
%   (particles whose maps are consistent with the revisit survive
%   resampling, others die out), rather than a single EKF state snapping
%   back.
%
%   result.poses          Nx3 weighted-mean trajectory
%   result.landmarks      Kx2 landmark positions (highest-weight particle)
%   result.runtime_s      wall-clock seconds
%   result.num_landmarks  K (highest-weight particle's landmark count)

if nargin < 2, n_particles = 50; end

N    = size(data.gt_poses,1);
obs  = data.observations;
odom = data.odom;

odom_sigma = data.params.odom_sigma;
R_meas     = diag(data.params.obs_sigma.^2);
GATE       = 9.21;   % chi-square 99% threshold for 2 DoF

pose0 = data.gt_poses(1,:);

particles = struct('pose',{},'lm_mu',{},'lm_sigma',{},'w',{});
for i = 1:n_particles
    particles(i).pose     = pose0;
    particles(i).lm_mu    = zeros(0,2);    % Kx2
    particles(i).lm_sigma = cell(0,1);     % K cells of 2x2
    particles(i).w        = 1/n_particles;
end

poses_est = zeros(N,3);
poses_est(1,:) = pose0;

tic;
for k = 1:N
    % ---- motion update: each particle gets its own noisy odometry draw ----
    if k > 1
        u = odom(k-1,:);
        for i = 1:n_particles
            u_i = u + odom_sigma .* randn(1,3);
            particles(i).pose = compose_pose(particles(i).pose, u_i);
        end
    end

    % ---- measurement update: accumulate log-weight over all obs at pose k ----
    rows = obs(obs(:,1)==k, :);
    if ~isempty(rows)
        for i = 1:n_particles
            log_w_total = 0;
            for r = 1:size(rows,1)
                z = rows(r,3:4);
                [particles(i), log_lik] = fs_update_particle(particles(i), z, R_meas, GATE);
                log_w_total = log_w_total + log_lik;
            end
            particles(i).w = particles(i).w * exp(log_w_total);
        end

        ws = [particles.w];
        s  = sum(ws);
        if s < 1e-300
            ws = ones(1,n_particles)/n_particles;
        else
            ws = ws/s;
        end
        for i = 1:n_particles, particles(i).w = ws(i); end

        % ---- low-variance resampling if effective sample size is low ----
        if 1/sum(ws.^2) < n_particles/2
            particles = resample_particles(particles);
        end
    end

    poses_est(k,:) = weighted_mean_pose(particles);
end
runtime_s = toc;

[~, best_i] = max([particles.w]);
landmarks_est = particles(best_i).lm_mu;

result.poses         = poses_est;
result.landmarks     = landmarks_est;
result.runtime_s     = runtime_s;
result.num_landmarks = size(landmarks_est,1);

end

%% ---------- local helper functions ----------

function [p, log_lik] = fs_update_particle(p, z, R_meas, GATE)
% Nearest-Mahalanobis association against this particle's landmark map,
% gated; either a 2x2 EKF landmark update or a new-landmark spawn.

theta = p.pose(3);
c = cos(theta); s = sin(theta);
RT = [c s; -s c];          % R(theta)' -- d(prediction)/d(landmark)
R_global = [c -s; s c];     % R(theta)  -- robot frame -> global frame

K_lm = size(p.lm_mu,1);
best_d2 = inf; best_idx = -1; best_innov = []; best_H = []; best_S = [];

for j = 1:K_lm
    d    = (p.lm_mu(j,:) - p.pose(1:2))';
    pred = (RT*d)';
    innov = z - pred;

    H = RT;
    S = H*p.lm_sigma{j}*H' + R_meas;
    d2 = (innov/S)*innov';

    if d2 < best_d2
        best_d2 = d2; best_idx = j; best_innov = innov; best_H = H; best_S = S;
    end
end

if best_idx > 0 && best_d2 < GATE
    K = (p.lm_sigma{best_idx}*best_H')/best_S;
    p.lm_mu(best_idx,:) = p.lm_mu(best_idx,:) + (K*best_innov')';
    p.lm_sigma{best_idx} = (eye(2) - K*best_H)*p.lm_sigma{best_idx};
    log_lik = -0.5*(log(max(det(2*pi*best_S),1e-12)) + best_d2);
else
    lm_pos = p.pose(1:2) + (R_global*z')';
    Sigma_new = R_global*R_meas*R_global';
    p.lm_mu(end+1,:) = lm_pos;
    p.lm_sigma{end+1,1} = Sigma_new;
    log_lik = -2;   % fixed penalty for spawning a new landmark
end
end

function particles = resample_particles(particles)
% Low-variance (systematic) resampling.
M  = numel(particles);
ws = [particles.w];
edges = cumsum(ws);
edges(end) = 1;

idx = zeros(1,M);
u0  = rand/M;
i = 1;
for j = 1:M
    uu = u0 + (j-1)/M;
    while uu > edges(i) && i < M
        i = i+1;
    end
    idx(j) = i;
end

particles = particles(idx);
for i = 1:M, particles(i).w = 1/M; end
end

function pose = weighted_mean_pose(particles)
ws = [particles.w];
xy = [0 0]; sa = 0; ca = 0;
for i = 1:numel(particles)
    xy = xy + ws(i)*particles(i).pose(1:2);
    sa = sa + ws(i)*sin(particles(i).pose(3));
    ca = ca + ws(i)*cos(particles(i).pose(3));
end
pose = [xy, atan2(sa,ca)];
end

function pb = compose_pose(pa, z)
c = cos(pa(3)); s = sin(pa(3));
dx = c*z(1) - s*z(2);
dy = s*z(1) + c*z(2);
pb = [pa(1)+dx, pa(2)+dy, wrap_angle(pa(3)+z(3))];
end

function a = wrap_angle(a)
a = mod(a+pi,2*pi)-pi;
end