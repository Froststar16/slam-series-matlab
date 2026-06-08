%% EKF SLAM - 2D LiDAR Simulation
% Project: Extended Kalman Filter SLAM in a simulated 2D environment
%
% Robot motion model : unicycle (v, omega)
% Sensor model       : range + bearing LiDAR
% State vector       : [x, y, theta, m1x, m1y, ..., mNx, mNy]'
%


clear; clc; close all;

%% ── Parameters ──────────────────────────────────────────────────────────
rng(42);                        % reproducible noise

DT          = 0.1;              % timestep [s]
N_STEPS     = 300;              % total simulation steps
LIDAR_RANGE = 3.5;              % max sensing range [m]
WORLD_SIZE  = 9;                % world is [0, WORLD_SIZE] x [0, WORLD_SIZE]

% Noise standard deviations
SIG_V   = 0.08;                 % linear velocity noise
SIG_W   = 0.04;                 % angular velocity noise
SIG_R   = 0.06;                 % range measurement noise
SIG_PHI = 0.03;                 % bearing measurement noise

% Process noise matrix Q (robot block only — expanded per new landmark)
Q_robot = diag([SIG_V^2, SIG_V^2, SIG_W^2]);

% Sensor noise matrix R
R_sensor = diag([SIG_R^2, SIG_PHI^2]);

% Data association threshold (Mahalanobis-like distance)
ASSOC_THRESH = 0.8;

%% ── True landmark positions ─────────────────────────────────────────────
true_landmarks = [
    2,2; 4,1; 5,3; 3,5; 1,4;
    6,2; 7,4; 5,6; 2,7; 7,7;
    0.5,6; 4,8; 8,1; 8,5; 1,1
];
N_LM_TRUE = size(true_landmarks, 1);

%% ── Waypoints for robot path ─────────────────────────────────────────────
waypoints = [1,1; 7,1; 7,8; 1,8; 1,1; 4,4; 7,1; 1,8; 7,8; 1,1];
wp_idx    = 1;

%% ── Initial state ────────────────────────────────────────────────────────
% True robot state
true_state = [0.5; 0.5; pi/4];

% EKF state: mu = [x, y, theta]' initially
mu    = [0.5; 0.5; pi/4];
Sigma = zeros(3, 3);

% Landmark registry: tracks which true LM index maps to EKF state index
lm_registry = containers.Map('KeyType','int32','ValueType','int32');
% lm_registry(true_lm_id) = ekf_landmark_index (0-based)

%% ── Logging ──────────────────────────────────────────────────────────────
true_path = zeros(N_STEPS+1, 2);
ekf_path  = zeros(N_STEPS+1, 2);
pose_error = zeros(N_STEPS, 1);
n_landmarks_discovered = zeros(N_STEPS, 1);

true_path(1,:) = true_state(1:2)';
ekf_path(1,:)  = mu(1:2)';

%% ── Figure setup ─────────────────────────────────────────────────────────
fig = figure('Name','EKF SLAM','Color','white','Position',[100 100 1100 520]);
tiledlayout(1, 2, 'TileSpacing','compact','Padding','compact');

ax_map  = nexttile; hold(ax_map,  'on'); axis(ax_map,  'equal');
ax_plot = nexttile; hold(ax_plot, 'on');

setup_map_axes(ax_map,  WORLD_SIZE);
setup_plot_axes(ax_plot, N_STEPS);

%% ══════════════════════════════════════════════════════════════════════════
%% MAIN SIMULATION LOOP
%% ══════════════════════════════════════════════════════════════════════════
for t = 1:N_STEPS

    %% CONTROL INPUT — navigate toward current waypoint ─────────────────
    [v, w] = compute_control(true_state, waypoints, wp_idx, SIG_V, SIG_W);
    % Advance waypoint if close enough
    if norm(true_state(1:2) - waypoints(wp_idx,:)') < 0.3
        wp_idx = mod(wp_idx, size(waypoints,1)) + 1;
    end

    %% TRUE MOTION (with noise) ─────────────────────────────────────────
    true_state = motion_model(true_state, v + randn*SIG_V, ...
                                          w + randn*SIG_W, DT);

    %% EKF PREDICT ──────────────────────────────────────────────────────
    [mu, Sigma] = ekf_predict(mu, Sigma, v, w, DT, Q_robot);

    %% LIDAR OBSERVATIONS ───────────────────────────────────────────────
    [obs, rays] = simulate_lidar(true_state, true_landmarks, ...
                                 LIDAR_RANGE, SIG_R, SIG_PHI);
    % obs: [true_lm_id, range_noisy, bearing_noisy] per row

    %% EKF UPDATE ───────────────────────────────────────────────────────
    [mu, Sigma, lm_registry] = ekf_update(mu, Sigma, obs, lm_registry, ...
                                           R_sensor, ASSOC_THRESH);

    %% LOG ──────────────────────────────────────────────────────────────
    true_path(t+1,:) = true_state(1:2)';
    ekf_path(t+1,:)  = mu(1:2)';
    pose_error(t)    = norm(true_state(1:2) - mu(1:2));
    n_landmarks_discovered(t) = (length(mu) - 3) / 2;

    %% VISUALISE ────────────────────────────────────────────────────────
    if mod(t, 2) == 0  % draw every 2 steps for speed
        draw_frame(ax_map, ax_plot, ...
                   true_state, mu, Sigma, ...
                   true_path(1:t+1,:), ekf_path(1:t+1,:), ...
                   true_landmarks, rays, ...
                   pose_error(1:t), n_landmarks_discovered(1:t), t);
        drawnow limitrate;
    end
end

fprintf('Simulation complete. Landmarks found: %d / %d\n', ...
        (length(mu)-3)/2, N_LM_TRUE);
fprintf('Final pose error: %.4f m\n', pose_error(end));


%% ══════════════════════════════════════════════════════════════════════════
%% LOCAL FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════════

%% ── Motion model (unicycle) ──────────────────────────────────────────────
function s = motion_model(s, v, w, dt)
    th  = s(3);
    s(1) = s(1) + v * cos(th + w*dt/2) * dt;
    s(2) = s(2) + v * sin(th + w*dt/2) * dt;
    s(3) = wrap_angle(s(3) + w*dt);
end

%% ── EKF Predict ──────────────────────────────────────────────────────────
function [mu, Sigma] = ekf_predict(mu, Sigma, v, w, dt, Q_robot)
    x  = mu(1); y = mu(2); th = mu(3);
    n  = length(mu);

    % Propagate robot pose
    mu(1) = x + v * cos(th + w*dt/2) * dt;
    mu(2) = y + v * sin(th + w*dt/2) * dt;
    mu(3) = wrap_angle(mu(3) + w*dt);

    % Jacobian of motion w.r.t. state (only robot block is non-identity)
    G = eye(n);
    G(1,3) = -v * sin(th + w*dt/2) * dt;
    G(2,3) =  v * cos(th + w*dt/2) * dt;

    % Expand Q to full state dimension
    Q_full = zeros(n);
    Q_full(1:3, 1:3) = Q_robot;

    Sigma = G * Sigma * G' + Q_full;
end

%% ── Simulate LiDAR ───────────────────────────────────────────────────────
function [obs, rays] = simulate_lidar(state, landmarks, range, sig_r, sig_phi)
    rx = state(1); ry = state(2); rth = state(3);
    obs  = [];
    rays = [];
    for i = 1:size(landmarks,1)
        dx = landmarks(i,1) - rx;
        dy = landmarks(i,2) - ry;
        r  = sqrt(dx^2 + dy^2);
        if r > range, continue; end
        phi = wrap_angle(atan2(dy,dx) - rth);
        obs  = [obs;  i, r + randn*sig_r, wrap_angle(phi + randn*sig_phi)]; %#ok
        rays = [rays; rx, ry, landmarks(i,1), landmarks(i,2)];              %#ok
    end
end

%% ── Observation model ────────────────────────────────────────────────────
function z = obs_model(rx, ry, rth, mx, my)
    dx  = mx - rx;
    dy  = my - ry;
    r   = sqrt(dx^2 + dy^2);
    phi = wrap_angle(atan2(dy,dx) - rth);
    z   = [r; phi];
end

%% ── Observation Jacobian ─────────────────────────────────────────────────
function H = obs_jacobian(rx, ry, ~, mx, my)
    dx  = mx - rx;
    dy  = my - ry;
    r2  = dx^2 + dy^2;
    r   = sqrt(r2);
    H   = [-dx/r,  -dy/r,   0,  dx/r,  dy/r;
            dy/r2, -dx/r2, -1, -dy/r2, dx/r2];
end

%% ── EKF Update ───────────────────────────────────────────────────────────
function [mu, Sigma, lm_reg] = ekf_update(mu, Sigma, obs, lm_reg, R, thresh)
    if isempty(obs), return; end
    n_ekf_lm = (length(mu) - 3) / 2;   % current EKF landmark count

    for k = 1:size(obs, 1)
        true_id = obs(k,1);
        r_obs   = obs(k,2);
        phi_obs = obs(k,3);
        z_obs   = [r_obs; phi_obs];

        % ── Data association ────────────────────────────────────────────
        if isKey(lm_reg, int32(true_id))
            % Known landmark
            ekf_idx = lm_reg(int32(true_id));
        else
            % Check Mahalanobis-like distance against all known landmarks
            best_id   = -1;
            best_dist = thresh;
            for j = 0:(n_ekf_lm - 1)
                mi = 3 + j*2 + 1;
                mx = mu(mi); my = mu(mi+1);
                z_hat = obs_model(mu(1),mu(2),mu(3),mx,my);
                innov = [r_obs - z_hat(1); wrap_angle(phi_obs - z_hat(2))];
                d     = sqrt(innov(1)^2 + innov(2)^2*4);
                if d < best_dist
                    best_dist = d;
                    best_id   = j;
                end
            end

            if best_id >= 0
                % Associate with existing EKF landmark
                ekf_idx = best_id;
                lm_reg(int32(true_id)) = ekf_idx;
            else
                % New landmark — initialise
                lm_x = mu(1) + r_obs * cos(phi_obs + mu(3));
                lm_y = mu(2) + r_obs * sin(phi_obs + mu(3));
                mu   = [mu; lm_x; lm_y];
                n    = length(mu);
                Sigma_new           = zeros(n);
                Sigma_new(1:n-2,1:n-2) = Sigma;
                Sigma_new(n-1,n-1)  = 1.0;   % initial landmark uncertainty
                Sigma_new(n,  n  )  = 1.0;
                Sigma    = Sigma_new;
                ekf_idx  = n_ekf_lm;
                n_ekf_lm = n_ekf_lm + 1;
                lm_reg(int32(true_id)) = ekf_idx;
            end
        end

        % ── Kalman update ────────────────────────────────────────────────
        mi     = 3 + ekf_idx*2 + 1;
        mx     = mu(mi); my = mu(mi+1);
        z_hat  = obs_model(mu(1), mu(2), mu(3), mx, my);
        innov  = [r_obs - z_hat(1); wrap_angle(phi_obs - z_hat(2))];

        H_small = obs_jacobian(mu(1), mu(2), mu(3), mx, my);
        n       = length(mu);
        H       = zeros(2, n);
        H(:,1:3)       = H_small(:,1:3);
        H(:,mi:mi+1)   = H_small(:,4:5);

        S = H * Sigma * H' + R;
        K = Sigma * H' / S;

        mu    = mu + K * innov;
        mu(3) = wrap_angle(mu(3));
        Sigma = (eye(n) - K*H) * Sigma;
    end
end

%% ── Control law ──────────────────────────────────────────────────────────
function [v, w] = compute_control(state, waypoints, wp_idx, ~, ~)
    dx   = waypoints(wp_idx,1) - state(1);
    dy   = waypoints(wp_idx,2) - state(2);
    dist = sqrt(dx^2 + dy^2);
    dth  = wrap_angle(atan2(dy,dx) - state(3));
    v    = min(0.5, dist) * 0.8;
    w    = dth * 2.0;
end

%% ── Angle wrapping ───────────────────────────────────────────────────────
function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

%% ── Draw frame ───────────────────────────────────────────────────────────
function draw_frame(ax1, ax2, true_st, mu, Sigma, ...
                    true_path, ekf_path, true_lm, rays, ...
                    pose_err, n_lm, t)
    cla(ax1);
    setup_map_axes(ax1, 9);

    % LiDAR rays
    for i = 1:size(rays,1)
        plot(ax1, [rays(i,1) rays(i,3)], [rays(i,2) rays(i,4)], ...
             'Color',[0 0.75 0.85 0.35], 'LineWidth', 0.7);
    end

    % True path
    plot(ax1, true_path(:,1), true_path(:,2), 'g-', 'LineWidth',1.2);
    % EKF path
    plot(ax1, ekf_path(:,1),  ekf_path(:,2),  'b-', 'LineWidth',1.2);

    % True landmarks
    plot(ax1, true_lm(:,1), true_lm(:,2), '^', ...
         'Color',[1 0.6 0], 'MarkerFaceColor',[1 0.6 0], 'MarkerSize',8);

    % EKF landmarks + covariance ellipses
    n_ekf_lm = (length(mu) - 3) / 2;
    for j = 0:n_ekf_lm-1
        mi = 3 + j*2 + 1;
        mx = mu(mi); my = mu(mi+1);
        plot(ax1, mx, my, 'p', 'Color',[0.9 0.1 0.4], ...
             'MarkerFaceColor',[0.9 0.1 0.4], 'MarkerSize',7);
        % 3-sigma covariance ellipse
        S_lm = Sigma(mi:mi+1, mi:mi+1);
        draw_ellipse(ax1, mx, my, S_lm, 3, [0.6 0.6 0.6]);
    end

    % True robot
    plot(ax1, true_st(1), true_st(2), 'go', ...
         'MarkerFaceColor','g', 'MarkerSize',9);
    draw_robot_arrow(ax1, true_st, 'g');

    % EKF robot estimate + pose ellipse
    plot(ax1, mu(1), mu(2), 'bo', ...
         'MarkerFaceColor','b', 'MarkerSize',8);
    draw_robot_arrow(ax1, mu(1:3), 'b');
    draw_ellipse(ax1, mu(1), mu(2), Sigma(1:2,1:2), 3, [0.2 0.5 0.9]);

    title(ax1, sprintf('EKF SLAM  |  Step %d  |  LMs found: %d', t, n_ekf_lm), ...
          'FontSize',11);

    % Error + landmark count plots
    cla(ax2);
    yyaxis(ax2, 'left');
    plot(ax2, 1:t, pose_err(1:t), 'b-', 'LineWidth',1.2);
    ylabel(ax2, 'Pose error [m]', 'Color','b');

    yyaxis(ax2, 'right');
    plot(ax2, 1:t, n_lm(1:t), 'r-', 'LineWidth',1.2);
    ylabel(ax2, 'Landmarks found', 'Color','r');
    xlabel(ax2, 'Time step');
    title(ax2, 'Estimation error & map growth', 'FontSize',11);
    xlim(ax2, [0, length(pose_err)]);
    grid(ax2, 'on');
end

%% ── Draw covariance ellipse ──────────────────────────────────────────────
function draw_ellipse(ax, cx, cy, S, n_sigma, color)
    [V, D] = eig(S);
    angles = linspace(0, 2*pi, 60);
    ell    = n_sigma * V * sqrt(D) * [cos(angles); sin(angles)];
    plot(ax, cx + ell(1,:), cy + ell(2,:), '--', ...
         'Color', [color, 0.5], 'LineWidth', 0.8);
end

%% ── Draw robot heading arrow ─────────────────────────────────────────────
function draw_robot_arrow(ax, state, color)
    x  = state(1); y = state(2); th = state(3);
    quiver(ax, x, y, 0.25*cos(th), 0.25*sin(th), 0, ...
           'Color', color, 'LineWidth', 1.5, 'MaxHeadSize', 2);
end

%% ── Axis setup ───────────────────────────────────────────────────────────
function setup_map_axes(ax, ws)
    axis(ax, [0 ws 0 ws]); grid(ax, 'on');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    set(ax,'FontSize',10,'Box','on');
    legend(ax, {'LiDAR','True path','EKF path','True LM','Est. LM'}, ...
           'Location','northeast','FontSize',8,'AutoUpdate','off');
end

function setup_plot_axes(ax, n_steps)
    xlim(ax, [0 n_steps]); grid(ax, 'on');
    set(ax,'FontSize',10,'Box','on');
end