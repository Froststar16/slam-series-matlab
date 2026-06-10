function save_nav_gif(fig, smooth_path, h_robot, h_trace, ax, spd, filename)
% SAVE_NAV_GIF  Export the navigation execution as an animated GIF.
%
%   Uses the print() → PNG → imwrite() pipeline — reliable on MATLAB
%   R2024a where getframe() silently fails with the OpenGL renderer.
%
%   The function re-runs the robot animation at 2× speed and captures
%   every Nth frame to keep the GIF file size manageable.

    tmp_dir   = fullfile(tempdir, 'nav_gif_frames');
    if ~exist(tmp_dir,'dir'); mkdir(tmp_dir); end

    N_WP      = size(smooth_path, 1);
    FRAME_SKIP = max(1, floor(N_WP / 60));  % target ~60 frames
    delay      = max(0.04, spd * FRAME_SKIP * 0.5);

    frame_files = {};
    frame_idx   = 0;
    trace_x     = smooth_path(1,1);
    trace_y     = smooth_path(1,2);

    for i = 2:N_WP
        trace_x(end+1) = smooth_path(i,1); %#ok
        trace_y(end+1) = smooth_path(i,2); %#ok
        set(h_trace, 'XData', trace_x, 'YData', trace_y);
        set(h_robot, 'XData', smooth_path(i,1), 'YData', smooth_path(i,2));

        progress = i / N_WP;
        title(ax, sprintf('\\color{white}Executing path  [%.0f%%]', progress*100),...
            'FontSize',12,'FontName','Courier');

        drawnow;

        if mod(i, FRAME_SKIP) == 0 || i == N_WP
            frame_idx  = frame_idx + 1;
            png_path   = fullfile(tmp_dir, sprintf('frame_%04d.png', frame_idx));
            print(fig, '-dpng', '-r80', png_path);
            frame_files{end+1} = png_path; %#ok
        end
    end

    if isempty(frame_files)
        warning('save_nav_gif: no frames captured.');
        return;
    end

    % Assemble GIF
    for f = 1:numel(frame_files)
        [img, cmap] = imread(frame_files{f});
        [idx, map]  = rgb2ind(img, 256);
        if f == 1
            imwrite(idx, map, filename, 'gif', ...
                'LoopCount', Inf, 'DelayTime', delay);
        else
            imwrite(idx, map, filename, 'gif', ...
                'WriteMode', 'append', 'DelayTime', delay);
        end
    end

    % Clean up temp PNGs
    for f = 1:numel(frame_files)
        delete(frame_files{f});
    end
end