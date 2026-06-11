function save_exploration_gif(fig, filename, delay, frame_idx)
% SAVE_EXPLORATION_GIF  Append one frame to an animated GIF.
%
%   Uses print() → PNG → imread → imwrite pipeline because getframe()
%   silently captures blank frames with MATLAB's OpenGL renderer in R2024a.
%
%   Usage:
%     save_exploration_gif(fig, 'exploration.gif', 0.1, 1);   % first frame
%     save_exploration_gif(fig, 'exploration.gif', 0.1, 2);   % subsequent
%
%   Inputs:
%     fig        – figure handle
%     filename   – output GIF path (string)
%     delay      – seconds per frame (e.g. 0.1)
%     frame_idx  – 1 for first frame, >1 for append

tmp = [tempname '.png'];
print(fig, tmp, '-dpng', '-r72');
frame = imread(tmp);
delete(tmp);

[imind, cm] = rgb2ind(frame, 256);

if frame_idx == 1
    imwrite(imind, cm, filename, 'gif', ...
        'Loopcount', Inf, 'DelayTime', delay);
else
    imwrite(imind, cm, filename, 'gif', ...
        'WriteMode', 'append', 'DelayTime', delay);
end

end