function save_pgo_gif(frame_dir, gif_path, delay)
%SAVE_PGO_GIF  Assemble a sequence of PNG frames into an animated GIF.
%
%   SAVE_PGO_GIF(frame_dir, gif_path, delay)
%
%   frame_dir   folder containing frame_0001.png, frame_0002.png, ...
%                (written by the caller via print(fig, ..., '-dpng', ...))
%   gif_path    output .gif file path
%   delay       seconds per frame in the animation
%
%   Uses the print()->PNG->imread->imwrite pipeline rather than
%   getframe(), since getframe() can silently capture blank frames with
%   the OpenGL renderer in R2024a.

files = dir(fullfile(frame_dir, 'frame_*.png'));
names = {files.name};
[~, order] = sort(names);
files = files(order);

if isempty(files)
    warning('save_pgo_gif:noFrames', 'No frame_*.png files found in %s', frame_dir);
    return
end

for k = 1:numel(files)
    img = imread(fullfile(frame_dir, files(k).name));
    [imind, cm] = rgb2ind(img, 256);
    if k == 1
        imwrite(imind, cm, gif_path, 'gif', 'Loopcount', inf, 'DelayTime', delay);
    else
        imwrite(imind, cm, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
    end
end

end