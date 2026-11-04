%% f1_record_animation.m
% Records the lap animation to an MP4 video (and, optionally, a GIF) for the README.
% Same picture as f1_track_animation, but rendered frame by frame, so the result does
% not depend on how fast MATLAB draws. Post-processing only: it does not modify the model.
%
% Requirements (after simulating, with "Single simulation output"):
%   out.carX_log, out.carY_log, out.xdot_log, out.lapCount; trackX, trackY in the workspace.
% Output: <project root>/docs/img/f1_lap.mp4  (and f1_lap.gif if makeGif = true)

lapToShow = 0;       % 0 = fastest lap
playSpeed = 5;       % 3 = three times real time (a 95 s lap becomes about 32 s)
fps       = 30;      % video frame rate
makeGif   = true;   % GIF is much larger than MP4; keep it short (high playSpeed)
gifFps    = 12;

if ~exist('out', 'var')
    error('f1_record_animation:noOut', 'No "out" in the workspace: simulate first.');
end
try, root = currentProject().RootFolder; catch, root = pwd; end
outDir = fullfile(root, 'docs', 'img');
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% Lap window (same rule as f1_lap_table)
lcData = out.lapCount.Data(:);  lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);
tBounds = [0; tCross];  lapDur = diff(tBounds);
if isempty(lapDur), error('f1_record_animation:noLap', 'No lap completed.'); end
if lapToShow == 0, [~, lapToShow] = min(lapDur); end
t0 = tBounds(lapToShow);  t1 = tBounds(lapToShow + 1);  lapTime = t1 - t0;

dtGrid = 0.02;
tg = (t0:dtGrid:t1)';
x  = interp1(out.carX_log.Time(:), out.carX_log.Data(:), tg);
y  = interp1(out.carY_log.Time(:), out.carY_log.Data(:), tg);
v_kmh = interp1(out.xdot_log.Time(:), out.xdot_log.Data(:), tg) * 3.6;
s  = [0; cumsum(hypot(diff(x), diff(y)))];

%% Figure (same palette as f1_track_animation)
keyIdx = [1 20 39 57 64];
keyRGB = [0.9325 0.1675 0.1675; 0.9325 0.9325 0.1675; 0.1675 0.9325 0.1675; 0.1675 0.9325 0.9325; 0.1675 0.6775 0.9325];
speedColormap = interp1(keyIdx, keyRGB, 1:64);
bgColor = [0.0431 0.0510 0.0706];
fig = figure('Color', bgColor, 'Name', 'F1 lap recording', 'Position', [80 80 1280 720], 'MenuBar', 'none', 'ToolBar', 'none');
ax = axes('Parent', fig, 'Color', bgColor);
hold(ax, 'on'); axis(ax, 'equal');
ax.XColor = 'w'; ax.YColor = 'w';
title(ax, sprintf('Silverstone Circuit - F1 Model Simulator (lap %d, %.2f s)', lapToShow, lapTime), 'Color', 'w');
xlabel(ax, 'X (m)', 'Color', 'w'); ylabel(ax, 'Y (m)', 'Color', 'w');
if exist('trackX', 'var') && exist('trackY', 'var')
    plot(ax, [trackX(:); trackX(1)], [trackY(:); trackY(1)], '-', 'Color', [0.25 0.27 0.32], 'LineWidth', 1);
end
patch(ax, [x; NaN], [y; NaN], [v_kmh; NaN], 'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 4);
colormap(ax, speedColormap);
clim(ax, [0, ceil(max(v_kmh)/10)*10]);
cb = colorbar(ax); cb.Label.String = 'Simulated speed (km/h)'; cb.Label.Color = 'w'; cb.Color = 'w';
carMarker = plot(ax, x(1), y(1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
hudText = text(ax, 0.02, 0.98, '', 'Units', 'normalized', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold', 'VerticalAlignment', 'top');

%% Render frame by frame
vw = VideoWriter(fullfile(outDir, 'f1_lap.mp4'), 'MPEG-4');
vw.FrameRate = fps;  vw.Quality = 90;
open(vw);
tFrames = 0 : playSpeed/fps : lapTime;
gifStep = max(1, round(fps / gifFps));
gifFile = fullfile(outDir, 'f1_lap.gif');
for k = 1:numel(tFrames)
    idx = min(numel(tg), max(1, floor(tFrames(k) / dtGrid) + 1));
    set(carMarker, 'XData', x(idx), 'YData', y(idx));
    set(hudText, 'String', sprintf('Speed: %.0f km/h\nTime: %.1f s\nDistance: %.0f m', v_kmh(idx), tFrames(k), s(idx)));
    drawnow;
    fr = getframe(fig);
    writeVideo(vw, fr);
    if makeGif && mod(k - 1, gifStep) == 0
        [A, map] = rgb2ind(fr.cdata, 128);
        if k == 1
            imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 1/gifFps);
        else
            imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1/gifFps);
        end
    end
end
close(vw);
close(fig);
fprintf('f1_record_animation: wrote %s (%d frames)\n', fullfile(outDir, 'f1_lap.mp4'), numel(tFrames));
