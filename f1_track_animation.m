%% f1_track_animation.m   (v2: uses the REAL simulation DATA)
% Replays a simulated lap over Silverstone: the dot follows the REAL
% trajectory of the car (carX_log, carY_log) and the line is colored with
% the REAL speed (xdot_log). It no longer uses the theoretical speed limit.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Requirements (after simulating, with "Single simulation output"):
%   out.carX_log, out.carY_log, out.xdot_log   (To Workspace, Timeseries)
%   out.lapCount                               (from f1_lap_timer)
%   trackX, trackY in the workspace            (f1_vehicle_params)
%
% Parameters:
lapToShow = 0;     % 0 = the fastest lap; N = that lap (1 = with standing start)
playSpeed = 1;     % 1 = real time; 2 = twice as fast, etc.
if ~exist('out', 'var')
    error('There is no "out" in the workspace: simulate first (Single simulation output enabled).');
end
%% Lap window (finish line crossings from lapCount, same as f1_lap_table)
lcData = out.lapCount.Data(:);
lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);       % instants at which each lap is closed
tBounds = [0; tCross];                              % lap k = [tBounds(k), tBounds(k+1)]
lapDur  = diff(tBounds);
if isempty(lapDur)
    error('No lap was completed in this simulation.');
end
if lapToShow == 0
    [~, lapToShow] = min(lapDur);
end
t0 = tBounds(lapToShow);  t1 = tBounds(lapToShow + 1);
lapTime = t1 - t0;
%% Real data of that lap, on a uniform grid
dtGrid = 0.02;                                      % [s]
tg = (t0:dtGrid:t1)';
x  = interp1(out.carX_log.Time(:), out.carX_log.Data(:), tg);
y  = interp1(out.carY_log.Time(:), out.carY_log.Data(:), tg);
v_kmh = interp1(out.xdot_log.Time(:), out.xdot_log.Data(:), tg) * 3.6;
s  = [0; cumsum(hypot(diff(x), diff(y)))];          % distance traveled [m]
fprintf('Lap %d: %.2f s | v max %.0f km/h | v min %.0f km/h | distance %.0f m\n', ...
    lapToShow, lapTime, max(v_kmh), min(v_kmh), s(end));
%% Palette (red = slow -> yellow -> green -> cyan -> blue = fast)
keyIdx = [1 20 39 57 64];
keyRGB = [0.9325 0.1675 0.1675;
    0.9325 0.9325 0.1675;
    0.1675 0.9325 0.1675;
    0.1675 0.9325 0.9325;
    0.1675 0.6775 0.9325];
speedColormap = interp1(keyIdx, keyRGB, 1:64);
%% Figure
bgColor = [0.0431 0.0510 0.0706]; % #0b0d12
fig = figure('Color', bgColor, 'Name', 'F1 Model Simulator - Silverstone');
ax = axes('Parent', fig, 'Color', bgColor);
hold(ax, 'on'); axis(ax, 'equal');
ax.XColor = 'w'; ax.YColor = 'w';
title(ax, sprintf('Silverstone Circuit - F1 Model Simulator (lap %d, %.2f s)', lapToShow, lapTime), 'Color', 'w');
xlabel(ax, 'X (m)', 'Color', 'w');
ylabel(ax, 'Y (m)', 'Color', 'w');
% Reference centerline (dim gray) under the real trajectory
if exist('trackX', 'var') && exist('trackY', 'var')
    plot(ax, [trackX(:); trackX(1)], [trackY(:); trackY(1)], '-', 'Color', [0.25 0.27 0.32], 'LineWidth', 1);
end
% REAL trajectory colored with the REAL speed
patch(ax, [x; NaN], [y; NaN], [v_kmh; NaN], ...
    'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 4);
colormap(ax, speedColormap);
clim(ax, [0, ceil(max(v_kmh)/10)*10]);
cb = colorbar(ax);
cb.Label.String = 'Real speed (km/h)';
cb.Label.Color = 'w';
cb.Color = 'w';
carMarker = plot(ax, x(1), y(1), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
hudText = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
    'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
%% Animation (looping until the figure is closed)
tStart = tic;
while ishandle(fig)
    elapsed = mod(toc(tStart) * playSpeed, lapTime);
    idx = min(numel(tg), max(1, floor(elapsed / dtGrid) + 1));
    set(carMarker, 'XData', x(idx), 'YData', y(idx));
    set(hudText, 'String', sprintf('Speed: %.0f km/h\nTime: %.1f s\nDistance: %.0f m', ...
        v_kmh(idx), elapsed, s(idx)));
    drawnow limitrate;
end