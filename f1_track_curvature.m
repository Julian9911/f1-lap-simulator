%% f1_track_curvature.m
% Takes the raw Silverstone centerline (unevenly spaced) and produces:
%   s          - accumulated distance, uniform spacing [m]
%   x, y       - circuit coordinates, uniformly resampled
%   curvature  - curvature at each point [1/m] (positive, unsigned)
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
% Circuit geometry: (c) OpenStreetMap contributors, ODbL.
%
% This is what the Pilot uses later for the corner speed limit: the higher
% the curvature, the lower the safe speed.

track = readmatrix('silverstone_centerline.csv');

%% Accumulated distance over the raw points (uneven spacing)
d = sqrt(sum(diff(track).^2, 2));
s_raw = [0; cumsum(d)];

% Remove duplicate points (distance ~0 between consecutive points, where two
% segments were joined at the exact same point) - a spline does not accept
% repeated values on its independent axis
keep = [true; diff(s_raw) > 1e-6];
s_raw = s_raw(keep);
track = track(keep, :);
totalLength = s_raw(end);

fprintf('Total length: %.3f km\n', totalLength/1000);

%% Resampling to uniform spacing (PERIODIC: the circuit is closed)
% v2: the circuit is extended periodically (the points of the end are "glued"
% to the start and vice versa) BEFORE resampling, smoothing and
% differentiating, and trimmed at the end. Without this, the smoothing
% filter and the derivatives have edge effects at the finish line: the
% track heading jumped about 5 degrees between the last and the first
% point (a steering spike when crossing the finish line).
ds = 2; % [m] distance between resampled points - 2 m is a good starting
         % point: fine enough not to lose the detail of tight corners,
         % without generating a huge array
pad   = 40;  % [points] padding on each side (80 m) that is discarded at the end
m_ext = 60;  % [raw points] used to extend the circuit (they cover > 80 m)

% The last raw point coincides with the first one (closed circuit): it is
% excluded when extending so that it is not duplicated.
s_ext     = [s_raw(end-m_ext:end-1) - totalLength; s_raw; s_raw(2:m_ext+1) + totalLength];
track_ext = [track(end-m_ext:end-1, :); track; track(2:m_ext+1, :)];

s_all = (-pad*ds : ds : totalLength + pad*ds)';

x = interp1(s_ext, track_ext(:,1), s_all, 'spline');
y = interp1(s_ext, track_ext(:,2), s_all, 'spline');

%% Smoothing before differentiating
% OpenStreetMap points have small mapping inaccuracies, and curvature
% depends on the second derivative - that amplifies the noise enormously.
% Without this step, the curvature comes out as isolated spikes instead of
% the smooth rises and falls that a real corner produces.
% Window ~50 m (25 points at 2 m/point), polynomial order 3.
win = 25;
x = sgolayfilt(x, 3, win);
y = sgolayfilt(y, 3, win);

%% Curvature by finite differences
dx  = gradient(x, ds);
dy  = gradient(y, ds);
ddx = gradient(dx, ds);
ddy = gradient(dy, ds);

curvature = abs(dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;

%% Trim to the circuit (discard the periodic padding)
keepIdx   = (s_all >= 0) & (s_all < totalLength);
s         = s_all(keepIdx);
x         = x(keepIdx);
y         = y(keepIdx);
curvature = curvature(keepIdx);

fprintf('Resampled points: %d (every %.1f m)\n', length(s), ds);

%% Visual check: plot the curvature next to the map
figure;

subplot(2,1,1);
plot(x, y, 'LineWidth', 1.5);
axis equal;
title('Silverstone - resampled centerline');
xlabel('X (m)'); ylabel('Y (m)');

subplot(2,1,2);
plot(s, curvature, 'LineWidth', 1);
title('Curvature vs. distance travelled');
xlabel('Distance (m)'); ylabel('Curvature (1/m)');
grid on;

% (the final save, with the names trackX/trackY/trackS, is at the end of
% the script, after the speed limit is also computed)

%% Speed limit from curvature (no aero yet - see note below)
% v_max = sqrt(a_lat_max / curvature) - the classic "friction circle"
% limit: the higher the curvature, the lower the safe speed.
%
% a_lat_max = 1.5g is ONLY mechanical grip, without downforce - a real F1
% car clearly exceeds this in fast corners thanks to the aerodynamic load,
% which grows with speed. This number will stay conservative in fast
% corners (Copse, Maggotts-Becketts) until the Aerodynamics subsystem
% exists - the slow corners (Village, The Loop) are already believable
% now, because aero weighs less there.
a_lat_max = 1.5 * 9.81; % [m/s^2] placeholder, no aero

curvature_safe = max(curvature, 1e-6); % avoid division by zero on straights
v_max = sqrt(a_lat_max ./ curvature_safe);

% Cap on straights: without this, curvature~0 gives absurd speeds. This
% value does NOT come from real physics (power vs. drag) - it is an
% arbitrary ceiling until the engine-drag balance really exists.
v_cap_kmh = 380;
v_max_kmh = min(v_max * 3.6, v_cap_kmh);

figure;
plot(s, v_max_kmh);
title('Speed limit vs. distance (no aero, placeholder)');
xlabel('Distance (m)'); ylabel('Speed limit (km/h)');
grid on;

trackX = x;
trackY = y;
trackS = s;
trackPsi = atan2(gradient(trackY), gradient(trackX));
save('silverstone_curvature.mat', 'trackS', 'trackX', 'trackY', 'curvature', 'v_max_kmh', 'totalLength');
fprintf('Saved: silverstone_curvature.mat (trackX, trackY, trackS, curvature, v_max_kmh)\n');