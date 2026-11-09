%% f1_load_raceline.m
% Replaces the pilot reference (centerline) with the RACING LINE generated
% by f1_make_raceline.py (silverstone_raceline.csv).
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Run AFTER f1_track_curvature.m (the master f1_vehicle_params.m already
% does it if useRaceline = true). It overwrites in the workspace:
%   trackX, trackY   - racing line, every 2 m, closed circuit
%   trackS           - accumulated distance [m]
%   curvature        - curvature of the racing line [1/m] (unsigned)
%   trackPsi         - heading of the line [rad] = atan2(dy, dx), computed
%                      with periodic extension (no jump at the finish line)
%   totalLength      - length of the racing line [m]
%
% The Simulink model does NOT change: the pilot follows trackX/trackY/trackPsi
% and the speed profile (f1_speed_profile.m) is recomputed with the new
% curvature. To go back to v1 (centerline): useRaceline = false in the master.

rl = readmatrix('silverstone_raceline.csv');
x0 = rl(:,1);  y0 = rl(:,2);
n0 = numel(x0);

ds = 2;                         % [m] same spacing as f1_track_curvature
pad = 40;                       % [points] periodic extension

% Periodic extension to differentiate without edge effects at the finish line
xe = [x0(end-pad+1:end); x0; x0(1:pad)];
ye = [y0(end-pad+1:end); y0; y0(1:pad)];
dx  = gradient(xe, ds);   dy  = gradient(ye, ds);
ddx = gradient(dx, ds);   ddy = gradient(dy, ds);
kappa = abs(dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;
psi   = atan2(dy, dx);

idx = pad+1 : pad+n0;
trackX    = x0;
trackY    = y0;
trackS    = (0:n0-1)' * ds;
curvature = kappa(idx);
trackPsi  = psi(idx);
totalLength = n0 * ds;

% THEORETICAL speed limit from curvature (same formula as f1_track_curvature.m).
% It is used by the legacy block f1_speed_target (Pilot/MATLAB Function2) and must
% have the SAME number of points as trackS, or it gives "Index exceeds array
% dimensions" (the centerline had 2929 points and the racing line 2894).
a_lat_max = 1.5 * 9.81;                                   % [m/s^2] same as in f1_track_curvature.m
v_max_kmh = min(sqrt(a_lat_max ./ max(curvature, 1e-6)) * 3.6, 380);

fprintf('Racing line: %d points, %.0f m | curvature p95 %.4f 1/m | heading jump at the finish line %.2f deg\n', ...
    n0, totalLength, prctile(curvature, 95), rad2deg(abs(atan2(sin(trackPsi(1)-trackPsi(end)), cos(trackPsi(1)-trackPsi(end))))));