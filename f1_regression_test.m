%% f1_regression_test.m
% Regression test of the simulator: after ANY change to the model or to the
% parameters, run f1_vehicle_params, simulate 260 s and run this script.
% It says PASS / FAIL per criterion, to know in seconds whether something broke.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Requires in the workspace: out (with carX_log, carY_log, xdot_log, lapCount),
% trackX, trackY, totalLength (from f1_vehicle_params).
%
% The thresholds and the baseline are below, with their justification. If you
% improve the model on purpose (e.g. more grip), update the BASELINE.

%% --- Baseline and thresholds -------------------------------------------
base.lapTime   = 94.7;    % [s] v4 + LSD + Tair 288.15 K (rho consistent with the profile): 94.71 s. (Before, Tair 273 K: 95.16 s. (v4 with open Active Differential: 94.86 s. (v3, ClA 2.7: 102.96 s. (v3 with h_cg 0.275: 102.45 s; v2 with useFC = false: 109.3 s)
tol.lapTime    = 1.5;     % [s] tolerance; a larger change = the behavior changed
tol.latErrMax  = 3.0;     % [m] maximum lateral error (v4 + Tair 288 K: 2.03 m; before 1.94 m, v3: 2.2 m, in the tight corner at ~1125 m; racing line half-width 4 m)
tol.latErrP95  = 1.8;     % [m] 95 % of the time the car follows the line with this precision (v4 + Tair 288 K: 1.56 m; before 1.51 m, v3: 1.55 m)
tol.lapSpread  = 1.5;     % [s] difference between consecutive complete laps (repeatability)
tol.lenErr     = 0.03;    % [-] the distance traveled must be +-3 % of the circuit length
tol.vMax       = 95;      % [m/s] ~342 km/h: above this, something is not physical
tol.vMinRun    = 10;      % [m/s] the car must not stop during a complete lap

%% --- Laps ----------------------------------------------------------------
lcData = out.lapCount.Data(:);  lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);
tBounds = [0; tCross];  lapDur = diff(tBounds);
nLaps = numel(lapDur);

res = cell(0, 4);   % {name, value (text), threshold (text), passes}
addrow = @(r, n, v, t, ok) [r; {n, v, t, ok}];   % anonymous function (does not depend on local functions)

res = addrow(res, 'Complete laps >= 2', sprintf('%d', nLaps), '>= 2', nLaps >= 2);
if nLaps < 1
    error('f1_regression_test:noLaps', 'No lap was completed; simulate longer (260 s).');
end

% Evaluation lap: the fastest among those that are NOT the standing start
valid = 2:nLaps;  if isempty(valid), valid = 1; end
[~, ib] = min(lapDur(valid));  lap = valid(ib);
t0 = tBounds(lap);  t1 = tBounds(lap + 1);
res = addrow(res, 'Lap time', sprintf('%.2f s (lap %d)', t1 - t0, lap), ...
    sprintf('%.1f +- %.1f s', base.lapTime, tol.lapTime), abs((t1 - t0) - base.lapTime) <= tol.lapTime);

if numel(lapDur) >= 3
    sp = max(lapDur(2:end)) - min(lapDur(2:end));
    res = addrow(res, 'Repeatability between laps', sprintf('%.2f s', sp), sprintf('< %.1f s', tol.lapSpread), sp < tol.lapSpread);
end

%% --- Signals in the evaluated lap -----------------------------------------
tg = (t0:0.05:t1)';
xs = interp1(out.carX_log.Time(:),  out.carX_log.Data(:),  tg);
ys = interp1(out.carY_log.Time(:),  out.carY_log.Data(:),  tg);
vs = interp1(out.xdot_log.Time(:),  out.xdot_log.Data(:),  tg);

res = addrow(res, 'Signals without NaN/Inf', sprintf('%d invalid', nnz(~isfinite([xs; ys; vs]))), '0', all(isfinite([xs; ys; vs])));
res = addrow(res, 'Maximum speed', sprintf('%.0f km/h', max(vs)*3.6), sprintf('< %.0f km/h', tol.vMax*3.6), max(vs) < tol.vMax);
if lap > 1
    res = addrow(res, 'No stops (complete lap)', sprintf('v min %.0f km/h', min(vs)*3.6), sprintf('> %.0f km/h', tol.vMinRun*3.6), min(vs) > tol.vMinRun);
end

% Distance traveled vs circuit length (detects shortcuts or fake laps)
dist = sum(hypot(diff(xs), diff(ys)));
lenErr = dist / totalLength - 1;
res = addrow(res, 'Distance traveled / circuit', sprintf('%.0f m (%+.1f %%)', dist, 100*lenErr), sprintf('+-%.0f %%', 100*tol.lenErr), abs(lenErr) <= tol.lenErr);

% Lateral error: distance to the nearest segment of the reference line
rx = [trackX(:); trackX(1)];  ry = [trackY(:); trackY(1)];
ax = rx(1:end-1);  ay = ry(1:end-1);  bx = rx(2:end);  by = ry(2:end);
ex = bx - ax;  ey = by - ay;  e2 = max(ex.^2 + ey.^2, eps);
latErr = zeros(size(xs));
for i = 1:numel(xs)
    u = min(max(((xs(i) - ax).*ex + (ys(i) - ay).*ey) ./ e2, 0), 1);
    latErr(i) = sqrt(min((xs(i) - ax - u.*ex).^2 + (ys(i) - ay - u.*ey).^2));
end
sl = sort(latErr);  p95 = sl(ceil(0.95*numel(sl)));   % without the Statistics Toolbox
res = addrow(res, 'Maximum lateral error', sprintf('%.2f m', max(latErr)), sprintf('< %.1f m', tol.latErrMax), max(latErr) < tol.latErrMax);
res = addrow(res, 'Lateral error p95', sprintf('%.2f m', p95), sprintf('< %.1f m', tol.latErrP95), p95 < tol.latErrP95);

% Diagnostic information (does not count for PASS/FAIL): where the maximum error occurs
sAcum = [0; cumsum(hypot(diff(xs), diff(ys)))];
[~, im] = max(latErr);
fprintf('\n[info] Maximum lateral error (%.2f m) at %.0f m of the lap, at %.0f km/h\n', latErr(im), sAcum(im), vs(im)*3.6);
big = latErr > 1.5;
if any(big), fprintf('[info] Stretch with error > 1.5 m: %.0f m in total (of %.0f m)\n', nnz(big)*mean(diff(sAcum)), sAcum(end)); end

%% --- Report ----------------------------------------------------------------
fprintf('\n=== f1_regression_test ===\n');
fprintf('%-34s %-22s %-18s %s\n', 'Criterion', 'Value', 'Threshold', 'Result');
for k = 1:size(res, 1)
    if res{k,4}, s = 'PASS'; else, s = 'FAIL'; end
    fprintf('%-34s %-22s %-18s %s\n', res{k,1}, res{k,2}, res{k,3}, s);
end
allOk = all([res{:,4}]);
if allOk, fprintf('\nRESULT: ALL OK (%d/%d)\n', size(res,1), size(res,1));
else,     fprintf('\nRESULT: %d criterion(s) FAILED\n', nnz(~[res{:,4}])); end