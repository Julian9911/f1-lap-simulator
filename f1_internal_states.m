%% f1_internal_states.m
% "Internal states" view of the simulated car: per-wheel load and forces, a
% wheel utilization index, lateral load transfer and body slip - signals that
% CANNOT be measured on a real car - next to the g-g diagram (which can).
% This is what a model-in-the-loop test bench gives you that the car does not.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% WHERE THE SIGNALS COME FROM: the Info bus of the Vehicle Body, not the
% Combined Slip Wheel blocks. In this model Fy is computed by the Vehicle Body
% from the mapped lateral table (CalphaMode = Mapped slip angle), Fx comes from
% the wheel blocks (External longitudinal forces) and Fz from the Vehicle Body
% load transfer. "Utilization" is an index defined here, not a model signal:
%     |F_xy| / (lateral peak mu * Fz)
%
% Requirements: simulate first (f1_add_internal_logging.m applied), so that the
% workspace has out (internal_log, xdot_log, lapCount) and tire, veh, aero
% (from f1_vehicle_params). Evaluates the fastest lap that is not the standing start.

if ~exist('out', 'var'), error('There is no "out" in the workspace: simulate first.'); end
try
    out.internal_log;
catch
    error('out.internal_log is missing: apply f1_add_internal_logging.m and simulate again.');
end
FzLow = 500;      % [N] below this a wheel is considered (almost) unloaded

%% Lap window (same criterion as f1_regression_test)
lcData = out.lapCount.Data(:);  lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);
tBounds = [0; tCross];  lapDur = diff(tBounds);
valid = 2:numel(lapDur);  if isempty(valid), valid = 1; end
[~, ib] = min(lapDur(valid));  lap = valid(ib);
t0 = tBounds(lap);  t1 = tBounds(lap + 1);

%% Data of the lap on a uniform grid
dt = 0.02;  tg = (t0:dt:t1)';
D = out.internal_log.Data;
if ndims(D) == 3, D = squeeze(D); end
if size(D, 1) == 15 && size(D, 2) ~= 15, D = D.'; end
Dg = interp1(out.internal_log.Time(:), D, tg);
v  = interp1(out.xdot_log.Time(:), out.xdot_log.Data(:), tg);
s  = cumtrapz(tg, v);                                  % distance [m]
Fz = abs(Dg(:, 1:4));  Fx = Dg(:, 5:8);  Fy = Dg(:, 9:12);
beta = Dg(:, 15);
g0 = 9.81;

%% Units of the acceleration signals: compare with the derivative of the speed
accNum = gradient(v, dt);
ratio  = std(accNum, 'omitnan') / std(Dg(:, 13), 'omitnan');
if ratio > 4 && ratio < 20
    accScale = g0;   unitTxt = 'g (converted to m/s^2)';
else
    accScale = 1;    unitTxt = 'm/s^2';
end
fprintf('Acceleration signals: std(dv/dt) / std(ax) = %.2f -> taken as %s\n', ratio, unitTxt);
ax = Dg(:, 13) * accScale / g0;  ay = Dg(:, 14) * accScale / g0;   % in g

%% Derived quantities
muPk = [tire.mu_pk_f tire.mu_pk_f tire.mu_pk_r tire.mu_pk_r];
util = hypot(Fx, Fy) ./ (repmat(muPk, size(Fz, 1), 1) .* max(Fz, 1));
util(Fz < FzLow) = NaN;                                % meaningless with an unloaded wheel
latTransfer = 100 * (Fz(:,2) + Fz(:,4) - Fz(:,1) - Fz(:,3)) ./ max(sum(Fz, 2), 1);   % right - left, % of total

%% Console report
wn = {'FL', 'FR', 'RL', 'RR'};
p95 = @(x) subsref(sort(x(~isnan(x))), struct('type', '()', 'subs', {{ceil(0.95*nnz(~isnan(x)))}}));
[vTop, it] = max(v);
FzExpected = veh.m * g0 + 0.5 * aero.rho * vTop^2 * aero.ClA;
fprintf('Lap %d (%.2f s)\n', lap, t1 - t0);
fprintf('Check at top speed (%.0f km/h): total Fz %.1f kN | weight + downforce %.1f kN\n', ...
    vTop*3.6, sum(Fz(it, :))/1e3, FzExpected/1e3);
for k = 1:4
    fprintf('  %s: Fz min %.1f kN | max %.1f kN | utilization p95 %.2f | max %.2f\n', ...
        wn{k}, min(Fz(:,k))/1e3, max(Fz(:,k))/1e3, p95(util(:,k)), max(util(:,k)));
end
fprintf('Max lateral %.2f g | max braking %.2f g | max traction %.2f g | |beta| max %.2f deg\n', ...
    max(abs(ay)), -min(ax), max(ax), max(abs(beta))*180/pi);
low = find(min(Fz, [], 2) < FzLow);
if isempty(low)
    fprintf('No wheel drops below %d N.\n', FzLow);
else
    brk = [0; find(diff(low) > 1); numel(low)];
    fprintf('Stretches with a (nearly) unloaded wheel (< %d N): %.1f %% of the lap\n', FzLow, 100*numel(low)/numel(tg));
    for q = 1:numel(brk)-1
        a = low(brk(q)+1);  b = low(brk(q+1));
        fprintf('   s = %.0f to %.0f m  (v = %.0f km/h)\n', s(a), s(b), mean(v(a:b))*3.6);
    end
end

%% Figure (light theme, front = blue, rear = orange; left = solid, right = dashed)
fig = figure('Color', 'w', 'Position', [40 40 1500 820], 'Name', 'F1 Model Simulator - internal states');
try, theme(fig, 'light'); catch, end                    % theme() exists in recent MATLAB releases
cW = [0.12 0.37 0.70; 0.12 0.37 0.70; 0.88 0.40 0.05; 0.88 0.40 0.05];
lS = {'-', '--', '-', '--'};
tl = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
% Click an item of a legend to hide / show that signal (anonymous: local functions fail in scripts here)
tog = @(~, e) set(e.Peer, 'Visible', matlab.lang.OnOffSwitchState(~logical(e.Peer.Visible)));
title(tl, sprintf('Internal states of the simulated car - lap %d (%.2f s)      * not measurable on a real car', lap, t1 - t0), ...
    'FontWeight', 'bold');
subtitle(tl, 'Click an item in a legend to hide / show that signal', 'Color', [0.4 0.4 0.4]);

ax1 = nexttile; hold on;
hFz = gobjects(1, 4);
for k = 1:4, hFz(k) = plot(s, Fz(:,k)/1e3, lS{k}, 'Color', cW(k,:), 'LineWidth', 1.2); end
ylabel('kN'); title('Vertical load per wheel *');
legend(hFz, wn, 'Location', 'northoutside', 'NumColumns', 4, 'ItemHitFcn', tog);

ax2 = nexttile; hold on;
hUt = gobjects(1, 4);
for k = 1:4, hUt(k) = plot(s, util(:,k), lS{k}, 'Color', cW(k,:), 'LineWidth', 1.2); end
yline(1, ':k', 'HandleVisibility', 'off'); ylim([0 1.5]);
ylabel('|F| / (\mu_{peak} Fz)   (unloaded wheels hidden)'); title('Wheel utilization *');
legend(hUt, wn, 'Location', 'northoutside', 'NumColumns', 4, 'ItemHitFcn', tog);

nexttile; hold on;
th = linspace(0, 2*pi, 200);
for r = 1:5, plot(r*cos(th), r*sin(th), ':', 'Color', [0.78 0.78 0.78], 'HandleVisibility', 'off'); end
hGG = gobjects(1, 3);
pBrk = ax < -0.5;  pAcc = ax > 0.5;  pOth = ~(pBrk | pAcc);
hGG(1) = scatter(ay(pOth), ax(pOth), 6, [0.55 0.60 0.68], 'filled');
hGG(2) = scatter(ay(pBrk), ax(pBrk), 6, [0.80 0.20 0.20], 'filled');
hGG(3) = scatter(ay(pAcc), ax(pAcc), 6, [0.15 0.60 0.30], 'filled');
axis equal; xlim([-5.5 5.5]); ylim([-5 3]);
xline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');  yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
xlabel('Lateral acceleration (g)   [left <-  -> right]'); ylabel('Longitudinal (g)   [braking -  / + traction]');
legend(hGG, {'cruise / cornering', 'braking (< -0.5 g)', 'traction (> 0.5 g)'}, 'Location', 'southoutside', 'NumColumns', 3, 'ItemHitFcn', tog);
title('g-g diagram, rings every 1 g (measurable with an IMU)');

ax4 = nexttile; plot(s, latTransfer, 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2); yline(0, ':k');
ylabel('% of total load'); title('Lateral load transfer, right - left *');

ax5 = nexttile; plot(s, beta*180/pi, 'Color', [0.55 0.15 0.55], 'LineWidth', 1.2); yline(0, ':k');
ylabel('deg'); title('Body slip angle *');

ax6 = nexttile; plot(s, v*3.6, 'Color', [0.1 0.1 0.1], 'LineWidth', 1.2);
ylabel('km/h'); title('Speed (reference)');

for a = [ax1 ax2 ax4 ax5 ax6]
    grid(a, 'on'); box(a, 'off'); xlabel(a, 'Distance (m)'); xlim(a, [0 s(end)]);
end
set(findall(fig, '-property', 'FontSize'), 'FontSize', 11);

exportgraphics(fig, 'f1_internal_states.png', 'Resolution', 200);
fprintf('Saved f1_internal_states.png\n');
