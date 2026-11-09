%% f1_telemetry.m
% Team-style telemetry: simulator against the real Silverstone lap, in THREE
% figures (black background) with the lap distance as the common axis and the
% three sectors as colored bands. Post-processing only: it does not touch the
% model and prints nothing to the console.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
% Real telemetry: TracingInsights/2026 (Apache-2.0), see README.
%
%   Figure 1 - Pace:        speed, time delta, lateral G, longitudinal G
%   Figure 2 - Driving:     speed (reference), throttle, brake
%   Figure 3 - Powertrain:  speed (reference), gear, rpm, engine power
%
% Zooming one figure applies to all three (the distance axes are linked).
%
% Requirements in the workspace: out (Single simulation output) and trackS
% (f1_vehicle_params). Signals logged in the model (To Workspace, Timeseries):
%   xdot_log, carX_log, carY_log, lapCount
%   acc_log, dec_log      AccelCmd / DecelCmd of the Predictive Driver (0 to 1)
%   gear_log              gear (2 to 8), from f1_shift_logic
%   engSpd_log [rad/s], engTrq_log [N*m]    from the Info bus of the transmission
% If a signal is missing, its panel shows a "no simulator data" notice.
% Files needed in the path: silverstone_centerline.csv and silverstone_real_telemetry.csv.

lapToShow = 0;                    % 0 = fastest simulated lap

% Sectors (the same as f1_sector_map.m): limits in lap meters over
% silverstone_centerline.csv; APPROXIMATE (taken from an image of the sectors).
sectorEdges  = [1660 4420];
sectorColors = [0.90 0.30 0.18; 0.30 0.78 0.95; 1.00 0.82 0.20];

% Dark theme colors
cBg  = [0.04 0.04 0.05];  cPanel = [0.075 0.075 0.09];  cFg = [0.95 0.95 0.95];
cDim = [0.62 0.62 0.67];  cGrid  = [0.20 0.20 0.24];
cReal = [0.85 0.85 0.89]; cSim = [1.00 0.48 0.27];

%% Simulated lap
lcData = out.lapCount.Data(:);
lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);
tBounds = [0; tCross];
lapDur  = diff(tBounds);
if lapToShow == 0, [~, lapToShow] = min(lapDur); end
t0 = tBounds(lapToShow);  t1 = tBounds(lapToShow + 1);
simLapTime = t1 - t0;

dtGrid = 0.02;
tg = (t0:dtGrid:t1)';
xs = interp1(out.carX_log.Time(:), out.carX_log.Data(:), tg);
ys = interp1(out.carY_log.Time(:), out.carY_log.Data(:), tg);
vs = interp1(out.xdot_log.Time(:), out.xdot_log.Data(:), tg) * 3.6;   % km/h
ss = [0; cumsum(hypot(diff(xs), diff(ys)))];

% Extra simulator signals on the same time grid
sigNames = {'acc_log', 'dec_log', 'gear_log', 'engSpd_log', 'engTrq_log'};
sig = struct();
for q = 1:numel(sigNames)
    try
        ts = out.get(sigNames{q});
        [tu, iu] = unique(ts.Time(:));
        du = double(ts.Data(:));  du = du(iu);
        if strcmp(sigNames{q}, 'gear_log'), mth = 'previous'; else, mth = 'linear'; end
        sig.(sigNames{q}) = interp1(tu, du, tg, mth, 'extrap');
    catch
        sig.(sigNames{q}) = nan(size(tg));
    end
end
rpmS = sig.engSpd_log * 30 / pi;
pwrS = sig.engTrq_log .* sig.engSpd_log / 1000;                       % kW
% Simulator G from the trajectory (smoothed ~0.5 s)
w = 25;
vms = movmean(vs/3.6, w);
hd  = unwrap(atan2(gradient(movmean(ys, w)), gradient(movmean(xs, w))));
ayS = abs(vms .* gradient(movmean(hd, w), dtGrid)) / 9.81;
axS = gradient(vms, dtGrid) / 9.81;

%% Real telemetry
R = readtable('silverstone_real_telemetry.csv');
realLapTime = R.Time_s(end);

%% Track length and common grid
C = readmatrix('silverstone_centerline.csv');
C = C([true; hypot(diff(C(:,1)), diff(C(:,2))) > 1e-6], :);
if norm(C(1,:) - C(end,:)) > 1, C = [C; C(1,:)]; end
Lc = sum(hypot(diff(C(:,1)), diff(C(:,2))));
if exist('totalLength', 'var'), L = totalLength; else, L = trackS(end) + 2; end
sg = (0:2:L-2)';
edG = [0, sectorEdges(:)'/Lc*L, L];             % sectors on the grid [m]

sa = mod(R.S_aligned(:), L);
[saU, ~, icR] = unique(sa);
toGrid = @(col) interp1([saU - L; saU; saU + L], repmat(accumarray(icR, col(:), [], @mean), 3, 1), sg, 'linear');
vreal = max(toGrid(R.Speed_kmh), 5);
thrR  = toGrid(R.Throttle);
brkR  = toGrid(R.Brake) * 100;
gerR  = toGrid(R.nGear);
rpmR  = toGrid(R.RPM);
ayR   = abs(toGrid(R.AccY)) / 9.81;
axR   = toGrid(R.AccX) / 9.81;

% Simulator on the distance grid
xg  = ss / ss(end) * L;
[xgU, ig] = unique(xg);
onG = @(y, m) interp1(xgU, y(ig), sg, m, 'extrap');
vsim  = max(onG(vs, 'linear'), 5);
thrS  = onG(sig.acc_log * 100, 'linear');
brkS  = onG(sig.dec_log * 100, 'linear');
gerS  = onG(sig.gear_log, 'nearest');
rpmSg = onG(rpmS, 'linear');
pwrSg = onG(pwrS, 'linear');
aySg  = onG(ayS, 'linear');
axSg  = onG(axS, 'linear');

% Accumulated time difference (scaled to the measured lap times)
tsim  = cumtrapz(sg, 1 ./ (vsim/3.6));   tsim  = tsim  * simLapTime  / tsim(end);
treal = cumtrapz(sg, 1 ./ (vreal/3.6));  treal = treal * realLapTime / treal(end);
dT = tsim - treal;                       % > 0: the simulator is behind
dInc = diff([0; dT]);

% Time lost by section type (according to the REAL speed)
tipoEdges = [0 150 250 inf];
lost = zeros(1, 3);
for k = 1:3
    lost(k) = sum(dInc(vreal >= tipoEdges(k) & vreal < tipoEdges(k+1)));
end

%% Available panels: {label, real, simulator, Y limits}
panel.vel = {'Speed [km/h]',    vreal, vsim,  [0 350]};
panel.dt  = {'Time delta [s]',     [],    dT,    []};
panel.ay  = {'Lateral G [g]',       ayR,   aySg,  []};
panel.ax  = {'Longitudinal G [g]',     axR,   axSg,  []};
panel.thr = {'Throttle [%]',      thrR,  thrS,  [-5 105]};
panel.brk = {'Brake [%]',           brkR,  brkS,  [-5 105]};
panel.ger = {'Gear',              gerR,  gerS,  [1.5 8.5]};
panel.rpm = {'Engine [rpm]',         rpmR,  rpmSg, []};
panel.pwr = {'Engine power [kW]', [],    pwrSg, []};

% Groups: name, panels, relative heights
G = { 'Pace',       {'vel', 'dt', 'ay', 'ax'},        [3.0 1.8 1.4 1.4];
      'Driving',     {'vel', 'thr', 'brk'},            [2.4 1.3 1.3];
      'Powertrain',  {'vel', 'ger', 'rpm', 'pwr'},     [1.8 1.1 1.4 1.3] };

axAll = gobjects(0);
for g = 1:size(G, 1)
    items = G{g, 2};  hr = G{g, 3};  nP = numel(items);
    top = 0.915;  bot = 0.075;  gp = 0.014;
    H = (top - bot - gp*(nP-1)) * hr / sum(hr);

    fig = figure('Name', ['Telemetry - ' G{g,1}], 'Color', cBg, ...
        'Position', [40 + 40*(g-1), 60 - 10*(g-1), 1300, 760], 'InvertHardcopy', 'off');
    ax = gobjects(nP, 1);
    y = top;
    for i = 1:nP
        key = items{i};  spec = panel.(key);
        y = y - H(i);
        ax(i) = axes('Parent', fig, 'Position', [0.07 y 0.905 H(i)], 'Color', cPanel, ...
            'XColor', cDim, 'YColor', cDim, 'GridColor', cGrid, 'GridAlpha', 1, ...
            'FontSize', 10, 'Box', 'on', 'Layer', 'top');
        hold(ax(i), 'on');  grid(ax(i), 'on');
        for k = 1:3
            xregion(ax(i), edG(k), edG(k+1), 'FaceColor', sectorColors(k,:), 'FaceAlpha', 0.07, ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
        ylabel(ax(i), spec{1}, 'Color', cFg, 'FontSize', 10);
        if i < nP, set(ax(i), 'XTickLabel', []); end

        hasReal = ~isempty(spec{2});
        if hasReal, hR = plot(ax(i), sg, spec{2}, 'Color', cReal, 'LineWidth', 1.3); end
        if any(~isnan(spec{3}))
            hS = plot(ax(i), sg, spec{3}, 'Color', cSim, 'LineWidth', 1.3);
        else
            text(ax(i), 0.5, 0.5, 'no simulator data (signal not logged)', 'Units', 'normalized', ...
                'Color', [1 0.38 0.33], 'HorizontalAlignment', 'center');
        end
        if ~isempty(spec{4}), set(ax(i), 'YLim', spec{4}); end

        if strcmp(key, 'dt')
            yline(ax(i), 0, '-', 'Color', cDim);
            text(ax(i), 0.01, 0.93, sprintf('Time lost by section type:  slow (<150 km/h) %+.1f s   |   medium (150-250) %+.1f s   |   fast (>250) %+.1f s', ...
                lost(1), lost(2), lost(3)), 'Units', 'normalized', 'Color', cFg, 'FontSize', 9, 'VerticalAlignment', 'top');
        elseif strcmp(key, 'ger')
            set(ax(i), 'YTick', 2:8);
        elseif strcmp(key, 'pwr')
            text(ax(i), 0.01, 0.93, 'simulator only (the real CSV has no power data)', 'Units', 'normalized', ...
                'Color', cDim, 'FontSize', 9, 'VerticalAlignment', 'top');
        end
        if i == 1
            if any(~isnan(spec{3})) && hasReal
                lg = legend(ax(i), [hR hS], {'Real', 'Simulator'}, 'Location', 'southwest');
                set(lg, 'Color', cBg, 'TextColor', cFg, 'EdgeColor', cGrid);
            end
            for k = 1:3
                text(ax(i), (edG(k) + edG(k+1)) / 2 / L, 0.97, sprintf('SECTOR %d', k), 'Units', 'normalized', ...
                    'Color', sectorColors(k,:), 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'top');
            end
        end
        y = y - gp;
    end
    set(ax, 'XLim', [0 L]);
    xlabel(ax(nP), 'Lap distance [m]', 'Color', cFg);
    sgtitle(fig, sprintf('%s - Silverstone, lap %d: sim %.2f s  |  real %.2f s  |  diff %+.2f s', ...
        G{g,1}, lapToShow, simLapTime, realLapTime, simLapTime - realLapTime), 'FontWeight', 'bold', 'Color', cFg);
    axAll = [axAll; ax];
end
linkaxes(axAll, 'x');
set(axAll, 'XLim', [0 L]);
