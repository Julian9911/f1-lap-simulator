%% f1_sector_map.m
% A single plot (black background): map of the circuit with the centerline colored
% by sector and, next to it, one card per sector with the engineering information:
% time, speeds and G forces, simulator against the real lap.
% It prints nothing to the console. Post-processing: it does not touch the model.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Requirements in the workspace: out (Single simulation output) and trackS
% (f1_vehicle_params). In the folder: silverstone_centerline.csv and
% silverstone_real_telemetry.csv.

lapToShow = 0;                    % 0 = fastest simulated lap

% Sector limits in lap meters (from the finish line, on silverstone_centerline.csv).
% APPROXIMATE: they come from locating the color cuts of an image of the sectors
% on the layout (cuts on the Wellington straight and at the start of the
% Hangar straight), not from official coordinates. Adjust them if you have the official ones.
sectorEdges  = [1660 4420];
sectorNames  = {'Abbey - Wellington', 'Brooklands - Chapel', 'Hangar - Club'};
sectorColors = [0.90 0.30 0.18;   % sector 1
                0.30 0.78 0.95;   % sector 2
                1.00 0.82 0.20];  % sector 3

% Dark theme colors
cBg   = [0.04 0.04 0.05];   cFg   = [0.95 0.95 0.95];   cDim  = [0.62 0.62 0.67];
cCard = [0.11 0.11 0.13];   cEdge = [0.28 0.28 0.32];
cGood = [0.35 0.85 0.50];   cBad  = [1.00 0.38 0.33];

%% Simulated lap (same window as f1_telemetry.m)
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

%% Real telemetry
R = readtable('silverstone_real_telemetry.csv');
realLapTime = R.Time_s(end);

%% Layout (centerline) and length
C = readmatrix('silverstone_centerline.csv');
% The CSV has repeated consecutive points: they are removed (interp1 requires unique distances)
C = C([true; hypot(diff(C(:,1)), diff(C(:,2))) > 1e-6], :);
if norm(C(1,:) - C(end,:)) > 1, C = [C; C(1,:)]; end
dC = [0; cumsum(hypot(diff(C(:,1)), diff(C(:,2))))];
Lc = dC(end);

%% Position of the real lap as a lap fraction (aligned by position)
if exist('totalLength', 'var'), L = totalLength; else, L = trackS(end) + 2; end
if ismember('S_aligned', R.Properties.VariableNames)
    frR = mod(R.S_aligned(:), L) / L;
else
    frR = R.Distance_m(:) / R.Distance_m(end);
end

%% Statistics per sector
edF = [0, sectorEdges(:)'/Lc, 1];            % limits as a lap fraction
nS  = numel(edF) - 1;
secLen = diff(edF(:)) * Lc;                  % length of each sector [m]

% Time of each sector, simulated
fr = ss / ss(end);
tt = tg - t0;
[frU, iu] = unique(fr);
tEd = interp1(frU, tt(iu), edF, 'linear', 'extrap');
tEd(1) = 0;  tEd(end) = simLapTime;
secSim = diff(tEd(:));

% Time of each sector, real: telemetry time at the aligned position
% The real lap does not start at the finish line (S_aligned starts at ~84 m): the
% position is unwrapped (u grows continuously) and one cycle is repeated so
% that sectors crossing the finish line can be measured.
tR = R.Time_s(:);
f0 = frR(1);
uR = frR;
wrapIdx = find(diff(frR) < -0.5, 1);
if ~isempty(wrapIdx), uR(wrapIdx+1:end) = uR(wrapIdx+1:end) + 1; end
[uRu, iru] = unique(uR);
u2 = [uRu; uRu + 1];
t2 = [tR(iru); tR(iru) + realLapTime];
[u2, i2] = unique(u2);  t2 = t2(i2);
ue = zeros(size(edF));
for i = 1:numel(edF)
    ue(i) = edF(i) + (edF(i) < f0);
    if i > 1
        while ue(i) <= ue(i-1), ue(i) = ue(i) + 1; end
    end
end
tEdR = interp1(u2, t2, ue, 'linear');
secReal = diff(tEdR(:));

% G forces of the simulator from the trajectory (smoothed ~0.5 s)
w = 25;
vms = movmean(vs/3.6, w);
hd  = unwrap(atan2(gradient(movmean(ys, w)), gradient(movmean(xs, w))));
ayG = abs(vms .* gradient(movmean(hd, w), dtGrid)) / 9.81;
axG = gradient(vms, dtGrid) / 9.81;

% G forces of the real lap: accelerations of the CSV (AccX, AccY, m/s^2)
% Note: whether these channels are measured or derived is not confirmed (see the README).
ayR = movmean(abs(R.AccY(:)), 5) / 9.81;
axR = movmean(-R.AccX(:), 5) / 9.81;
vR  = R.Speed_kmh(:);

z = zeros(nS, 1);
S = struct('vmax', z, 'vmin', z, 'rvmax', z, 'rvmin', z, 'glat', z, 'gbrk', z, 'rglat', z, 'rgbrk', z);
for k = 1:nS
    if k < nS
        m  = fr  >= edF(k) & fr  < edF(k+1);
        mR = frR >= edF(k) & frR < edF(k+1);
    else
        m  = fr  >= edF(k);
        mR = frR >= edF(k);
    end
    S.vmax(k) = max(vs(m));   S.vmin(k) = min(vs(m));
    S.rvmax(k) = max(vR(mR)); S.rvmin(k) = min(vR(mR));
    S.glat(k) = max(ayG(m));  S.gbrk(k) = max(-axG(m));
    S.rglat(k) = max(ayR(mR)); S.rgbrk(k) = max(axR(mR));
end
% Average speed = sector distance / sector time (the same for sim and real)
vavgS = secLen ./ secSim * 3.6;
vavgR = secLen ./ secReal * 3.6;

%% Figure
fig = figure('Name', 'Sectors', 'Color', cBg, 'Position', [60 60 1400 720], 'InvertHardcopy', 'off');

% --- Map
axM = axes('Parent', fig, 'Position', [0.02 0.03 0.50 0.85], 'Color', cBg);
hold(axM, 'on');
plot(axM, C(:,1), C(:,2), 'Color', [0.40 0.40 0.44], 'LineWidth', 14);   % asphalt edge
plot(axM, C(:,1), C(:,2), 'Color', [0.10 0.10 0.11], 'LineWidth', 11);   % asphalt
edM = [0, sectorEdges(:)', Lc];
cen = mean(C(1:end-1, :), 1);
for k = 1:nS
    m = dC >= edM(k) & dC <= edM(k+1);
    plot(axM, C(m,1), C(m,2), 'Color', sectorColors(k,:), 'LineWidth', 3);
    sm = (edM(k) + edM(k+1)) / 2;
    p  = [interp1(dC, C(:,1), sm), interp1(dC, C(:,2), sm)];
    dv = p - cen;  dv = dv / norm(dv);
    q  = p + 110 * dv;
    text(axM, q(1), q(2), sprintf('%d', k), 'FontWeight', 'bold', 'FontSize', 14, ...
        'HorizontalAlignment', 'center', 'Color', 'k', 'BackgroundColor', sectorColors(k,:), ...
        'EdgeColor', cFg, 'Margin', 4);
end
% Sector cuts (white mark) and finish line
for e = sectorEdges(:)'
    p  = [interp1(dC, C(:,1), e), interp1(dC, C(:,2), e)];
    p2 = [interp1(dC, C(:,1), e + 5), interp1(dC, C(:,2), e + 5)];
    tg1 = (p2 - p) / norm(p2 - p);  nm = [-tg1(2), tg1(1)];
    plot(axM, p(1) + [-1 1]*22*nm(1), p(2) + [-1 1]*22*nm(2), '-', 'Color', cFg, 'LineWidth', 2.5);
end
tg0 = (C(2,:) - C(1,:)) / norm(C(2,:) - C(1,:));  n0 = [-tg0(2), tg0(1)];
plot(axM, C(1,1) + [-1 1]*26*n0(1), C(1,2) + [-1 1]*26*n0(2), '-', 'Color', cFg, 'LineWidth', 4);
text(axM, C(1,1) - 75*n0(1), C(1,2) - 75*n0(2), 'FINISH', 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', cFg);
axis(axM, 'equal');  axis(axM, 'off');
xl = xlim(axM);  yl = ylim(axM);
xlim(axM, xl + [-1 1]*170);  ylim(axM, yl + [-1 1]*170);

% --- Cards
axT = axes('Parent', fig, 'Position', [0.54 0.03 0.44 0.85], 'Color', cBg);
hold(axT, 'on');  axis(axT, 'off');  xlim(axT, [0 1]);  ylim(axT, [0 1]);
gap = 0.02;  h = (1 - gap*(nS-1)) / nS;
colX = [0.50 0.68 0.88];                       % Sim / Real / Diff. columns
for k = 1:nS
    y0 = 1 - k*h - (k-1)*gap;
    rectangle(axT, 'Position', [0 y0 1 h], 'FaceColor', cCard, 'EdgeColor', cEdge);
    rectangle(axT, 'Position', [0 y0 0.02 h], 'FaceColor', sectorColors(k,:), 'EdgeColor', 'none');
    text(axT, 0.04, y0 + h - 0.035, sprintf('SECTOR %d   %s', k, sectorNames{k}), ...
        'FontWeight', 'bold', 'FontSize', 13, 'Color', cFg, 'VerticalAlignment', 'middle');
    text(axT, colX(1), y0 + h - 0.085, 'Sim',  'Color', cDim, 'HorizontalAlignment', 'right');
    text(axT, colX(2), y0 + h - 0.085, 'Real', 'Color', cDim, 'HorizontalAlignment', 'right');
    text(axT, colX(3), y0 + h - 0.085, 'Diff.', 'Color', cDim, 'HorizontalAlignment', 'right');

    % label, sim, real, format, type (1 time, 2 speed, 3 G)
    rows = { 'Time [s]',           secSim(k),   secReal(k),   '%.2f', 1;
             'Max speed [km/h]',   S.vmax(k),   S.rvmax(k),   '%.0f', 2;
             'Min speed [km/h]',   S.vmin(k),   S.rvmin(k),   '%.0f', 2;
             'Avg speed [km/h]',   vavgS(k),    vavgR(k),     '%.0f', 2;
             'Max lateral G [g]',  S.glat(k),   S.rglat(k),   '%.1f', 3;
             'Max braking G [g]',  S.gbrk(k),   S.rgbrk(k),   '%.1f', 3 };
    nR = size(rows, 1);
    dy = (h - 0.13) / nR;
    for r = 1:nR
        yy = y0 + h - 0.12 - r*dy + dy/2;
        fmt = rows{r,4};
        d = rows{r,2} - rows{r,3};
        if rows{r,5} == 1
            if d > 0, cd = cBad; else, cd = cGood; end     % time: + is worse
        elseif rows{r,5} == 2
            if d < 0, cd = cBad; else, cd = cGood; end     % speed: - is worse
        else
            cd = cDim;                                     % G: no judgment
        end
        text(axT, 0.04, yy, rows{r,1}, 'Color', cFg);
        text(axT, colX(1), yy, sprintf(fmt, rows{r,2}), 'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'Color', cFg);
        text(axT, colX(2), yy, sprintf(fmt, rows{r,3}), 'HorizontalAlignment', 'right', 'Color', cDim);
        text(axT, colX(3), yy, sprintf(['%+' fmt(2:end)], d), 'HorizontalAlignment', 'right', 'Color', cd);
    end
end

sgtitle(fig, sprintf('Silverstone - lap %d: %.2f s   |   real %.2f s   |   diff. %+.2f s', ...
    lapToShow, simLapTime, realLapTime, simLapTime - realLapTime), 'FontWeight', 'bold', 'Color', cFg);