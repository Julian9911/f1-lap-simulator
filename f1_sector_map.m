%% f1_sector_map.m
% Un solo grafico (fondo negro): mapa del circuito con la linea central de color
% por sector y, al lado, una tarjeta por sector con la informacion de ingenieria:
% tiempo, velocidades y fuerzas G, simulador contra vuelta real.
% No imprime nada en la consola. Post-proceso: no toca el modelo.
%
% Requisitos en el workspace: out (Single simulation output) y trackS
% (f1_vehicle_params). En la carpeta: silverstone_centerline.csv y
% silverstone_real_telemetry.csv.

lapToShow = 0;                    % 0 = vuelta simulada mas rapida

% Limites de sector en metros de vuelta (desde la meta, sobre silverstone_centerline.csv).
% APROXIMADOS: salen de ubicar los cortes de color de una imagen de los sectores
% sobre el trazado (cortes en la recta Wellington y al inicio de la recta
% Hangar), no de coordenadas oficiales. Ajustalos si tienes los oficiales.
sectorEdges  = [1660 4420];
sectorNames  = {'Abbey - Wellington', 'Brooklands - Chapel', 'Hangar - Club'};
sectorColors = [0.90 0.30 0.18;   % sector 1
                0.30 0.78 0.95;   % sector 2
                1.00 0.82 0.20];  % sector 3

% Colores del tema oscuro
cBg   = [0.04 0.04 0.05];   cFg   = [0.95 0.95 0.95];   cDim  = [0.62 0.62 0.67];
cCard = [0.11 0.11 0.13];   cEdge = [0.28 0.28 0.32];
cGood = [0.35 0.85 0.50];   cBad  = [1.00 0.38 0.33];

%% Vuelta simulada (misma ventana que f1_compare_real.m)
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

%% Telemetria real
R = readtable('silverstone_real_telemetry.csv');
realLapTime = R.Time_s(end);

%% Trazado (linea central) y longitud
C = readmatrix('silverstone_centerline.csv');
% El CSV trae puntos consecutivos repetidos: se eliminan (interp1 exige distancias unicas)
C = C([true; hypot(diff(C(:,1)), diff(C(:,2))) > 1e-6], :);
if norm(C(1,:) - C(end,:)) > 1, C = [C; C(1,:)]; end
dC = [0; cumsum(hypot(diff(C(:,1)), diff(C(:,2))))];
Lc = dC(end);

%% Posicion de la vuelta real como fraccion de vuelta (alineada por posicion)
if exist('totalLength', 'var'), L = totalLength; else, L = trackS(end) + 2; end
if ismember('S_aligned', R.Properties.VariableNames)
    frR = mod(R.S_aligned(:), L) / L;
else
    frR = R.Distance_m(:) / R.Distance_m(end);
end

%% Estadisticas por sector
edF = [0, sectorEdges(:)'/Lc, 1];            % limites como fraccion de vuelta
nS  = numel(edF) - 1;
secLen = diff(edF(:)) * Lc;                  % longitud de cada sector [m]

% Tiempo de cada sector, simulado
fr = ss / ss(end);
tt = tg - t0;
[frU, iu] = unique(fr);
tEd = interp1(frU, tt(iu), edF, 'linear', 'extrap');
tEd(1) = 0;  tEd(end) = simLapTime;
secSim = diff(tEd(:));

% Tiempo de cada sector, real: tiempo de la telemetria en la posicion alineada
% La vuelta real no empieza en la meta (S_aligned parte de ~84 m): se desenrolla
% la posicion (u crece de forma continua) y se repite un ciclo para poder medir
% sectores que cruzan la meta.
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

% Fuerzas G del simulador a partir de la trayectoria (suavizada ~0.5 s)
w = 25;
vms = movmean(vs/3.6, w);
hd  = unwrap(atan2(gradient(movmean(ys, w)), gradient(movmean(xs, w))));
ayG = abs(vms .* gradient(movmean(hd, w), dtGrid)) / 9.81;
axG = gradient(vms, dtGrid) / 9.81;

% Fuerzas G de la vuelta real: aceleraciones del CSV (AccX, AccY, m/s^2)
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
% Velocidad media = distancia del sector / tiempo del sector (igual para sim y real)
vavgS = secLen ./ secSim * 3.6;
vavgR = secLen ./ secReal * 3.6;

%% Figura
fig = figure('Name', 'Sectores', 'Color', cBg, 'Position', [60 60 1400 720], 'InvertHardcopy', 'off');

% --- Mapa
axM = axes('Parent', fig, 'Position', [0.02 0.03 0.50 0.85], 'Color', cBg);
hold(axM, 'on');
plot(axM, C(:,1), C(:,2), 'Color', [0.40 0.40 0.44], 'LineWidth', 14);   % borde del asfalto
plot(axM, C(:,1), C(:,2), 'Color', [0.10 0.10 0.11], 'LineWidth', 11);   % asfalto
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
% Cortes de sector (marca blanca) y meta
for e = sectorEdges(:)'
    p  = [interp1(dC, C(:,1), e), interp1(dC, C(:,2), e)];
    p2 = [interp1(dC, C(:,1), e + 5), interp1(dC, C(:,2), e + 5)];
    tg1 = (p2 - p) / norm(p2 - p);  nm = [-tg1(2), tg1(1)];
    plot(axM, p(1) + [-1 1]*22*nm(1), p(2) + [-1 1]*22*nm(2), '-', 'Color', cFg, 'LineWidth', 2.5);
end
tg0 = (C(2,:) - C(1,:)) / norm(C(2,:) - C(1,:));  n0 = [-tg0(2), tg0(1)];
plot(axM, C(1,1) + [-1 1]*26*n0(1), C(1,2) + [-1 1]*26*n0(2), '-', 'Color', cFg, 'LineWidth', 4);
text(axM, C(1,1) - 75*n0(1), C(1,2) - 75*n0(2), 'META', 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', cFg);
axis(axM, 'equal');  axis(axM, 'off');
xl = xlim(axM);  yl = ylim(axM);
xlim(axM, xl + [-1 1]*170);  ylim(axM, yl + [-1 1]*170);

% --- Tarjetas
axT = axes('Parent', fig, 'Position', [0.54 0.03 0.44 0.85], 'Color', cBg);
hold(axT, 'on');  axis(axT, 'off');  xlim(axT, [0 1]);  ylim(axT, [0 1]);
gap = 0.02;  h = (1 - gap*(nS-1)) / nS;
colX = [0.50 0.68 0.88];                       % columnas Sim / Real / Dif.
for k = 1:nS
    y0 = 1 - k*h - (k-1)*gap;
    rectangle(axT, 'Position', [0 y0 1 h], 'FaceColor', cCard, 'EdgeColor', cEdge);
    rectangle(axT, 'Position', [0 y0 0.02 h], 'FaceColor', sectorColors(k,:), 'EdgeColor', 'none');
    text(axT, 0.04, y0 + h - 0.035, sprintf('SECTOR %d   %s', k, sectorNames{k}), ...
        'FontWeight', 'bold', 'FontSize', 13, 'Color', cFg, 'VerticalAlignment', 'middle');
    text(axT, colX(1), y0 + h - 0.085, 'Sim',  'Color', cDim, 'HorizontalAlignment', 'right');
    text(axT, colX(2), y0 + h - 0.085, 'Real', 'Color', cDim, 'HorizontalAlignment', 'right');
    text(axT, colX(3), y0 + h - 0.085, 'Dif.', 'Color', cDim, 'HorizontalAlignment', 'right');

    % etiqueta, sim, real, formato, tipo (1 tiempo, 2 velocidad, 3 G)
    rows = { 'Tiempo [s]',        secSim(k),   secReal(k),   '%.2f', 1;
             'Vel. max [km/h]',   S.vmax(k),   S.rvmax(k),   '%.0f', 2;
             'Vel. min [km/h]',   S.vmin(k),   S.rvmin(k),   '%.0f', 2;
             'Vel. media [km/h]', vavgS(k),    vavgR(k),     '%.0f', 2;
             'G lateral max [g]', S.glat(k),   S.rglat(k),   '%.1f', 3;
             'G frenada max [g]', S.gbrk(k),   S.rgbrk(k),   '%.1f', 3 };
    nR = size(rows, 1);
    dy = (h - 0.13) / nR;
    for r = 1:nR
        yy = y0 + h - 0.12 - r*dy + dy/2;
        fmt = rows{r,4};
        d = rows{r,2} - rows{r,3};
        if rows{r,5} == 1
            if d > 0, cd = cBad; else, cd = cGood; end     % tiempo: + es peor
        elseif rows{r,5} == 2
            if d < 0, cd = cBad; else, cd = cGood; end     % velocidad: - es peor
        else
            cd = cDim;                                     % G: sin juicio
        end
        text(axT, 0.04, yy, rows{r,1}, 'Color', cFg);
        text(axT, colX(1), yy, sprintf(fmt, rows{r,2}), 'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'Color', cFg);
        text(axT, colX(2), yy, sprintf(fmt, rows{r,3}), 'HorizontalAlignment', 'right', 'Color', cDim);
        text(axT, colX(3), yy, sprintf(['%+' fmt(2:end)], d), 'HorizontalAlignment', 'right', 'Color', cd);
    end
end

sgtitle(fig, sprintf('Silverstone - vuelta %d: %.2f s   |   real %.2f s   |   dif. %+.2f s', ...
    lapToShow, simLapTime, realLapTime, simLapTime - realLapTime), 'FontWeight', 'bold', 'Color', cFg);
