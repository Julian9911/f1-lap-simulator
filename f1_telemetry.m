%% f1_telemetry.m
% Telemetria tipo equipo: simulador contra la vuelta real de Silverstone, en TRES
% graficos (fondo negro) con la distancia de vuelta como eje comun y los tres
% sectores como bandas de color. SUSTITUYE a f1_compare_real.m.
% No imprime nada en la consola. Post-proceso: no toca el modelo.
%
%   Grafico 1 - Ritmo:        velocidad, diferencia de tiempo, G lateral, G longitudinal
%   Grafico 2 - Pilotaje:     velocidad (referencia), acelerador, freno
%   Grafico 3 - Tren motriz:  velocidad (referencia), marcha, rpm, potencia del motor
%
% El zoom de un grafico se aplica a los tres (los ejes de distancia estan enlazados).
%
% Requisitos en el workspace: out (Single simulation output) y trackS
% (f1_vehicle_params). Senales registradas en el modelo (To Workspace, Timeseries):
%   xdot_log, carX_log, carY_log, lapCount   (ya existian)
%   acc_log, dec_log      AccelCmd / DecelCmd del Predictive Driver (0 a 1)
%   gear_log              marcha (2 a 8), de f1_shift_logic
%   engSpd_log [rad/s], engTrq_log [N*m]    del bus Info de la transmision
% Si falta una senal, su panel sale con el aviso "sin datos".
% En la carpeta: silverstone_centerline.csv y silverstone_real_telemetry.csv.

lapToShow = 0;                    % 0 = vuelta simulada mas rapida

% Sectores (los mismos que f1_sector_map.m): limites en metros de vuelta sobre
% silverstone_centerline.csv; APROXIMADOS (salen de una imagen de los sectores).
sectorEdges  = [1660 4420];
sectorColors = [0.90 0.30 0.18; 0.30 0.78 0.95; 1.00 0.82 0.20];

% Colores del tema oscuro
cBg  = [0.04 0.04 0.05];  cPanel = [0.075 0.075 0.09];  cFg = [0.95 0.95 0.95];
cDim = [0.62 0.62 0.67];  cGrid  = [0.20 0.20 0.24];
cReal = [0.85 0.85 0.89]; cSim = [1.00 0.48 0.27];

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

% Senales extra del simulador sobre la misma malla de tiempo
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
% G del simulador a partir de la trayectoria (suavizada ~0.5 s)
w = 25;
vms = movmean(vs/3.6, w);
hd  = unwrap(atan2(gradient(movmean(ys, w)), gradient(movmean(xs, w))));
ayS = abs(vms .* gradient(movmean(hd, w), dtGrid)) / 9.81;
axS = gradient(vms, dtGrid) / 9.81;

%% Telemetria real
R = readtable('silverstone_real_telemetry.csv');
realLapTime = R.Time_s(end);

%% Longitud del trazado y rejilla comun
C = readmatrix('silverstone_centerline.csv');
C = C([true; hypot(diff(C(:,1)), diff(C(:,2))) > 1e-6], :);
if norm(C(1,:) - C(end,:)) > 1, C = [C; C(1,:)]; end
Lc = sum(hypot(diff(C(:,1)), diff(C(:,2))));
if exist('totalLength', 'var'), L = totalLength; else, L = trackS(end) + 2; end
sg = (0:2:L-2)';
edG = [0, sectorEdges(:)'/Lc*L, L];             % sectores sobre la rejilla [m]

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

% Simulador sobre la rejilla de distancia
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

% Diferencia de tiempo acumulada (escalada a los tiempos de vuelta medidos)
tsim  = cumtrapz(sg, 1 ./ (vsim/3.6));   tsim  = tsim  * simLapTime  / tsim(end);
treal = cumtrapz(sg, 1 ./ (vreal/3.6));  treal = treal * realLapTime / treal(end);
dT = tsim - treal;                       % > 0: el simulador va por detras
dInc = diff([0; dT]);

% Tiempo perdido por tipo de tramo (segun la velocidad REAL), como en compare_real
tipoEdges = [0 150 250 inf];
lost = zeros(1, 3);
for k = 1:3
    lost(k) = sum(dInc(vreal >= tipoEdges(k) & vreal < tipoEdges(k+1)));
end

%% Paneles disponibles: {etiqueta, real, simulador, limites Y}
panel.vel = {'Velocidad [km/h]',    vreal, vsim,  [0 350]};
panel.dt  = {'Dif. tiempo [s]',     [],    dT,    []};
panel.ay  = {'G lateral [g]',       ayR,   aySg,  []};
panel.ax  = {'G longitud. [g]',     axR,   axSg,  []};
panel.thr = {'Acelerador [%]',      thrR,  thrS,  [-5 105]};
panel.brk = {'Freno [%]',           brkR,  brkS,  [-5 105]};
panel.ger = {'Marcha',              gerR,  gerS,  [1.5 8.5]};
panel.rpm = {'Motor [rpm]',         rpmR,  rpmSg, []};
panel.pwr = {'Potencia motor [kW]', [],    pwrSg, []};

% Grupos: nombre, paneles, alturas relativas
G = { 'Ritmo',       {'vel', 'dt', 'ay', 'ax'},        [3.0 1.8 1.4 1.4];
      'Pilotaje',    {'vel', 'thr', 'brk'},            [2.4 1.3 1.3];
      'Tren motriz', {'vel', 'ger', 'rpm', 'pwr'},     [1.8 1.1 1.4 1.3] };

axAll = gobjects(0);
for g = 1:size(G, 1)
    items = G{g, 2};  hr = G{g, 3};  nP = numel(items);
    top = 0.915;  bot = 0.075;  gp = 0.014;
    H = (top - bot - gp*(nP-1)) * hr / sum(hr);

    fig = figure('Name', ['Telemetria - ' G{g,1}], 'Color', cBg, ...
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
            text(ax(i), 0.5, 0.5, 'sin datos del simulador (senal no registrada)', 'Units', 'normalized', ...
                'Color', [1 0.38 0.33], 'HorizontalAlignment', 'center');
        end
        if ~isempty(spec{4}), set(ax(i), 'YLim', spec{4}); end

        if strcmp(key, 'dt')
            yline(ax(i), 0, '-', 'Color', cDim);
            text(ax(i), 0.01, 0.93, sprintf('Perdido por tipo de tramo:  lento (<150 km/h) %+.1f s   |   medio (150-250) %+.1f s   |   rapido (>250) %+.1f s', ...
                lost(1), lost(2), lost(3)), 'Units', 'normalized', 'Color', cFg, 'FontSize', 9, 'VerticalAlignment', 'top');
        elseif strcmp(key, 'ger')
            set(ax(i), 'YTick', 2:8);
        elseif strcmp(key, 'pwr')
            text(ax(i), 0.01, 0.93, 'solo simulador (el CSV real no trae potencia)', 'Units', 'normalized', ...
                'Color', cDim, 'FontSize', 9, 'VerticalAlignment', 'top');
        end
        if i == 1
            if any(~isnan(spec{3})) && hasReal
                lg = legend(ax(i), [hR hS], {'Real', 'Simulador'}, 'Location', 'southwest');
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
    xlabel(ax(nP), 'Distancia de vuelta [m]', 'Color', cFg);
    sgtitle(fig, sprintf('%s - Silverstone, vuelta %d: sim %.2f s  |  real %.2f s  |  dif. %+.2f s', ...
        G{g,1}, lapToShow, simLapTime, realLapTime, simLapTime - realLapTime), 'FontWeight', 'bold', 'Color', cFg);
    axAll = [axAll; ax];
end
linkaxes(axAll, 'x');
set(axAll, 'XLim', [0 L]);