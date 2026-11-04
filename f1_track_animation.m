%% f1_track_animation.m   (v2: usa los DATOS REALES de la simulacion)
% Reproduce una vuelta simulada sobre Silverstone: el punto sigue la
% trayectoria REAL del auto (carX_log, carY_log) y la linea se colorea con
% la velocidad REAL (xdot_log). Ya no usa la velocidad limite teorica.
%
% Requisitos (despues de simular, con "Single simulation output"):
%   out.carX_log, out.carY_log, out.xdot_log   (To Workspace, Timeseries)
%   out.lapCount                               (de f1_lap_timer)
%   trackX, trackY en el workspace             (f1_vehicle_params)
%
% Parametros:
lapToShow = 0;     % 0 = la vuelta mas rapida; N = esa vuelta (1 = con arranque parado)
playSpeed = 1;     % 1 = tiempo real; 2 = el doble de rapido, etc.

if ~exist('out', 'var')
    error('No existe "out" en el workspace: simula primero (Single simulation output activado).');
end

%% Ventana de la vuelta (cruces de meta segun lapCount, igual que f1_lap_table)
lcData = out.lapCount.Data(:);
lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);       % instantes en que se cierra cada vuelta
tBounds = [0; tCross];                              % vuelta k = [tBounds(k), tBounds(k+1)]
lapDur  = diff(tBounds);
if isempty(lapDur)
    error('No se completo ninguna vuelta en esta simulacion.');
end
if lapToShow == 0
    [~, lapToShow] = min(lapDur);
end
t0 = tBounds(lapToShow);  t1 = tBounds(lapToShow + 1);
lapTime = t1 - t0;

%% Datos reales de esa vuelta, en una rejilla uniforme
dtGrid = 0.02;                                      % [s]
tg = (t0:dtGrid:t1)';
x  = interp1(out.carX_log.Time(:), out.carX_log.Data(:), tg);
y  = interp1(out.carY_log.Time(:), out.carY_log.Data(:), tg);
v_kmh = interp1(out.xdot_log.Time(:), out.xdot_log.Data(:), tg) * 3.6;
s  = [0; cumsum(hypot(diff(x), diff(y)))];          % distancia recorrida [m]

fprintf('Vuelta %d: %.2f s | v max %.0f km/h | v min %.0f km/h | distancia %.0f m\n', ...
    lapToShow, lapTime, max(v_kmh), min(v_kmh), s(end));

%% Paleta (rojo = lento -> amarillo -> verde -> cian -> azul = rapido)
keyIdx = [1 20 39 57 64];
keyRGB = [0.9325 0.1675 0.1675;
    0.9325 0.9325 0.1675;
    0.1675 0.9325 0.1675;
    0.1675 0.9325 0.9325;
    0.1675 0.6775 0.9325];
speedColormap = interp1(keyIdx, keyRGB, 1:64);

%% Figura
bgColor = [0.0431 0.0510 0.0706]; % #0b0d12
fig = figure('Color', bgColor, 'Name', 'F1 Model Simulator - Silverstone');
ax = axes('Parent', fig, 'Color', bgColor);
hold(ax, 'on'); axis(ax, 'equal');
ax.XColor = 'w'; ax.YColor = 'w';
title(ax, sprintf('Silverstone Circuit - F1 Model Simulator (vuelta %d, %.2f s)', lapToShow, lapTime), 'Color', 'w');
xlabel(ax, 'X (m)', 'Color', 'w');
ylabel(ax, 'Y (m)', 'Color', 'w');

% Centerline de referencia (gris tenue) debajo de la trayectoria real
if exist('trackX', 'var') && exist('trackY', 'var')
    plot(ax, [trackX(:); trackX(1)], [trackY(:); trackY(1)], '-', 'Color', [0.25 0.27 0.32], 'LineWidth', 1);
end

% Trayectoria REAL coloreada con la velocidad REAL
patch(ax, [x; NaN], [y; NaN], [v_kmh; NaN], ...
    'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 4);
colormap(ax, speedColormap);
clim(ax, [0, ceil(max(v_kmh)/10)*10]);
cb = colorbar(ax);
cb.Label.String = 'Velocidad real (km/h)';
cb.Label.Color = 'w';
cb.Color = 'w';

carMarker = plot(ax, x(1), y(1), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
hudText = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
    'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold', 'VerticalAlignment', 'top');

%% Animacion (en loop hasta cerrar la figura)
tStart = tic;
while ishandle(fig)
    elapsed = mod(toc(tStart) * playSpeed, lapTime);
    idx = min(numel(tg), max(1, floor(elapsed / dtGrid) + 1));

    set(carMarker, 'XData', x(idx), 'YData', y(idx));
    set(hudText, 'String', sprintf('Velocidad: %.0f km/h\nTiempo: %.1f s\nDistancia: %.0f m', ...
        v_kmh(idx), elapsed, s(idx)));
    drawnow limitrate;
end