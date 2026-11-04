%% f1_regression_test.m
% Prueba de regresion del simulador: despues de CUALQUIER cambio al modelo o a
% los parametros, corre f1_vehicle_params, simula 260 s y corre este script.
% Dice PASS / FAIL por criterio, para saber en segundos si algo se rompio.
%
% Requiere en el workspace: out (con carX_log, carY_log, xdot_log, lapCount),
% trackX, trackY, totalLength (de f1_vehicle_params).
%
% Los umbrales y la linea base estan abajo, con su justificacion. Si mejoras
% el modelo a proposito (p. ej. mas agarre), actualiza la LINEA BASE.

%% --- Linea base y umbrales ---------------------------------------------
base.lapTime   = 94.7;    % [s] v4 + LSD + Tair 288.15 K (rho consistente con el perfil): 94.71 s. (Antes Tair 273 K: 95.16 s. (v4 con Active Differential abierto: 94.86 s. (v3, ClA 2.7: 102.96 s. (v3 con h_cg 0.275: 102.45 s; v2 con useFC = false: 109.3 s)
tol.lapTime    = 1.5;     % [s] tolerancia; un cambio mayor = el comportamiento cambio
tol.latErrMax  = 3.0;     % [m] error lateral maximo (v4 + Tair 288 K: 2.03 m; antes 1.94 m, v3: 2.2 m, en la curva cerrada a ~1125 m; semiancho de la linea de carrera 4 m)
tol.latErrP95  = 1.8;     % [m] el 95 % del tiempo el auto sigue la linea con esta precision (v4 + Tair 288 K: 1.56 m; antes 1.51 m, v3: 1.55 m)
tol.lapSpread  = 1.5;     % [s] diferencia entre vueltas completas consecutivas (repetibilidad)
tol.lenErr     = 0.03;    % [-] la distancia recorrida debe ser +-3 % de la longitud del circuito
tol.vMax       = 95;      % [m/s] ~342 km/h: por encima, algo no fisico
tol.vMinRun    = 10;      % [m/s] el auto no debe detenerse durante una vuelta completa

%% --- Vueltas ----------------------------------------------------------
lcData = out.lapCount.Data(:);  lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);
tBounds = [0; tCross];  lapDur = diff(tBounds);
nLaps = numel(lapDur);

res = cell(0, 4);   % {nombre, valor (texto), umbral (texto), pasa}
addrow = @(r, n, v, t, ok) [r; {n, v, t, ok}];   % funcion anonima (no depende de funciones locales)

res = addrow(res, 'Vueltas completas >= 2', sprintf('%d', nLaps), '>= 2', nLaps >= 2);
if nLaps < 1
    error('f1_regression_test:sinVueltas', 'No se completo ninguna vuelta; simula mas tiempo (260 s).');
end

% Vuelta de evaluacion: la mas rapida entre las que NO son la salida parada
valid = 2:nLaps;  if isempty(valid), valid = 1; end
[~, ib] = min(lapDur(valid));  lap = valid(ib);
t0 = tBounds(lap);  t1 = tBounds(lap + 1);
res = addrow(res, 'Tiempo de vuelta', sprintf('%.2f s (vuelta %d)', t1 - t0, lap), ...
    sprintf('%.1f +- %.1f s', base.lapTime, tol.lapTime), abs((t1 - t0) - base.lapTime) <= tol.lapTime);

if numel(lapDur) >= 3
    sp = max(lapDur(2:end)) - min(lapDur(2:end));
    res = addrow(res, 'Repetibilidad entre vueltas', sprintf('%.2f s', sp), sprintf('< %.1f s', tol.lapSpread), sp < tol.lapSpread);
end

%% --- Senales en la vuelta evaluada --------------------------------------
tg = (t0:0.05:t1)';
xs = interp1(out.carX_log.Time(:),  out.carX_log.Data(:),  tg);
ys = interp1(out.carY_log.Time(:),  out.carY_log.Data(:),  tg);
vs = interp1(out.xdot_log.Time(:),  out.xdot_log.Data(:),  tg);

res = addrow(res, 'Senales sin NaN/Inf', sprintf('%d invalidas', nnz(~isfinite([xs; ys; vs]))), '0', all(isfinite([xs; ys; vs])));
res = addrow(res, 'Velocidad maxima', sprintf('%.0f km/h', max(vs)*3.6), sprintf('< %.0f km/h', tol.vMax*3.6), max(vs) < tol.vMax);
if lap > 1
    res = addrow(res, 'Sin detenciones (vuelta completa)', sprintf('v min %.0f km/h', min(vs)*3.6), sprintf('> %.0f km/h', tol.vMinRun*3.6), min(vs) > tol.vMinRun);
end

% Distancia recorrida vs longitud del circuito (detecta atajos o vueltas falsas)
dist = sum(hypot(diff(xs), diff(ys)));
lenErr = dist / totalLength - 1;
res = addrow(res, 'Distancia recorrida / circuito', sprintf('%.0f m (%+.1f %%)', dist, 100*lenErr), sprintf('+-%.0f %%', 100*tol.lenErr), abs(lenErr) <= tol.lenErr);

% Error lateral: distancia al segmento mas cercano de la linea de referencia
rx = [trackX(:); trackX(1)];  ry = [trackY(:); trackY(1)];
ax = rx(1:end-1);  ay = ry(1:end-1);  bx = rx(2:end);  by = ry(2:end);
ex = bx - ax;  ey = by - ay;  e2 = max(ex.^2 + ey.^2, eps);
latErr = zeros(size(xs));
for i = 1:numel(xs)
    u = min(max(((xs(i) - ax).*ex + (ys(i) - ay).*ey) ./ e2, 0), 1);
    latErr(i) = sqrt(min((xs(i) - ax - u.*ex).^2 + (ys(i) - ay - u.*ey).^2));
end
sl = sort(latErr);  p95 = sl(ceil(0.95*numel(sl)));   % sin Statistics Toolbox
res = addrow(res, 'Error lateral maximo', sprintf('%.2f m', max(latErr)), sprintf('< %.1f m', tol.latErrMax), max(latErr) < tol.latErrMax);
res = addrow(res, 'Error lateral p95', sprintf('%.2f m', p95), sprintf('< %.1f m', tol.latErrP95), p95 < tol.latErrP95);

% Informacion de diagnostico (no cuenta para PASS/FAIL): donde ocurre el error maximo
sAcum = [0; cumsum(hypot(diff(xs), diff(ys)))];
[~, im] = max(latErr);
fprintf('\n[info] Error lateral maximo (%.2f m) a los %.0f m de la vuelta, a %.0f km/h\n', latErr(im), sAcum(im), vs(im)*3.6);
big = latErr > 1.5;
if any(big), fprintf('[info] Tramo con error > 1.5 m: %.0f m en total (de %.0f m)\n', nnz(big)*mean(diff(sAcum)), sAcum(end)); end

%% --- Informe ---------------------------------------------------------------
fprintf('\n=== f1_regression_test ===\n');
fprintf('%-34s %-22s %-18s %s\n', 'Criterio', 'Valor', 'Umbral', 'Resultado');
for k = 1:size(res, 1)
    if res{k,4}, s = 'PASS'; else, s = 'FAIL'; end
    fprintf('%-34s %-22s %-18s %s\n', res{k,1}, res{k,2}, res{k,3}, s);
end
allOk = all([res{:,4}]);
if allOk, fprintf('\nRESULTADO: TODO OK (%d/%d)\n', size(res,1), size(res,1));
else,     fprintf('\nRESULTADO: %d criterio(s) FALLARON\n', nnz(~[res{:,4}])); end