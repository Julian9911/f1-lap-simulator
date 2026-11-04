%% f1_load_raceline.m
% Reemplaza la referencia del piloto (centerline) por la LINEA DE CARRERA
% generada con f1_make_raceline.py (silverstone_raceline.csv).
%
% Correr DESPUES de f1_track_curvature.m (el maestro f1_vehicle_params.m ya lo
% hace si useRaceline = true). Sobrescribe en el workspace:
%   trackX, trackY   - linea de carrera, cada 2 m, circuito cerrado
%   trackS           - distancia acumulada [m]
%   curvature        - curvatura de la linea de carrera [1/m] (sin signo)
%   trackPsi         - rumbo de la linea [rad] = atan2(dy, dx), calculado
%                      con extension periodica (sin salto en la meta)
%   totalLength      - longitud de la linea de carrera [m]
%
% El modelo de Simulink NO cambia: el piloto sigue trackX/trackY/trackPsi y
% el perfil de velocidad (f1_speed_profile.m) se recalcula con la nueva
% curvatura. Para volver a la v1 (centerline): useRaceline = false en el maestro.

rl = readmatrix('silverstone_raceline.csv');
x0 = rl(:,1);  y0 = rl(:,2);
n0 = numel(x0);
ds = 2;                         % [m] mismo espaciado que f1_track_curvature
pad = 40;                       % [puntos] extension periodica

% Extension periodica para derivar sin efectos de borde en la meta
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

% Velocidad limite TEORICA por curvatura (misma formula que f1_track_curvature.m).
% La usa el bloque heredado f1_speed_target (Pilot/MATLAB Function2) y debe
% tener el MISMO numero de puntos que trackS, o da "Index exceeds array
% dimensions" (el centerline tenia 2929 puntos y la linea de carrera 2894).
a_lat_max = 1.5 * 9.81;                                   % [m/s^2] igual que en f1_track_curvature.m
v_max_kmh = min(sqrt(a_lat_max ./ max(curvature, 1e-6)) * 3.6, 380);

fprintf('Linea de carrera: %d puntos, %.0f m | curvatura p95 %.4f 1/m | salto de rumbo en la meta %.2f grados\n', ...
    n0, totalLength, prctile(curvature, 95), rad2deg(abs(atan2(sin(trackPsi(1)-trackPsi(end)), cos(trackPsi(1)-trackPsi(end))))));