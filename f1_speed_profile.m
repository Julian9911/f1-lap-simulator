%% f1_speed_profile.m
% Perfil de velocidad de referencia para Silverstone (VelRef del piloto).
% Correr DESPUES de f1_vehicle_params (necesita trackS, trackX, trackY,
% curvature en el workspace). Produce: trackVref [m/s], mismo tamano que trackS.
%
% Idea fisica:
%  1) Limite en curva con carga aerodinamica. La aceleracion lateral
%     maxima es   v^2*kappa = mu*(g + k*v^2),   k = 0.5*rho*ClA/m
%     (la aero crece con v^2 y "pega" el auto al suelo). Despejando:
%         v = sqrt( mu*g / (kappa - mu*k) )     si kappa > mu*k
%     Si kappa <= mu*k la aero sostiene cualquier velocidad -> sin limite
%     de curva (solo manda el tope v_cap).
%  2) Pasada hacia atras (frenado): la velocidad en cada punto no puede
%     exceder lo que se alcanza a frenar hasta el punto siguiente:
%         v(i) <= sqrt( v(i+1)^2 + 2*a_brake*ds )
%     Se recorre el circuito dos veces (circular) para cerrar la vuelta.
%     Sin esto, VelRef caeria de golpe al llegar a la curva y el piloto
%     no tendria distancia para frenar.
%  (No se hace pasada hacia adelante: si VelRef sube mas rapido de lo que
%   el auto acelera, el piloto simplemente va a acelerador completo.)

%% Parametros (ajustables)
rho     = 1.225;      % [kg/m^3] densidad del aire
ClA     = 3.2 * 1.5;  % [m^2]    Cl*Af del Vehicle Body (= 4.8; fuentes: 3.2-6.4). v4: con FZMAX 15000 ya no hay bloqueos de rueda
m       = 770;        % [kg]
g       = 9.81;       % [m/s^2]
mu_v    = 1.7;        % v3 (con circulo de friccion): 1.7. Antes 1.5 (v1 validada: vuelta
                      % 133.69 s, error 1.49 m). Con 1.7-1.8 el auto sigue
                      % la pista pero aparecen disparos/bloqueos de rueda trasera
v_cap   = 90;         % [m/s] tope en rectas (~ velocidad maxima del auto)
a_brake = 20;         % [m/s^2] frenado del perfil = el que el auto logra en la sim (medido ~20; real ~22)

win_k   = 5;          % [puntos] ventana de suavizado de la curvatura (5 pts = 10 m; antes 15 = 30 m, inflaba curvas)

ds = trackS(2) - trackS(1);   % 2 m
N  = numel(trackS);

%% 0) Suavizado de la curvatura (quita el ruido de OpenStreetMap)
% El ruido de la curvatura hace que VelRef "tiemble" +-2-4 m/s en las
% zonas de curvas encadenadas, y el PI del piloto lo persigue alternando
% acelerador y freno. Primero un maximo movil (envolvente: conserva el
% pico de cada curva y no lo suaviza hacia abajo) y despues un promedio.
curvature_s = movmean(movmax(curvature(:), win_k), win_k);

%% 1) Limite de curva con aero
k   = 0.5 * rho * ClA / m;
den = curvature_s - mu_v * k;
vcurve = v_cap * ones(N, 1);
ok = den > 1e-9;
vcurve(ok) = sqrt(mu_v * g ./ den(ok));
trackVref = min(vcurve, v_cap);

%% 2) Pasada hacia atras (frenado), circular, dos vueltas
vcurve0 = trackVref;      % limite de curva puro (para la pasada hacia adelante)
for pass = 1:2
    for i = N:-1:1
        j = mod(i, N) + 1;                       % siguiente punto (circular)
        trackVref(i) = min(trackVref(i), sqrt(trackVref(j)^2 + 2*a_brake*ds));
    end
end

%% 3) Pasada hacia adelante (aceleracion), circular, dos vueltas
% El auto no puede salir de una curva a la velocidad de la recta siguiente
% al instante: su aceleracion maxima depende de la velocidad (motor - arrastre).
% v5: la tabla ya no se mide: se CALCULA desde el mapa del motor, la caja de 8 marchas y
% las resistencias, con los mismos parametros del modelo (reproducible). Validacion: con
% el motor anterior (1 MW, una marcha de 1.35) este calculo reproduce la tabla medida de la
% v4 con diferencias <= 0.4 m/s^2 entre 30 y 90 m/s.
%   a(v) = [ min(F_motor, F_tracccion) - F_aero - F_rodadura ] / m
eng_scale = 0.91;     % factor del torque del motor: 750 kW (reglamento 2026) a 12 000 rpm. DEBE coincidir con
                      % el factor aplicado a f_tbrake en el Mapped SI Engine ("]*3.897*0.91")
Re_w   = 0.3515;      % [m] radio efectivo de la rueda
N_diff = 4;  eta_tr = 0.9;
N_box  = [2.893 2.407 1.965 1.680 1.473 1.276 1.139];   % marchas 2..8 (= vector N de la transmision)
rpm_up = 11000;       % la marcha es la mas baja con rpm <= rpm_up (igual que f1_shift_logic)
rpm_bpt = [0 750 1053.57 1357.14 1660.71 1964.29 2267.86 2571.43 2875 3178.57 3482.14 3785.71 4089.29 4392.86 4696.43 5000] * 3.2;   % [rpm]
T_full  = [77.7345 77.7345 102.2072 120.9147 132.0395 143.8904 150.1347 155.4065 155.4065 156.9623 161.5149 166.2934 171.8297 173.7861 174.6472 174.6472] * 3.897;  % [N*m] fila de torque maximo del mapa
CdA    = 0.85;        % [m^2] modo recta (aero activa). Debe coincidir con CdA_recta de f1_aero_active.m (calibracion)
Q1 = 0.00699077564863536;  Q3 = 0.00153535478298811;  Q7 = 0.89443879773793;  Fz0 = 4300;
                      % rodadura: coeficientes QSY1, QSY3, QSY7 de la rueda. El termino QSY4 (v^4) no se
                      % incluye: con el la tabla calculada no reproduce la aceleracion medida
mu_trac = 1.4;  fr_rear = 0.55;   % agarre del limitador y reparto de masa trasero (traccion trasera)
v_tab = 0:2:100;
a_tab = zeros(size(v_tab));
for ii = 1:numel(v_tab)
    vv  = v_tab(ii);
    Lz  = m*g + 0.5*rho*ClA*vv^2;                          % carga vertical total
    Nsel = N_box(end);
    for kk = 1:numel(N_box)
        if vv / Re_w * N_box(kk) * N_diff * 30/pi <= rpm_up, Nsel = N_box(kk); break, end
    end
    rpm = vv / Re_w * Nsel * N_diff * 30/pi;
    Tq  = interp1(rpm_bpt, T_full, min(rpm, rpm_bpt(end))) * eng_scale;
    Tq  = Tq * min(1, max(0, (15000 - rpm) / 1000));       % limitador de rpm (14000-15000)
    Fm  = Tq * Nsel * N_diff * eta_tr / Re_w;
    Ft  = mu_trac * fr_rear * Lz;
    Fa  = 0.5 * rho * CdA * vv^2;
    Fr  = Lz * (Q1 + Q3*vv/16.67) * ((Lz/4)/Fz0)^Q7;
    a_tab(ii) = (min(Fm, Ft) - Fa - Fr) / m;
end
vf = vcurve0;
for pass = 1:2
    for i = 1:N
        j = mod(i, N) + 1;
        a_acc = interp1(v_tab, a_tab, min(vf(i), v_tab(end)), 'linear', 'extrap');
        vf(j) = min(vf(j), sqrt(vf(i)^2 + 2*a_acc*ds));
    end
end
% El perfil final cumple ambos limites: frenado (atras) y aceleracion (adelante)
trackVref = min(trackVref, vf);

%% Verificacion
fprintf('Vuelta ideal del perfil (alcanzable): %.2f s\n', sum(ds ./ trackVref));
fprintf('Perfil de velocidad: min %.1f m/s (%.0f km/h) en s = %.0f m | max %.1f m/s\n', ...
    min(trackVref), min(trackVref)*3.6, trackS(find(trackVref == min(trackVref), 1)), max(trackVref));

figure; plot(trackS, trackVref*3.6); grid on
xlabel('Distancia s [m]'); ylabel('VelRef [km/h]'); title('Perfil de velocidad de referencia');