%% f1_lateral_tire_table.m
% Tabla de rigidez de curva Cy(alpha) para el Vehicle Body 3DOF (modo
% "Mapped slip angle"), de modo que la fuerza lateral SATURE.
%
% Fisica (documentacion del bloque, por rueda):
%     Fy = Cy(alpha) * alpha * mu * Fz / (2*Fznom)
% Entonces el agarre lateral usado por eje, mu_y = Fy/Fz, vale
%     mu_y(alpha) = Cy(alpha) * alpha / (2*Fznom)           (mu = 1)
% Se disena primero mu_y(alpha) con la forma de la Magic Formula
%     mu_y = D*sin( C*atan( B*a - E*(B*a - atan(B*a)) ) )
% y luego se despeja la tabla:   Cy(alpha) = 2*Fznom*mu_y(alpha)/alpha.
%
% Parametros de diseno (ajustables):
%   mu_pk  : pico de Fy/Fz (D). Delantero 1.7 y trasero 1.8 -> el eje
%            delantero satura primero (subviraje en el limite = estable).
%   k      : pendiente inicial de Fy/Fz [1/rad] = rigidez / carga. El valor
%            actual del modelo es ~8 (delantero) y ~10 (trasero): demasiado
%            blando, el auto necesitaria 10-12 grados de deriva para dar
%            1.7 de agarre. Un neumatico de F1 es ~20-25.
%   C, E   : forma. C controla el "codo"; E negativo adelanta el pico y
%            deja una caida suave despues (no hay acantilado: evita
%            inestabilidades numericas y de manejo).

%% Parametros
Fznom   = 4000;      % [N] igual que en el Vehicle Body (Fznom)
mu_pk_f = 1.7;   k_f = 20;     % eje delantero
mu_pk_r = 1.8;   k_r = 24;     % eje trasero
C = 1.3;  E = -2;

%% Tabla (simetrica, para que funcione con alpha negativo)
a_pos     = linspace(0, 0.8, 81);              % 0 a 0.8 rad (0 a 46 grados)
alpha_brk = [-fliplr(a_pos(2:end)), a_pos];    % 161 puntos

Bf = k_f / (C * mu_pk_f);
Br = k_r / (C * mu_pk_r);
muF = @(a) mu_pk_f * sin(C * atan(Bf*a - E*(Bf*a - atan(Bf*a))));
muR = @(a) mu_pk_r * sin(C * atan(Br*a - E*(Br*a - atan(Br*a))));

Cy_f_tab = 2 * Fznom * muF(alpha_brk) ./ alpha_brk;
Cy_r_tab = 2 * Fznom * muR(alpha_brk) ./ alpha_brk;
Cy_f_tab(alpha_brk == 0) = 2 * Fznom * k_f;    % limite alpha -> 0
Cy_r_tab(alpha_brk == 0) = 2 * Fznom * k_r;

%% Resumen
[pkF, iF] = max(muF(a_pos));  [pkR, iR] = max(muR(a_pos));
fprintf('Delantero: Cy(0) = %.0f N/rad | pico mu_y = %.2f en %.1f grados\n', Cy_f_tab(alpha_brk==0), pkF, rad2deg(a_pos(iF)));
fprintf('Trasero:   Cy(0) = %.0f N/rad | pico mu_y = %.2f en %.1f grados\n', Cy_r_tab(alpha_brk==0), pkR, rad2deg(a_pos(iR)));
fprintf('Tabla: %d puntos, alpha de %.2f a %.2f rad\n', numel(alpha_brk), alpha_brk(1), alpha_brk(end));

%% Grafica de verificacion (comparada con el modelo lineal actual)
figure;
subplot(2,1,1);
plot(rad2deg(a_pos), muF(a_pos), 'b', rad2deg(a_pos), muR(a_pos), 'r', ...
     rad2deg(a_pos), 65e3/(2*Fznom)*a_pos, 'b--', rad2deg(a_pos), 80e3/(2*Fznom)*a_pos, 'r--');
ylim([0 2.5]); grid on; xlabel('alpha [grados]'); ylabel('mu_y = Fy/Fz');
legend('delantero (nuevo)', 'trasero (nuevo)', 'delantero (lineal actual)', 'trasero (lineal actual)', 'Location', 'southeast');
title('Agarre lateral vs. angulo de deriva');
subplot(2,1,2);
plot(rad2deg(a_pos), Cy_f_tab(alpha_brk >= 0), 'b', rad2deg(a_pos), Cy_r_tab(alpha_brk >= 0), 'r'); grid on
xlabel('alpha [grados]'); ylabel('Cy(alpha) [N/rad]'); title('Tabla que va al Vehicle Body');
