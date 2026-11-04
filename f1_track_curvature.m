%% f1_track_curvature.m
% Toma el centerline crudo de Silverstone (espaciado desigual) y produce:
%   s          - distancia acumulada, espaciado uniforme [m]
%   x, y       - coordenadas del circuito, remuestreadas uniformemente
%   curvature  - curvatura en cada punto [1/m] (positivo, sin signo)
%
% Esto es lo que el Piloto va a usar despues para el limite de velocidad
% por curva: a mayor curvatura, menor velocidad segura.

track = readmatrix('silverstone_centerline.csv');

%% Distancia acumulada sobre los puntos crudos (espaciado desigual)
d = sqrt(sum(diff(track).^2, 2));
s_raw = [0; cumsum(d)];

% Quitar puntos duplicados (distancia ~0 entre consecutivos, donde se
% unieron dos segmentos en el mismo punto exacto) - una spline no acepta
% valores repetidos en su eje independiente
keep = [true; diff(s_raw) > 1e-6];
s_raw = s_raw(keep);
track = track(keep, :);
totalLength = s_raw(end);

fprintf('Longitud total: %.3f km\n', totalLength/1000);

%% Remuestreo a espaciado uniforme (PERIODICO: el circuito es cerrado)
% v2: el circuito se extiende de forma periodica (se le "pegan" los puntos
% del final al inicio y viceversa) ANTES de remuestrear, suavizar y derivar,
% y al final se recorta. Sin esto, el filtro de suavizado y las derivadas
% tienen efectos de borde en la meta: el rumbo de la pista saltaba ~5 grados
% entre el ultimo y el primer punto (pico de direccion al cruzar la meta).
ds = 2; % [m] distancia entre puntos remuestreados - 2m es un buen punto
         % de partida: suficientemente fino para no perder detalle de
         % curvas cerradas, sin generar un arreglo enorme
pad   = 40;  % [puntos] relleno a cada lado (80 m) que se descarta al final
m_ext = 60;  % [puntos crudos] usados para extender el circuito (cubren >80 m)

% El ultimo punto crudo coincide con el primero (circuito cerrado): se
% excluye al extender para no duplicarlo.
s_ext     = [s_raw(end-m_ext:end-1) - totalLength; s_raw; s_raw(2:m_ext+1) + totalLength];
track_ext = [track(end-m_ext:end-1, :); track; track(2:m_ext+1, :)];

s_all = (-pad*ds : ds : totalLength + pad*ds)';

x = interp1(s_ext, track_ext(:,1), s_all, 'spline');
y = interp1(s_ext, track_ext(:,2), s_all, 'spline');

%% Suavizado antes de derivar
% Los puntos de OpenStreetMap tienen pequenas imprecisiones de mapeo, y
% la curvatura depende de la segunda derivada - eso amplifica muchisimo
% ese ruido. Sin este paso, la curvatura sale como picos aislados en vez
% de las subidas/bajadas suaves que una curva real produce.
% Ventana ~50m (25 puntos a 2m/punto), orden polinomial 3.
win = 25;
x = sgolayfilt(x, 3, win);
y = sgolayfilt(y, 3, win);

%% Curvatura por diferencias finitas
dx  = gradient(x, ds);
dy  = gradient(y, ds);
ddx = gradient(dx, ds);
ddy = gradient(dy, ds);

curvature = abs(dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;

%% Recortar al circuito (descarta el relleno periodico)
keepIdx   = (s_all >= 0) & (s_all < totalLength);
s         = s_all(keepIdx);
x         = x(keepIdx);
y         = y(keepIdx);
curvature = curvature(keepIdx);

fprintf('Puntos remuestreados: %d (cada %.1f m)\n', length(s), ds);

%% Verificacion visual: graficar curvatura junto al mapa
figure;

subplot(2,1,1);
plot(x, y, 'LineWidth', 1.5);
axis equal;
title('Silverstone - centerline remuestreado');
xlabel('X (m)'); ylabel('Y (m)');

subplot(2,1,2);
plot(s, curvature, 'LineWidth', 1);
title('Curvatura vs. distancia recorrida');
xlabel('Distancia (m)'); ylabel('Curvatura (1/m)');
grid on;

% (el guardado final, con nombres trackX/trackY/trackS, esta al final
% del script, despues de calcular tambien la velocidad limite)

%% Velocidad limite por curvatura (sin aero todavia - ver nota abajo)
% v_max = sqrt(a_lat_max / curvatura) - el limite clasico de "friction
% circle": a mayor curvatura, menor velocidad segura.
%
% a_lat_max = 1.5g es SOLO grip mecanico, sin downforce - un F1 real
% supera esto ampliamente en curvas rapidas gracias a la carga
% aerodinamica, que crece con la velocidad. Este numero va a quedar
% conservador en curvas rapidas (Copse, Maggotts-Becketts) hasta que
% exista el subsistema de Aerodinamica - las curvas lentas (Village,
% The Loop) ya son creibles desde ahora, porque ahi la aero pesa menos.
a_lat_max = 1.5 * 9.81; % [m/s^2] placeholder, sin aero

curvature_safe = max(curvature, 1e-6); % evitar division por cero en rectas
v_max = sqrt(a_lat_max ./ curvature_safe);

% Tope en rectas: sin esto, curvatura~0 da velocidades absurdas. Este
% valor NO sale de fisica real (potencia vs. arrastre) - es un techo
% arbitrario hasta que el balance motor-drag exista de verdad.
v_cap_kmh = 380;
v_max_kmh = min(v_max * 3.6, v_cap_kmh);

figure;
plot(s, v_max_kmh);
title('Velocidad limite vs. distancia (sin aero, placeholder)');
xlabel('Distancia (m)'); ylabel('Velocidad limite (km/h)');
grid on;

trackX = x;
trackY = y;
trackS = s;
trackPsi = atan2(gradient(trackY), gradient(trackX));
save('silverstone_curvature.mat', 'trackS', 'trackX', 'trackY', 'curvature', 'v_max_kmh', 'totalLength');
fprintf('Guardado: silverstone_curvature.mat (trackX, trackY, trackS, curvature, v_max_kmh)\n');