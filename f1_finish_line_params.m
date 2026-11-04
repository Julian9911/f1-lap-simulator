%% f1_finish_line_params.m
% Calcula la geometria de la linea de meta a partir del circuito real
% (silverstone_centerline.csv, ya con la meta real en el indice 1).
%
% startX, startY  - punto de la meta
% tangentX/Y      - direccion de avance en la meta (vector unitario)
% normalX/Y       - perpendicular a la direccion de avance (vector unitario)
%
% Estas 6 constantes son las que alimentan al MATLAB Function block
% que detecta el cruce de meta.

track = readmatrix('silverstone_centerline.csv');

startX = track(1,1);
startY = track(1,2);

dirVec = track(2,:) - track(1,:);
dirVec = dirVec / norm(dirVec);
tangentX = dirVec(1);
tangentY = dirVec(2);

% Normal = tangente rotada 90 grados
normalX = -tangentY;
normalY = tangentX;

% Orientacion inicial del auto (psi_o para el Vehicle Body) - debe
% apuntar en la direccion real de la pista en la meta, no en un valor
% generico. Sin esto, el auto arranca mirando hacia un lado que no
% corresponde con hacia donde el circuito realmente va desde ahi.
psi_o = atan2(tangentY, tangentX);

fprintf('psi_o (orientacion inicial): %.3f rad (%.1f grados)\n', psi_o, rad2deg(psi_o));

fprintf('Meta en (%.1f, %.1f)\n', startX, startY);
fprintf('Direccion de avance: (%.3f, %.3f)\n', tangentX, tangentY);