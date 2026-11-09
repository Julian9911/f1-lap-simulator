%% f1_lateral_tire_table.m
% Cornering stiffness table Cy(alpha) for the Vehicle Body 3DOF ("Mapped
% slip angle" mode), so that the lateral force SATURATES.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Physics (block documentation, per wheel):
%     Fy = Cy(alpha) * alpha * mu * Fz / (2*Fznom)
% Then the lateral grip used per axle, mu_y = Fy/Fz, is
%     mu_y(alpha) = Cy(alpha) * alpha / (2*Fznom)           (mu = 1)
% mu_y(alpha) is designed first with the shape of the Magic Formula
%     mu_y = D*sin( C*atan( B*a - E*(B*a - atan(B*a)) ) )
% and then the table is solved:   Cy(alpha) = 2*Fznom*mu_y(alpha)/alpha.
%
% Design parameters (adjustable):
%   mu_pk  : peak of Fy/Fz (D). Front 1.7 and rear 1.8 -> the front axle
%            saturates first (understeer at the limit = stable).
%   k      : initial slope of Fy/Fz [1/rad] = stiffness / load. The previous
%            value of the model was ~8 (front) and ~10 (rear): too soft, the
%            car would need 10-12 degrees of slip to give 1.7 of grip. An
%            F1 tire is ~20-25.
%   C, E   : shape. C controls the "knee"; a negative E brings the peak
%            forward and leaves a smooth drop afterwards (no cliff: it avoids
%            numerical and handling instabilities).

%% Parameters
Fznom   = tire.Fznom; % [N] same as in the Vehicle Body (Fznom); defined in f1_vehicle_params.m
mu_pk_f = tire.mu_pk_f;   k_f = 20;     % front axle
mu_pk_r = tire.mu_pk_r;   k_r = 24;     % rear axle
C = 1.3;  E = -2;

%% Table (symmetric, so it works with negative alpha)
a_pos     = linspace(0, 0.8, 81);              % 0 to 0.8 rad (0 to 46 degrees)
alpha_brk = [-fliplr(a_pos(2:end)), a_pos];    % 161 points

Bf = k_f / (C * mu_pk_f);
Br = k_r / (C * mu_pk_r);
muF = @(a) mu_pk_f * sin(C * atan(Bf*a - E*(Bf*a - atan(Bf*a))));
muR = @(a) mu_pk_r * sin(C * atan(Br*a - E*(Br*a - atan(Br*a))));

Cy_f_tab = 2 * Fznom * muF(alpha_brk) ./ alpha_brk;
Cy_r_tab = 2 * Fznom * muR(alpha_brk) ./ alpha_brk;
Cy_f_tab(alpha_brk == 0) = 2 * Fznom * k_f;    % limit alpha -> 0
Cy_r_tab(alpha_brk == 0) = 2 * Fznom * k_r;

%% Summary
[pkF, iF] = max(muF(a_pos));  [pkR, iR] = max(muR(a_pos));
fprintf('Front: Cy(0) = %.0f N/rad | peak mu_y = %.2f at %.1f degrees\n', Cy_f_tab(alpha_brk==0), pkF, rad2deg(a_pos(iF)));
fprintf('Rear:  Cy(0) = %.0f N/rad | peak mu_y = %.2f at %.1f degrees\n', Cy_r_tab(alpha_brk==0), pkR, rad2deg(a_pos(iR)));
fprintf('Table: %d points, alpha from %.2f to %.2f rad\n', numel(alpha_brk), alpha_brk(1), alpha_brk(end));

%% Verification plot (compared with the previous linear model)
figure;
subplot(2,1,1);
plot(rad2deg(a_pos), muF(a_pos), 'b', rad2deg(a_pos), muR(a_pos), 'r', ...
    rad2deg(a_pos), 65e3/(2*Fznom)*a_pos, 'b--', rad2deg(a_pos), 80e3/(2*Fznom)*a_pos, 'r--');
ylim([0 2.5]); grid on; xlabel('alpha [degrees]'); ylabel('mu_y = Fy/Fz');
legend('front (new)', 'rear (new)', 'front (previous linear)', 'rear (previous linear)', 'Location', 'southeast');
title('Lateral grip vs. slip angle');
subplot(2,1,2);
plot(rad2deg(a_pos), Cy_f_tab(alpha_brk >= 0), 'b', rad2deg(a_pos), Cy_r_tab(alpha_brk >= 0), 'r'); grid on
xlabel('alpha [degrees]'); ylabel('Cy(alpha) [N/rad]'); title('Table that goes to the Vehicle Body');