%% f1_speed_profile.m
% Reference speed profile for Silverstone (VelRef of the pilot).
% Run AFTER the reference line is loaded (it needs trackS, trackX, trackY
% and curvature in the workspace; f1_vehicle_params does this for you).
% Produces: trackVref [m/s], same size as trackS.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Physical idea:
%  1) Corner limit with aerodynamic load. The maximum lateral acceleration
%     is   v^2*kappa = mu*(g + k*v^2),   k = 0.5*rho*ClA/m
%     (the aero grows with v^2 and "presses" the car on the ground). Solving:
%         v = sqrt( mu*g / (kappa - mu*k) )     if kappa > mu*k
%     If kappa <= mu*k the aero sustains any speed -> no corner limit
%     (only the cap v_cap applies).
%  2) Backward pass (braking): the speed at each point cannot exceed what
%     can be braked down to the next point:
%         v(i) <= sqrt( v(i+1)^2 + 2*a_brake*ds )
%     The circuit is traversed twice (circular) to close the lap.
%     Without this, VelRef would drop suddenly on arriving at the corner
%     and the pilot would have no distance to brake.
%  3) Forward pass (acceleration): the car cannot leave a corner at the
%     speed of the next straight instantly; see section 3 below.

%% Parameters (adjustable)
% Values SHARED with the model come from f1_vehicle_params.m (structs veh, aero, pwr,
% tire): one single source, so there is no second place to edit. The variable names
% are kept as before, so nothing else in the project changes.
rho     = aero.rho;   % [kg/m^3] air density
ClA     = aero.ClA;   % [m^2]    Cl*Af of the Vehicle Body (= 4.8; sources: 3.2-6.4). With FZMAX 15000 there are no wheel lockups
m       = veh.m;      % [kg]
g       = 9.81;       % [m/s^2]
mu_v    = tire.mu_pk_f;           % 1.7 with the friction circle. It was 1.5 in validated v1 (lap
                      % 133.69 s, error 1.49 m). With 1.7-1.8 the car still
                      % follows the track but rear wheel spikes/lockups appear
v_cap   = 90;         % [m/s] cap on straights (~ top speed of the car)
a_brake = 20;         % [m/s^2] braking of the profile = what the car achieves in the sim (measured ~20; real ~22)

win_k   = 5;          % [points] curvature smoothing window (5 pts = 10 m; 15 = 30 m used to inflate corners)

ds = trackS(2) - trackS(1);   % 2 m
N  = numel(trackS);

%% 0) Curvature smoothing (removes OpenStreetMap noise)
% Curvature noise makes VelRef "shake" +-2-4 m/s in chained corners, and the
% pilot controller chases it by alternating throttle and brake. First a
% moving maximum (envelope: it keeps the peak of each corner and does not
% smooth it downwards) and then a moving average.
curvature_s = movmean(movmax(curvature(:), win_k), win_k);

%% 1) Corner limit with aero
k   = 0.5 * rho * ClA / m;
den = curvature_s - mu_v * k;
vcurve = v_cap * ones(N, 1);
ok = den > 1e-9;
vcurve(ok) = sqrt(mu_v * g ./ den(ok));
trackVref = min(vcurve, v_cap);

%% 2) Backward pass (braking), circular, two laps
vcurve0 = trackVref;      % pure corner limit (for the forward pass)
for pass = 1:2
    for i = N:-1:1
        j = mod(i, N) + 1;                       % next point (circular)
        trackVref(i) = min(trackVref(i), sqrt(trackVref(j)^2 + 2*a_brake*ds));
    end
end

%% 3) Forward pass (acceleration), circular, two laps
% The car cannot leave a corner at the speed of the next straight
% instantly: its maximum acceleration depends on speed (engine - drag).
% The table is not measured: it is COMPUTED from the engine map, the 8-speed
% gearbox and the resistances, with the same parameters as the model
% (reproducible). Validation: with the earlier engine (1 MW, a single gear
% of 1.35) this calculation reproduces the measured table of v4 with
% differences <= 0.4 m/s^2 between 30 and 90 m/s.
%   a(v) = [ min(F_engine, F_traction) - F_aero - F_rolling ] / m
eng_scale = pwr.eng_scale;% engine torque factor: 750 kW (2026 regulation) at 12 000 rpm. MUST match
                      % the factor applied to f_tbrake in the Mapped SI Engine ("]*3.897*0.91")
Re_w   = pwr.Re;      % [m] effective wheel radius
N_diff = pwr.N_diff;  eta_tr = pwr.eta_tr;
N_box  = pwr.gears;  % gears 2..8 (= N vector of the transmission)
rpm_up = pwr.rpm_up;  % the gear is the lowest one with rpm <= rpm_up (same as f1_shift_logic)
rpm_bpt = pwr.rpm_bpt;   % [rpm]
T_full  = pwr.T_full;  % [N*m] maximum torque row of the map
CdA    = aero.CdA_straight;   % [m^2] straight mode (active aero). Must match CdA_straight in f1_aero_active.m (calibration)
Q1 = tire.QSY1;  Q3 = tire.QSY3;  Q7 = tire.QSY7;  Fz0 = tire.Fz0_roll;
                      % rolling resistance: coefficients QSY1, QSY3, QSY7 of the wheel. The QSY4 (v^4) term is
                      % not included: with it the computed table does not reproduce the measured acceleration
mu_trac = tire.mu_trac;  fr_rear = veh.weight_dist_r;   % limiter grip and rear mass fraction (rear-wheel drive)
v_tab = 0:2:100;
a_tab = zeros(size(v_tab));
for ii = 1:numel(v_tab)
    vv  = v_tab(ii);
    Lz  = m*g + 0.5*rho*ClA*vv^2;                          % total vertical load
    Nsel = N_box(end);
    for kk = 1:numel(N_box)
        if vv / Re_w * N_box(kk) * N_diff * 30/pi <= rpm_up, Nsel = N_box(kk); break, end
    end
    rpm = vv / Re_w * Nsel * N_diff * 30/pi;
    Tq  = interp1(rpm_bpt, T_full, min(rpm, rpm_bpt(end))) * eng_scale;
    Tq  = Tq * min(1, max(0, (pwr.rpm_lim_end - rpm) / pwr.rpm_lim_taper));   % rev limiter (14000-15000)
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
% The final profile meets both limits: braking (backward) and acceleration (forward)
trackVref = min(trackVref, vf);

%% Check
fprintf('Ideal lap of the profile (reachable): %.2f s\n', sum(ds ./ trackVref));
fprintf('Speed profile: min %.1f m/s (%.0f km/h) at s = %.0f m | max %.1f m/s\n', ...
    min(trackVref), min(trackVref)*3.6, trackS(find(trackVref == min(trackVref), 1)), max(trackVref));

figure('Name', 'Reference speed profile'); plot(trackS, trackVref*3.6); grid on
xlabel('Distance s [m]'); ylabel('VelRef [km/h]'); title('Reference speed profile');