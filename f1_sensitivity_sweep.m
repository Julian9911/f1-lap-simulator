function results = f1_sensitivity_sweep(true)
%F1_SENSITIVITY_SWEEP  Lap-time sensitivity of the full Simulink model.
%
%   f1_sensitivity_sweep(true)    quick test: only the base case (1 simulation)
%   f1_sensitivity_sweep          full sweep (base + 5 cases, +5 % each)
%   results = f1_sensitivity_sweep;   table with the lap time and the change vs base
%
% For each case it changes one group of base values (through the hooks
% sweepA / sweepB of f1_vehicle_params.m), re-runs f1_vehicle_params (profile,
% tables and fp_* aliases included), simulates 260 s like the Run button and
% reads the lap time like f1_regression_test (fastest lap that is not the
% standing start). At the end it restores the normal workspace.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% Output: console table, f1_sensitivity_result.csv and f1_sensitivity_result.png.
% It closes all open figures (f1_vehicle_params opens some on every run).
% It does NOT modify or save the model.

if nargin < 1, quick = false; end
mdl = 'F1_Simulator';
load_system(mdl);

% {name, code for hook A (veh/aero/tire), code for hook B (pwr/brk)}
C = {
 'Base',               '',                                                                                                                     ''
 'Grip mu +5 %',       'tire.mu_pk_f=tire.mu_pk_f*1.05; tire.mu_pk_r=tire.mu_pk_r*1.05; tire.mu_trac=tire.mu_trac*1.05; tire.mu_brake=tire.mu_brake*1.05;', ''
 'Downforce ClA +5 %', 'aero.Cl=aero.Cl*1.05;',                                                                                                 ''
 'Mass +5 %',          'veh.m=veh.m*1.05;',                                                                                                      ''
 'Engine torque +5 %', '',                                                                                                                       'pwr.eng_scale=pwr.eng_scale*1.05;'
 'Drag CdA +5 %',      'aero.CdA_straight=aero.CdA_straight*1.05;',                                                                              ''
};
if quick, C = C(1, :); end
n = size(C, 1);

restore = onCleanup(@() evalin('base', 'clear sweepA sweepB; f1_vehicle_params; close all'));

lap = nan(n, 1);  secs = nan(n, 1);
for k = 1:n
    fprintf('\n##### Case %d/%d: %s #####\n', k, n, C{k,1});
    assignin('base', 'sweepA', C{k,2});
    assignin('base', 'sweepB', C{k,3});
    evalin('base', 'f1_vehicle_params');
    close all
    t0 = tic;
    out = sim(mdl, 'StopTime', '260');
    secs(k) = toc(t0);
    lcData = out.lapCount.Data(:);  lcTime = out.lapCount.Time(:);
    tCross = lcTime(find(diff(lcData) > 0) + 1);
    lapDur = diff([0; tCross]);
    if numel(lapDur) >= 2
        lap(k) = min(lapDur(2:end));
    else
        fprintf('  WARNING: fewer than 2 complete laps, no valid lap time.\n');
    end
    fprintf('  lap time: %.2f s  (simulation took %.0f s)\n', lap(k), secs(k));
end

delta = lap - lap(1);
results = table(C(:,1), lap, delta, secs, 'VariableNames', {'Case', 'LapTime_s', 'DeltaVsBase_s', 'SimSeconds'});
fprintf('\n=== Lap-time sensitivity (negative = faster) ===\n');
disp(results);
writetable(results, 'f1_sensitivity_result.csv');

if n > 1
    fig = figure('Color', 'w', 'Position', [100 100 760 380]);
    barh(delta(2:end));
    set(gca, 'YTick', 1:n-1, 'YTickLabel', C(2:end, 1), 'YDir', 'reverse');
    xlabel('Lap time change vs base [s]  (negative = faster)');
    title(sprintf('Lap-time sensitivity of the full model (base %.2f s)', lap(1)));
    grid on;
    exportgraphics(fig, 'f1_sensitivity_result.png', 'Resolution', 200);
end
fprintf('\nSaved f1_sensitivity_result.csv and .png. Workspace restored.\n');
end
