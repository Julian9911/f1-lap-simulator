0%% 
%% f1_lap_table.m
% Arma la tabla de tiempos por vuelta despues de simular.
%
% Con bloques "To Workspace" (Save format: Timeseries) y "Single
% simulation output" activado, cada bloque aparece como una propiedad
% directa de "out", nombrada igual que su "Variable name" - por eso
% out.lapCount funciona directo, sin pasar por logsout.

lapCountData = out.lapCount.Data;
lastLapTimeData = out.lastLapTime.Data;

changes = find(diff(lapCountData) > 0);
lapTimes = lastLapTimeData(changes + 1);

lapTable = table((1:length(lapTimes))', lapTimes, ...
    'VariableNames', {'Vuelta', 'Tiempo_s'});

disp(lapTable);

% Vuelta mas rapida, util para comparar contra el record real (87.097 s)
[bestTime, bestLap] = min(lapTimes);
fprintf('Vuelta mas rapida: %.3f s (vuelta %d)\n', bestTime, bestLap);