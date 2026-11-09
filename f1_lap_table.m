%% f1_lap_table.m
% Builds the lap-time table after simulating.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
%
% With "To Workspace" blocks (Save format: Timeseries) and "Single
% simulation output" enabled, each block appears as a direct property of
% "out", named like its "Variable name" - that is why out.lapCount works
% directly, without going through logsout.
lapCountData = out.lapCount.Data;
lastLapTimeData = out.lastLapTime.Data;
changes = find(diff(lapCountData) > 0);
lapTimes = lastLapTimeData(changes + 1);
lapTable = table((1:length(lapTimes))', lapTimes, ...
    'VariableNames', {'Lap', 'Time_s'});
disp(lapTable);
% Fastest lap, useful to compare against the real record (87.097 s)
[bestTime, bestLap] = min(lapTimes);
fprintf('Fastest lap: %.3f s (lap %d)\n', bestTime, bestLap);