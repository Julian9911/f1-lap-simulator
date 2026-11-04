%% f1_export_figures.m
% Exports the figures that are open (telemetry, sector map, ...) and screenshots of
% the model to <project root>/docs/img as PNG, for the README and the report.
%
% Use: run f1_telemetry and f1_sector_map (leave their figures open), then run this
% script. Model screenshots need the model on the path (open the project first).
% Post-processing only: it does not modify the model.

try, root = currentProject().RootFolder; catch, root = pwd; end
outDir = fullfile(root, 'docs', 'img');
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% Open figures -> PNG (the file name is built from the figure name)
figs = findall(groot, 'Type', 'figure');
nOk = 0;
for k = 1:numel(figs)
    nm = figs(k).Name;
    if isempty(nm), continue; end
    fn = regexprep(lower(nm), '[^a-z0-9]+', '_');
    fn = regexprep(fn, '^_+|_+$', '');
    exportgraphics(figs(k), fullfile(outDir, [fn '.png']), 'Resolution', 200, 'BackgroundColor', 'current');
    nOk = nOk + 1;
end

%% Model screenshots (top level and the three subsystems)
mdl = 'F1_Simulator';
shots = {mdl,                     'model_top_level';
         [mdl '/Pilot'],          'model_controller';
         [mdl '/Vehicle Dynamics'], 'model_plant';
         [mdl '/LapTimer'],       'model_instrumentation'};
try
    load_system(mdl);
    for k = 1:size(shots, 1)
        try
            print(['-s' shots{k,1}], '-dpng', '-r200', fullfile(outDir, [shots{k,2} '.png']));
            nOk = nOk + 1;
        catch ME
            warning('f1_export_figures:shot', 'Could not capture %s: %s', shots{k,1}, ME.message);
        end
    end
catch ME
    warning('f1_export_figures:model', 'Model screenshots skipped: %s', ME.message);
end
fprintf('f1_export_figures: %d image(s) written to %s\n', nOk, outDir);
