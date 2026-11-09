%% f1_finish_line_params.m
% Computes the finish line geometry from the real circuit
% (silverstone_centerline.csv, already with the real finish line at index 1).
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License
% Circuit geometry: (c) OpenStreetMap contributors, ODbL.
%
% startX, startY  - finish line point
% tangentX/Y      - direction of travel at the finish line (unit vector)
% normalX/Y       - perpendicular to the direction of travel (unit vector)
%
% These 6 constants feed the MATLAB Function block that detects the
% finish line crossing.

track = readmatrix('silverstone_centerline.csv');

startX = track(1,1);
startY = track(1,2);

dirVec = track(2,:) - track(1,:);
dirVec = dirVec / norm(dirVec);

tangentX = dirVec(1);
tangentY = dirVec(2);

% Normal = tangent rotated 90 degrees
normalX = -tangentY;
normalY = tangentX;

% Initial heading of the car (psi_o for the Vehicle Body) - it must point
% in the real direction of the track at the finish line, not in a generic
% value. Without this, the car starts facing a side that does not match
% where the circuit really goes from there.
psi_o = atan2(tangentY, tangentX);

fprintf('psi_o (initial heading): %.3f rad (%.1f degrees)\n', psi_o, rad2deg(psi_o));
fprintf('Finish line at (%.1f, %.1f)\n', startX, startY);
fprintf('Direction of travel: (%.3f, %.3f)\n', tangentX, tangentY);