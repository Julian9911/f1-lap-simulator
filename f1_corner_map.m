function f1_corner_map(lapToShow)
%F1_CORNER_MAP  Interactive aerial map of the circuit with the line the model drove.
%
%   f1_corner_map            fastest complete lap that is not the standing start
%   f1_corner_map(n)         lap number n
%
% The simulated trajectory is colored by what the driver does (brake, throttle,
% lift / coast). Every corner has a numbered marker: click it (or choose it in
% the drop-down) to zoom in. The numbers of the corner are shown over the map and
% the right-hand panel plots speed, pedals and accelerations around it, simulator
% against the real reference lap. The slider (or the arrow buttons) moves a point
% along the circuit and a cursor follows it in the three plots.
% Zoom with the mouse wheel, pan by dragging, legend items hide / show a signal.
%
% Post-processing only: it does not touch the model.
%
% Requirements: simulate first, so that the base workspace has "out"
% (carX_log, carY_log, xdot_log, lapCount; acc_log and dec_log if present).
% In the current folder or on the path: silverstone_centerline.csv (track edges)
% and, optionally, silverstone_real_telemetry.csv (reference lap).
%
% Track width is NOT measured: OpenStreetMap has none. The edges are drawn at
% +/- trackHalfWidth from the centerline and are only a visual aid.
%
% Author: Julian Orozco - https://github.com/Julian9911 - MIT License

if nargin < 1, lapToShow = 0; end

%% Settings
trackHalfWidth = 8;       % [m] drawn half width of the track (illustrative)
minProminence  = 14;      % [km/h] a speed minimum must be this deep to count as a corner
minSeparation  = 25;      % [samples of 2 m] minimum distance between two corners
brakeThr       = 0.05;    % normalized brake command above which the car is braking
throttleThr    = 0.5;     % normalized throttle command above which the car is on throttle
radiusReal     = 40;      % [m] real-lap samples this close to the apex are compared

bg    = [0.047 0.051 0.067];   panelBg = [0.082 0.090 0.114];   edgeC = [0.17 0.18 0.22];
ink   = [0.93 0.94 0.96];      mute    = [0.56 0.59 0.65];      gold  = [1.00 0.82 0.25];
cBrk  = [1.00 0.30 0.24];      cCoast  = [0.56 0.60 0.69];      cThr  = [0.21 0.82 0.42];
g0 = 9.81;  ds = 2;

%% Simulated lap
if ~evalin('base', 'exist(''out'', ''var'')'), error('There is no "out" in the workspace: simulate first.'); end
out = evalin('base', 'out');
lcData = out.lapCount.Data(:);  lcTime = out.lapCount.Time(:);
tCross = lcTime(find(diff(lcData) > 0) + 1);
tBounds = [0; tCross];  lapDur = diff(tBounds);
if lapToShow == 0
    valid = 2:numel(lapDur);  if isempty(valid), valid = 1; end
    [~, ib] = min(lapDur(valid));  lapToShow = valid(ib);
end
t0 = tBounds(lapToShow);  t1 = tBounds(lapToShow + 1);

tg = (t0:0.02:t1)';
xg = interp1(out.carX_log.Time(:),  out.carX_log.Data(:),  tg);
yg = interp1(out.carY_log.Time(:),  out.carY_log.Data(:),  tg);
vg = interp1(out.xdot_log.Time(:),  out.xdot_log.Data(:),  tg);
sg = cumtrapz(tg, vg);
havePedals = true;
try
    accg = interp1(out.acc_log.Time(:), out.acc_log.Data(:), tg);
    decg = interp1(out.dec_log.Time(:), out.dec_log.Data(:), tg);
catch
    havePedals = false;  accg = zeros(size(tg));  decg = zeros(size(tg));
end
[sg, iu] = unique(sg);
sU = (0:ds:sg(end))';
X   = interp1(sg, xg(iu),   sU);   Y   = interp1(sg, yg(iu),   sU);
Vms = interp1(sg, vg(iu),   sU);   ACC = interp1(sg, accg(iu), sU);   DEC = interp1(sg, decg(iu), sU);
n = numel(sU);
Xs = movmean(X, 5);  Ys = movmean(Y, 5);
psi = unwrap(atan2(gradient(Ys), gradient(Xs)));
kap = movmean(gradient(psi, ds), 5);                 % signed curvature [1/m], + = left
ayG = Vms.^2 .* abs(kap) / g0;
axG = movmean(gradient(Vms.^2 / 2, ds), 5) / g0;
Vk  = Vms * 3.6;

%% What the car is doing: brake / throttle / lift
if havePedals
    brake = (DEC / max(abs(DEC) + eps)) > brakeThr;
    thr   = (ACC / max(abs(ACC) + eps)) > throttleThr & ~brake;
else
    brake = axG < -0.5;  thr = axG > 0.3 & ~brake;
end
coast = ~brake & ~thr;

%% Corners: deep speed minima (the lap is treated as circular, so a corner at the finish line is found)
idc = @(k) mod(k - 1, n) + 1;                       % circular index
Vsm = movmean(Vk, 9);
P   = min(120, n - 1);
Vpad = [Vsm(end-P+1:end); Vsm; Vsm(1:P)];
icp = find(islocalmin(Vpad, 'MinProminence', minProminence, 'MinSeparation', minSeparation));
ic  = unique(idc(icp - P));
keep = true(size(ic));
for q = 2:numel(ic)
    if ic(q) - ic(q-1) < minSeparation, keep(q) = false; end
end
if numel(ic) > 1 && n - ic(end) + ic(1) < minSeparation, keep(end) = false; end
ic = ic(keep);
nC = numel(ic);

%% Reference (calculated) line, for the lateral error
hasRef = evalin('base', 'exist(''trackX'', ''var'') && exist(''trackY'', ''var'')');
if hasRef
    refX = evalin('base', 'trackX(:)');  refY = evalin('base', 'trackY(:)');
end

%% Real reference lap (optional), placed along the lap by position (S_aligned)
hasReal = false;  dRs = [];  RvS = [];  RthS = [];  RbrS = [];
try
    R = readtable('silverstone_real_telemetry.csv');
    if exist('totalLength', 'var'), Lref = totalLength; else, Lref = max(sU) + 2; end
    if ismember('S_aligned', R.Properties.VariableNames)
        fracR = mod(R.S_aligned(:), Lref) / Lref;
    else
        fracR = R.Distance_m(:) / R.Distance_m(end);
    end
    [dRs, ir] = sort(fracR * sU(end));
    RvS = R.Speed_kmh(ir);  RthS = R.Throttle(ir);  RbrS = R.Brake(ir);
    hasReal = true;
catch
end

if hasReal
    [dRu, iu2] = unique(dRs);  RvU = RvS(iu2);
else
    dRu = [];  RvU = [];
end

%% Statistics of each corner
S = struct('n', {}, 'i', {}, 'dist', {}, 'dirn', {}, 'vmin', {}, 'vin', {}, 'vout', {}, ...
           'brkDist', {}, 'brkG', {}, 'latG', {}, 'radius', {}, 'latErr', {}, 'vReal', {});
for q = 1:nC
    i = ic(q);
    k = i;
    while ~brake(idc(k)) && i - k < 30, k = k - 1; end
    if brake(idc(k))
        kEnd = k;
        while brake(idc(k - 1)) && kEnd - k < 200, k = k - 1; end
        kStart = k;
    else
        kStart = i;  kEnd = i;
    end
    w = idc(i-8:i+8);
    S(q).n = q;  S(q).i = i;  S(q).dist = sU(i);
    if kap(i) >= 0, S(q).dirn = 'left'; else, S(q).dirn = 'right'; end
    S(q).vmin = Vk(i);
    S(q).vin  = Vk(idc(kStart));
    S(q).vout = Vk(idc(i + 25));
    S(q).brkDist = (kEnd - kStart) * ds;
    if kEnd > kStart, S(q).brkG = -min(axG(idc(kStart:kEnd))); else, S(q).brkG = 0; end
    S(q).latG   = max(ayG(w));
    S(q).radius = 1 / max(abs(kap(w)));
    S(q).latErr = NaN;
    if hasRef
        ww = idc(i-30:i+30);
        e = zeros(numel(ww), 1);
        for m = 1:numel(ww), e(m) = sqrt(min((refX - X(ww(m))).^2 + (refY - Y(ww(m))).^2)); end
        S(q).latErr = max(e);
    end
    S(q).vReal = NaN;
    if hasReal
        dd0 = abs(dRs - sU(i));  dd0 = min(dd0, sU(end) - dd0);        % circular distance [m]
        near = dd0 < radiusReal;
        if any(near), S(q).vReal = min(RvS(near)); end
    end
end

%% Console check: every corner with its numbers
fprintf('Lap %d (%.2f s): %d corners\n', lapToShow, t1 - t0, nC);
fprintf(' #   dist[m]  dir    vmin   vin   vout  brk[m]  brk[g]  lat[g]  R[m]  err[m]  real[km/h]\n');
for q = 1:nC
    fprintf('%2d  %7.0f  %-5s %5.0f %5.0f %5.0f %7.0f %7.1f %7.1f %5.0f %7.1f %8.0f\n', q, S(q).dist, S(q).dirn, ...
        S(q).vmin, S(q).vin, S(q).vout, S(q).brkDist, S(q).brkG, S(q).latG, S(q).radius, S(q).latErr, S(q).vReal);
end
if any(isnan([S.latErr])), fprintf('Note: some lateral errors are NaN (trackX / trackY not in the workspace).\n'); end
if any(isnan([S.vReal])), fprintf('Note: some corners have no real reference (file missing or lap position not aligned).\n'); end

%% Track edges from the centerline
C = readmatrix('silverstone_centerline.csv');
C = C([true; hypot(diff(C(:,1)), diff(C(:,2))) > 1e-6], :);
if norm(C(1,:) - C(end,:)) < 1e-6, C(end,:) = []; end
C = [C; C(1,:)];
sC = [0; cumsum(hypot(diff(C(:,1)), diff(C(:,2))))];
Cr = interp1(sC, C, (0:ds:sC(end))');  Cr(end,:) = [];
pad = 9;
Cp = movmean([Cr(end-pad+1:end,:); Cr; Cr(1:pad,:)], 9);
Cs = Cp(pad+1:end-pad, :);
tn = circshift(Cs, -1) - circshift(Cs, 1);
tn = tn ./ hypot(tn(:,1), tn(:,2));
nm = [-tn(:,2), tn(:,1)];
EL = Cs + trackHalfWidth * nm;   ER = Cs - trackHalfWidth * nm;
EL = [EL; EL(1,:)];  ER = [ER; ER(1,:)];

%% Pedals normalized to percent (simulator)
if havePedals
    thrP = max(ACC, 0) / max(abs(ACC) + eps) * 100;
    brkP = max(DEC, 0) / max(abs(DEC) + eps) * 100;
else
    thrP = nan(n, 1);  brkP = nan(n, 1);
end

% HTML of the info card (rendered by uihtml; MATLAB sends the numbers through .Data)
htmlSrc = strjoin({ ...
    '<!DOCTYPE html>', ...
    '<html><head><meta charset="utf-8">', ...
    '<style>', ...
    'html,body{margin:0;height:100%;background:#15171d;font-family:"Segoe UI",system-ui,-apple-system,Roboto,Arial,sans-serif;color:#eceef3;overflow:hidden}', ...
    '.card{box-sizing:border-box;height:100%;padding:14px 16px 12px;border:1px solid #2b2e38;border-left:4px solid #ffd23f;display:flex;flex-direction:column;gap:10px}', ...
    '.head{display:flex;align-items:center;justify-content:space-between}', ...
    '.title{font-size:20px;font-weight:700;letter-spacing:.05em;text-transform:uppercase}', ...
    '.chip{font-size:11px;color:#aab0be;text-transform:uppercase;letter-spacing:.08em;border:1px solid #343846;padding:3px 9px;border-radius:11px;background:#0f1116}', ...
    '.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}', ...
    '.tile{background:#0f1116;border:1px solid #23262f;border-radius:6px;padding:8px 10px}', ...
    '.cap{font-size:10px;color:#8f96a6;text-transform:uppercase;letter-spacing:.09em}', ...
    '.val{font-size:23px;font-weight:700;line-height:1.2;font-variant-numeric:tabular-nums}', ...
    '.val small{font-size:12px;font-weight:500;color:#8f96a6;margin-left:3px}', ...
    '.sub{font-size:11px;color:#8f96a6;white-space:nowrap}', ...
    '.good{color:#35d06a}.bad{color:#ff6a5a}', ...
    '.head{cursor:move;user-select:none;gap:6px}', ...
    '.title,.chip{white-space:nowrap}', ...
    '.chip{font-size:10px;letter-spacing:.06em;padding:3px 8px}', ...
    '.btn{cursor:pointer;width:24px;height:24px;border:1px solid #343846;border-radius:12px;background:#0f1116;color:#eceef3;text-align:center;line-height:22px;font-size:15px;margin-left:8px}', ...
    '.btn:hover{border-color:#ffd23f}', ...
    '.sum{display:flex;flex-direction:column;gap:5px;font-size:12px;color:#8f96a6;border-top:1px solid #2b2e38;padding-top:8px}', ...
    '.sum div{display:flex;justify-content:space-between}', ...
    '.sum b{font-weight:600;color:#eceef3;font-variant-numeric:tabular-nums}', ...
    '.now{margin-top:auto;border-top:1px solid #2b2e38;padding-top:8px;display:flex;justify-content:space-between;align-items:baseline;font-size:12px;color:#8f96a6}', ...
    '.now b{font-weight:600;color:#eceef3;font-variant-numeric:tabular-nums}', ...
    '</style></head>', ...
    '<body><div class="card" id="c"></div>', ...
    '<script>', ...
    'function render(d){', ...
    '  if(!d){return;}', ...
    '  var h="<div class=\"head\"><div class=\"title\">"+d.title+"</div><div style=\"display:flex;align-items:center\"><div class=\"chip\">"+d.chip+"</div><div class=\"btn\" title=\"Minimize / restore\">"+(d.min?"+":"&#8211;")+"</div></div></div>";', ...
    '  if(d.min){document.getElementById("c").innerHTML=h;return;}', ...
    '  h+="<div class=\"grid\">";', ...
    '  for(var i=0;i<d.t.length;i++){', ...
    '    var t=d.t[i];', ...
    '    h+="<div class=\"tile\"><div class=\"cap\">"+t.cap+"</div><div class=\"val\">"+t.val+"<small>"+t.unit+"</small></div><div class=\"sub "+t.cls+"\">"+t.sub+"</div></div>";', ...
    '  }', ...
    '  h+="</div><div class=\"sum\">";', ...
    '  for(var j=0;j<d.sum.length;j++){h+="<div><span>"+d.sum[j].k+"</span><b>"+d.sum[j].v+"</b></div>";}', ...
    '  h+="</div><div class=\"now\"><span>"+d.now+"</span><b>"+d.nowVal+"</b></div>";', ...
    '  document.getElementById("c").innerHTML=h;', ...
    '}', ...
    'var HC=null,dragging=false,lx=0,ly=0,px=0,py=0,lastT=0;', ...
    'document.addEventListener("click",function(e){if(HC&&e.target.closest(".btn")){HC.sendEventToMATLAB("toggle",{});}});', ...
    'document.addEventListener("mousedown",function(e){if(e.target.closest(".head")&&!e.target.closest(".btn")){dragging=true;lx=e.screenX;ly=e.screenY;px=0;py=0;e.preventDefault();}});', ...
    'document.addEventListener("mousemove",function(e){if(!dragging||!HC){return;}px+=e.screenX-lx;py+=e.screenY-ly;lx=e.screenX;ly=e.screenY;var t=Date.now();if(t-lastT>25){HC.sendEventToMATLAB("move",{dx:px,dy:py});px=0;py=0;lastT=t;}});', ...
    'document.addEventListener("mouseup",function(){if(dragging&&HC&&(px||py)){HC.sendEventToMATLAB("move",{dx:px,dy:py});}dragging=false;px=0;py=0;});', ...
    'function setup(htmlComponent){', ...
    '  HC=htmlComponent;', ...
    '  htmlComponent.addEventListener("DataChanged",function(){render(htmlComponent.Data);});', ...
    '  render(htmlComponent.Data);', ...
    '}', ...
    '</script></body></html>' ...
    }, newline);

%% Figure and controls
rotated  = true;                 % landscape by default; false = north up
selected = 0;
cursorD  = 0;
cSim = [1.00 0.55 0.25];  cReal = [0.85 0.87 0.92];
fig = uifigure('Name', 'F1 corner map', 'Color', bg, 'Position', [40 40 1560 860]);
gl  = uigridlayout(fig, [1 2]);
gl.ColumnWidth = {'1x', 470};  gl.BackgroundColor = bg;  gl.Padding = [0 0 0 0];  gl.ColumnSpacing = 0;

% Left: map and slider
lgl = uigridlayout(gl, [2 1]);  lgl.Layout.Column = 1;
lgl.RowHeight = {'1x', 52};  lgl.BackgroundColor = bg;  lgl.Padding = [0 0 0 0];  lgl.RowSpacing = 0;
ax  = uiaxes(lgl);  ax.Layout.Row = 1;
ax.Color = bg;  ax.XColor = 'none';  ax.YColor = 'none';  ax.Box = 'off';
ax.Interactions = [zoomInteraction, panInteraction];
hold(ax, 'on');
cgl = uigridlayout(lgl, [1 4]);  cgl.Layout.Row = 2;
cgl.ColumnWidth = {46, '1x', 46, 250};  cgl.BackgroundColor = panelBg;  cgl.Padding = [10 6 10 6];
bBack = uibutton(cgl, 'Text', char(9664), 'ButtonPushedFcn', @(~, ~) stepPos(-10));
bFwd  = uibutton(cgl, 'Text', char(9654), 'ButtonPushedFcn', @(~, ~) stepPos(10));
sl = uislider(cgl, 'Limits', [0 sU(end)], 'Value', 0, 'MajorTicks', [], 'MinorTicks', [], ...
    'ValueChangingFcn', @(~, e) setPos(e.Value), 'ValueChangedFcn', @(s, ~) setPos(s.Value));
bBack.Layout.Column = 1;  sl.Layout.Column = 2;  bFwd.Layout.Column = 3;
lblPos = uilabel(cgl, 'Text', '', 'FontSize', 13, 'FontColor', ink, 'HorizontalAlignment', 'right');
lblPos.Layout.Column = 4;
set([bBack bFwd], 'BackgroundColor', bg, 'FontColor', ink);

% Right: title, corner selector and three plots
pn = uipanel(gl);  pn.Layout.Column = 2;  pn.BackgroundColor = panelBg;  pn.BorderType = 'none';
pg = uigridlayout(pn, [5 1]);
pg.BackgroundColor = panelBg;  pg.Padding = [14 12 14 12];  pg.RowSpacing = 6;
pg.RowHeight = {40, 28, '1x', '1x', '1x'};
lblTitle = uilabel(pg, 'Text', '', 'FontSize', 28, 'FontWeight', 'bold', 'FontColor', ink);
dd = uidropdown(pg, 'Items', [{'Overview'}, arrayfun(@(q) sprintf('Corner %d', q), 1:nC, 'UniformOutput', false)], ...
    'ItemsData', 0:nC, 'Value', 0, 'ValueChangedFcn', @(s, ~) selectCorner(s.Value));
ax1 = uiaxes(pg);  ax2 = uiaxes(pg);  ax3 = uiaxes(pg);
for a = [ax1 ax2 ax3]
    set(a, 'Color', bg, 'XColor', mute, 'YColor', mute, 'XGrid', 'on', 'YGrid', 'on', ...
        'GridColor', [0.35 0.36 0.42], 'GridAlpha', 0.35, 'FontSize', 10, 'Box', 'off');
end
hCur = gobjects(1, 3);

% Buttons over the map
bx = uibutton(fig, 'Text', 'Fit', 'Position', [14 820 64 26], 'ButtonPushedFcn', @(~, ~) selectCorner(0));
br = uibutton(fig, 'Text', 'Rotate', 'Position', [84 820 64 26], 'ButtonPushedFcn', @(~, ~) rotateMap());
be = uibutton(fig, 'Text', 'Export PNG', 'Position', [154 820 90 26], 'ButtonPushedFcn', @(~, ~) exportPng());
set([bx br be], 'BackgroundColor', panelBg, 'FontColor', ink);

cardW = 340;  cardH = 372;  cardMinH = 62;  panelW = 470;
minimized = false;  userMoved = false;
card = uihtml(fig, 'HTMLSource', htmlSrc, 'Position', [0 0 cardW cardH], 'HTMLEventReceivedFcn', @(~, e) cardEvent(e));
fig.AutoResizeChildren = 'off';                 % required for SizeChangedFcn; the grid layout still fills the window
fig.SizeChangedFcn = @(~, ~) placeCard();
placeCard();

hMk = gobjects(1, nC);  hTx = gobjects(1, nC);  hLine = gobjects(1, 3);  hDot = [];
drawMap();
selectCorner(0);

    %% ---- nested functions -------------------------------------------------
    function [u, v] = T(x, y)
        if rotated, u = y;  v = -x;  else, u = x;  v = y; end
    end

    function drawMap()
        delete(allchild(ax));  hold(ax, 'on');       % allchild also removes objects with HandleVisibility off
        [ul, vl] = T(EL(:,1), EL(:,2));  [ur, vr] = T(ER(:,1), ER(:,2));
        patch(ax, [ul; flipud(ur)], [vl; flipud(vr)], [0.14 0.145 0.18], 'EdgeColor', 'none', 'HitTest', 'off');
        plot(ax, ul, vl, '-', 'Color', [0.90 0.91 0.94], 'LineWidth', 0.8, 'HitTest', 'off');
        plot(ax, ur, vr, '-', 'Color', [0.90 0.91 0.94], 'LineWidth', 0.8, 'HitTest', 'off');
        [u, v] = T(X, Y);
        masks = {brake, thr, coast};  cols = {cBrk, cThr, cCoast};
        names = {'braking', 'throttle', 'lift / coast'};
        for m = 1:3
            mk = masks{m};  mk = mk | [false; mk(1:end-1)];
            uu = u;  vv = v;  uu(~mk) = NaN;  vv(~mk) = NaN;
            hLine(m) = plot(ax, uu, vv, '-', 'Color', cols{m}, 'LineWidth', 2.6, 'DisplayName', names{m}, 'HitTest', 'off');
        end
        [us, vs] = T(X(1), Y(1));
        plot(ax, us, vs, 's', 'MarkerSize', 8, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'w', 'HitTest', 'off', 'HandleVisibility', 'off');
        text(ax, us, vs, '   START / FINISH', 'Color', 'w', 'FontWeight', 'bold', 'FontSize', 10, 'HitTest', 'off', 'HandleVisibility', 'off');
        for q = 1:nC
            [uq, vq] = T(X(S(q).i), Y(S(q).i));
            hMk(q) = plot(ax, uq, vq, 'o', 'MarkerSize', 19, 'MarkerFaceColor', bg, 'MarkerEdgeColor', gold, ...
                'LineWidth', 2, 'ButtonDownFcn', @(~, ~) selectCorner(q), 'HandleVisibility', 'off');
            hTx(q) = text(ax, uq, vq, num2str(q), 'Color', ink, 'FontWeight', 'bold', 'FontSize', 11, ...
                'HorizontalAlignment', 'center', 'HitTest', 'off', 'PickableParts', 'none', 'HandleVisibility', 'off');
        end
        hDot = plot(ax, NaN, NaN, 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', gold, 'LineWidth', 2, ...
            'HitTest', 'off', 'HandleVisibility', 'off');
        lg = legend(ax, hLine, 'Location', 'southwest', 'TextColor', ink, 'Color', panelBg, 'EdgeColor', edgeC);
        lg.ItemHitFcn = @toggleLine;
        axis(ax, 'equal');
        fitView();
        paintSelection();
        setPos(cursorD);
    end

    function toggleLine(~, evt)
        if evt.Peer.Visible == "on", evt.Peer.Visible = 'off'; else, evt.Peer.Visible = 'on'; end
    end

    function fitView()
        [ua, va] = T([EL(:,1); ER(:,1)], [EL(:,2); ER(:,2)]);
        mg = 60;
        xlim(ax, [min(ua) - mg, max(ua) + mg]);  ylim(ax, [min(va) - mg, max(va) + mg]);
    end

    function rotateMap()
        rotated = ~rotated;  drawMap();
        if selected > 0, zoomToCorner(selected); end
    end

    function paintSelection()
        for q = 1:nC
            if q == selected
                hMk(q).MarkerFaceColor = gold;  hTx(q).Color = [0.07 0.07 0.07];
            else
                hMk(q).MarkerFaceColor = bg;    hTx(q).Color = ink;
            end
        end
    end

    function zoomToCorner(q)
        [uq, vq] = T(X(S(q).i), Y(S(q).i));
        xlim(ax, uq + [-170 170]);  ylim(ax, vq + [-120 120]);
    end

    function selectCorner(q)
        selected = q;  dd.Value = q;  paintSelection();
        if q == 0
            d0 = 0;  d1 = sU(end);  cursorD = 0;
            lblTitle.Text = 'Overview';
            fitView();
        else
            d0 = max(0, S(q).dist - 150);  d1 = min(sU(end), S(q).dist + 150);  cursorD = S(q).dist;
            lblTitle.Text = sprintf('Corner %d', q);
            zoomToCorner(q);
        end
        sl.Limits = [min(sl.Limits(1), d0), max(sl.Limits(2), d1)];   % widen first so Value is never outside
        sl.Value = cursorD;  sl.Limits = [d0 d1];
        updateGraphs(d0, d1);
        setPos(cursorD);
    end

    function updateGraphs(d0, d1)
        m  = sU >= d0 & sU <= d1;
        mr = hasReal & dRs >= d0 & dRs <= d1;
        % 1. speed
        cla(ax1);  hold(ax1, 'on');
        plot(ax1, sU(m), Vk(m), '-', 'Color', cSim, 'LineWidth', 1.8, 'DisplayName', 'simulator');
        if hasReal, plot(ax1, dRs(mr), RvS(mr), '-', 'Color', cReal, 'LineWidth', 1.4, 'DisplayName', 'real'); end
        hCur(1) = xline(ax1, cursorD, '-', 'Color', gold, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xlim(ax1, [d0 d1]);  ylabel(ax1, 'km/h', 'Color', mute);  title(ax1, 'Speed', 'Color', ink, 'FontSize', 11);
        lg1 = legend(ax1, 'Location', 'best', 'TextColor', ink, 'Color', panelBg, 'EdgeColor', edgeC, 'FontSize', 9);  lg1.ItemHitFcn = @toggleLine;
        % 2. pedals
        cla(ax2);  hold(ax2, 'on');
        plot(ax2, sU(m), thrP(m), '-', 'Color', cThr, 'LineWidth', 1.6, 'DisplayName', 'throttle sim');
        plot(ax2, sU(m), brkP(m), '-', 'Color', cBrk, 'LineWidth', 1.6, 'DisplayName', 'brake sim');
        if hasReal
            plot(ax2, dRs(mr), RthS(mr), '--', 'Color', cThr * 0.75 + 0.25, 'LineWidth', 1.0, 'DisplayName', 'throttle real');
            plot(ax2, dRs(mr), RbrS(mr), '--', 'Color', cBrk * 0.75 + 0.25, 'LineWidth', 1.0, 'DisplayName', 'brake real');
        end
        hCur(2) = xline(ax2, cursorD, '-', 'Color', gold, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xlim(ax2, [d0 d1]);  ylim(ax2, [-5 105]);  ylabel(ax2, '%', 'Color', mute);  title(ax2, 'Throttle and brake', 'Color', ink, 'FontSize', 11);
        lg2 = legend(ax2, 'Location', 'best', 'TextColor', ink, 'Color', panelBg, 'EdgeColor', edgeC, 'FontSize', 9);  lg2.ItemHitFcn = @toggleLine;
        % 3. accelerations (simulator, from the trajectory)
        cla(ax3);  hold(ax3, 'on');
        plot(ax3, sU(m), axG(m), '-', 'Color', cSim, 'LineWidth', 1.6, 'DisplayName', 'longitudinal');
        plot(ax3, sU(m), ayG(m), '-', 'Color', [0.45 0.75 1.0], 'LineWidth', 1.6, 'DisplayName', 'lateral (abs)');
        hCur(3) = xline(ax3, cursorD, '-', 'Color', gold, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xlim(ax3, [d0 d1]);  ylabel(ax3, 'g', 'Color', mute);  xlabel(ax3, 'Lap distance [m]', 'Color', mute);
        title(ax3, 'Accelerations (simulator)', 'Color', ink, 'FontSize', 11);
        lg3 = legend(ax3, 'Location', 'best', 'TextColor', ink, 'Color', panelBg, 'EdgeColor', edgeC, 'FontSize', 9);  lg3.ItemHitFcn = @toggleLine;
    end

    function stepPos(dd_)
        lim = sl.Limits;
        d = min(max(cursorD + dd_, lim(1)), lim(2));
        sl.Value = d;  setPos(d);
    end

    function setPos(d)
        cursorD = d;
        xq = interp1(sU, X, d, 'linear', 'extrap');  yq = interp1(sU, Y, d, 'linear', 'extrap');
        [uq, vq] = T(xq, yq);
        if ~isempty(hDot) && isvalid(hDot), hDot.XData = uq;  hDot.YData = vq; end
        for kk = 1:3
            if isgraphics(hCur(kk)), hCur(kk).Value = d; end
        end
        vNow = interp1(sU, Vk, d);  ayNow = interp1(sU, ayG, d);  axNow = interp1(sU, axG, d);
        lblPos.Text = sprintf('%.0f m   %.0f km/h   %.1f g lat', d, vNow, ayNow);
        card.Data = cardData(d);
    end

    function placeCard()
        p = fig.Position;
        if minimized, hh = cardMinH; else, hh = cardH; end
        if userMoved
            q = card.Position;                       % keep the user's top-left corner, only clamp inside the window
            top = min(max(q(2) + q(4), hh), p(4));
            x = min(max(q(1), 0), p(3) - cardW);
            card.Position = [x, top - hh, cardW, hh];
        else
            card.Position = [p(3) - panelW - cardW - 16, p(4) - hh - 46, cardW, hh];
        end
    end

    function cardEvent(e)
        switch e.HTMLEventName
            case 'move'
                q = card.Position;  p = fig.Position;
                x = min(max(q(1) + e.HTMLEventData.dx, 0), p(3) - cardW);
                y = min(max(q(2) - e.HTMLEventData.dy, 0), p(4) - q(4));
                card.Position = [x, y, q(3), q(4)];
                userMoved = true;
            case 'toggle'
                q = card.Position;  top = q(2) + q(4);
                minimized = ~minimized;
                if minimized, hh = cardMinH; else, hh = cardH; end
                card.Position = [q(1), top - hh, cardW, hh];
                userMoved = true;
                card.Data = cardData(cursorD);
        end
    end

    function D = cardData(d)
        tile = @(cap, val, unit, sub, cls) struct('cap', cap, 'val', val, 'unit', unit, 'sub', sub, 'cls', cls);
        row  = @(k, v) struct('k', k, 'v', v);
        % live values at the cursor
        vNow = interp1(sU, Vk, d);  ayNow = interp1(sU, ayG, d);  axNow = interp1(sU, axG, d);
        thNow = interp1(sU, thrP, d);  brNow = interp1(sU, brkP, d);
        if hasReal
            vr = interp1(dRu, RvU, d, 'linear', NaN);
        else
            vr = NaN;
        end
        if isnan(vr)
            sub1 = 'no real reference';  cl1 = '';
        else
            if vNow >= vr, cl1 = 'good'; else, cl1 = 'bad'; end
            sub1 = sprintf('real %.0f km/h, %+.0f', vr, vNow - vr);
        end
        if axNow < -0.3, mode = 'braking'; elseif axNow > 0.3, mode = 'accelerating'; else, mode = 'steady'; end
        if brNow > 5
            pv = sprintf('%.0f', brNow);  pu = '% brake';
        elseif isnan(thNow)
            pv = '-';  pu = '';
        else
            pv = sprintf('%.0f', thNow);  pu = '% throttle';
        end
        if selected == 0, latPeak = max(ayG); else, latPeak = S(selected).latG; end
        D.min = minimized;
        D.t = [tile('Speed', sprintf('%.0f', vNow), 'km/h', sub1, cl1), ...
               tile('Lateral', sprintf('%.1f', ayNow), 'g', sprintf('peak %.1f g', latPeak), ''), ...
               tile('Longitudinal', sprintf('%+.1f', axNow), 'g', mode, ''), ...
               tile('Pedal', pv, pu, sprintf('throttle %.0f%% | brake %.0f%%', thNow, brNow), '')];
        % fixed summary (does not move with the cursor)
        if selected == 0
            [vm, im] = min([S.vmin]);  [lm, il] = max([S.latG]);
            D.title = sprintf('Lap %d', lapToShow);
            D.chip  = sprintf('%d corners', nC);
            D.sum = [row('Lap time', sprintf('%.2f s', t1 - t0)), ...
                     row('Top speed', sprintf('%.0f km/h', max(Vk))), ...
                     row('Slowest corner', sprintf('%d, %.0f km/h', im, vm)), ...
                     row('Highest lateral', sprintf('%d, %.1f g', il, lm))];
            D.now = sprintf('At %.0f m', d);  D.nowVal = 'move the slider or pick a corner';
        else
            c = S(selected);
            D.title = sprintf('Corner %d', selected);
            D.chip  = sprintf('%s-hander, %.0f m', c.dirn, c.dist);
            if isnan(c.vReal), mv = sprintf('%.0f km/h', c.vmin); else, mv = sprintf('%.0f km/h (real %.0f)', c.vmin, c.vReal); end
            if c.brkDist > 0, bs = sprintf('%.0f m from %.0f km/h, %.1f g', c.brkDist, c.vin, c.brkG); else, bs = sprintf('lift only, entry %.0f km/h', c.vin); end
            if isnan(c.latErr), es = 'n/a'; else, es = sprintf('%.1f m max', c.latErr); end
            D.sum = [row('Corner min speed', mv), ...
                     row('Braking zone', bs), ...
                     row('Radius', sprintf('%.0f m', c.radius)), ...
                     row('50 m after apex', sprintf('%.0f km/h', c.vout)), ...
                     row('Distance to the line', es)];
            D.now = sprintf('At %.0f m', d);  D.nowVal = sprintf('%+.0f m from the apex', d - c.dist);
        end
    end

    function exportPng()
        try
            prj = currentProject();  root = char(prj.RootFolder);
        catch
            root = pwd;
        end
        outDir = fullfile(root, 'docs', 'img');
        if ~exist(outDir, 'dir'), mkdir(outDir); end
        f = fullfile(outDir, 'corner_map.png');
        exportgraphics(ax, f, 'Resolution', 200, 'BackgroundColor', bg);
        fprintf('Saved %s\n', f);
    end
end