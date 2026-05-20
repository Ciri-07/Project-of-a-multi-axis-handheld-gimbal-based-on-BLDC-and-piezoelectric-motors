function generate_dual_hall_layout_diagram()
%GENERATE_DUAL_HALL_LAYOUT_DIAGRAM
% Create a schematic for the selected dual linear Hall layout:
% 18 mm hollow magnet + two linear Hall sensors + 16-bit ADC.

close all;

scriptDir = fileparts(mfilename('fullpath'));
figDir = fullfile(scriptDir, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

outerR = 9.0;          % mm, 18 mm hollow magnet outer radius
innerR = 4.2;          % mm, illustrative inner hole radius
hallR = 11.5;          % mm, Hall sensor radial position
polePairs = 6;         % 12 poles -> 6 pole pairs, based on 6x12 pole note
hallElecSepDeg = 90;   % desired magnetic/electrical phase separation
hallMechSepDeg = hallElecSepDeg / polePairs;

fig = figure('Color', 'w', 'Position', [100 80 1280 820]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

%% Top-view layout
ax1 = nexttile([2 1]);
hold(ax1, 'on');
axis(ax1, 'equal');
axis(ax1, 'off');
title(ax1, 'Dual Linear Hall Layout: 18 mm Hollow Magnet + 2 Hall Sensors', ...
    'FontWeight', 'bold', 'FontSize', 14);

drawPoleRing(ax1, outerR, innerR, polePairs);
drawCircle(ax1, outerR, [0.15 0.15 0.15], 1.2, '-');
drawCircle(ax1, innerR, [0.15 0.15 0.15], 1.2, '-');
drawCircle(ax1, hallR, [0.45 0.45 0.45], 1.0, '--');

% Center hole and shaft label
fill(ax1, innerR*cos(linspace(0,2*pi,240)), innerR*sin(linspace(0,2*pi,240)), ...
    [1 1 1], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.2);
text(ax1, 0, 0, 'Hollow shaft / cable pass', 'HorizontalAlignment', 'center', ...
    'FontSize', 8.5, 'Color', [0.15 0.15 0.15], ...
    'BackgroundColor', 'w', 'Margin', 2);

% Hall sensors: A at 0 deg, B at +90 electrical deg.
drawHall(ax1, hallR, 0, 'Hall A', [0.10 0.45 0.85]);
drawHall(ax1, hallR, hallMechSepDeg, 'Hall B', [0.05 0.62 0.30]);

% Angle arc
drawArc(ax1, 0, 0, hallR*0.72, 0, hallMechSepDeg, [0.1 0.1 0.1]);
text(ax1, hallR*0.55*cosd(hallMechSepDeg/2), hallR*0.55*sind(hallMechSepDeg/2)+0.25, ...
    sprintf('90 deg magnetic phase\n%.1f deg mechanical if p=%d', hallMechSepDeg, polePairs), ...
    'HorizontalAlignment', 'left', 'FontSize', 8.5, 'Color', [0.1 0.1 0.1], ...
    'BackgroundColor', 'w', 'Margin', 2);

% Mechanical rotation arrow
drawArc(ax1, 0, 0, outerR+1.3, 210, 285, [0.85 0.25 0.15]);
text(ax1, -outerR-2.7, -outerR-1.0, 'Rotor / magnet rotation', ...
    'FontSize', 10, 'Color', [0.85 0.25 0.15], 'FontWeight', 'bold');

% Dimension line for 18 mm OD
plot(ax1, [-outerR outerR], [-outerR-2.5 -outerR-2.5], 'k-', 'LineWidth', 1.0);
plot(ax1, [-outerR -outerR], [-outerR-2.8 -outerR-2.2], 'k-', 'LineWidth', 1.0);
plot(ax1, [outerR outerR], [-outerR-2.8 -outerR-2.2], 'k-', 'LineWidth', 1.0);
text(ax1, 0, -outerR-3.15, '18 mm magnet outer diameter', ...
    'HorizontalAlignment', 'center', 'FontSize', 10);

legendHandles = gobjects(3,1);
legendHandles(1) = patch(ax1, nan, nan, [0.88 0.22 0.20], 'EdgeColor', 'none');
legendHandles(2) = patch(ax1, nan, nan, [0.22 0.36 0.82], 'EdgeColor', 'none');
legendHandles(3) = plot(ax1, nan, nan, 'ks', 'MarkerFaceColor', [0.10 0.45 0.85]);
legend(ax1, legendHandles, {'N pole sector', 'S pole sector', 'Linear Hall IC'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');

xlim(ax1, [-14 14]);
ylim(ax1, [-14 14]);

%% Signal chain
ax2 = nexttile;
axis(ax2, 'off');
xlim(ax2, [0 1]);
ylim(ax2, [0 1]);
title(ax2, 'Signal Chain', 'FontWeight', 'bold', 'FontSize', 13);
boxText = {
    'Hall A / Hall B voltage'
    '16-bit ADC sampling'
    'Offset + amplitude calibration'
    'Phase orthogonalization'
    'atan2 angle extraction'
    'LUT/Fourier compensation'
    'theta_i, omega_i, valid_i'
    };
drawFlow(ax2, boxText);

%% Expected two-channel waveform
ax3 = nexttile;
theta = linspace(0, 360, 800);
hs = sin(deg2rad(theta));
hc = cos(deg2rad(theta));
plot(ax3, theta, hs, 'Color', [0.10 0.45 0.85], 'LineWidth', 1.8);
hold(ax3, 'on');
plot(ax3, theta, hc, 'Color', [0.05 0.62 0.30], 'LineWidth', 1.8);
grid(ax3, 'on');
xlim(ax3, [0 360]);
ylim(ax3, [-1.25 1.25]);
xlabel(ax3, 'Magnetic angle / deg');
ylabel(ax3, 'Normalized Hall signal');
title(ax3, 'Ideal Hall sin/cos Signals After Calibration', 'FontWeight', 'bold');
legend(ax3, {'Hall A: sin', 'Hall B: cos'}, 'Location', 'southoutside', ...
    'Orientation', 'horizontal', 'Box', 'off');

note = sprintf(['Selected scheme: 18 mm hollow magnet + two linear Hall sensors. ', ...
    'For a 12-pole magnet, 90 deg magnetic phase equals %.1f deg mechanical spacing. ', ...
    'Final spacing must be verified by magnetic-field measurement or simulation.'], hallMechSepDeg);
annotation(fig, 'textbox', [0.04 0.01 0.92 0.05], 'String', note, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 10, ...
    'Color', [0.25 0.25 0.25]);

pngPath = fullfile(figDir, 'dual_linear_hall_18mm_hollow_magnet_layout.png');

exportgraphics(fig, pngPath, 'Resolution', 220);

fprintf('Saved layout diagram:\n%s\n', pngPath);
end

function drawPoleRing(ax, outerR, innerR, polePairs)
numPoles = polePairs * 2;
for k = 1:numPoles
    a1 = (k-1) * 360 / numPoles;
    a2 = k * 360 / numPoles;
    t = linspace(deg2rad(a1), deg2rad(a2), 40);
    x = [outerR*cos(t), innerR*cos(fliplr(t))];
    y = [outerR*sin(t), innerR*sin(fliplr(t))];
    if mod(k,2) == 1
        c = [0.88 0.22 0.20];
        label = 'N';
    else
        c = [0.22 0.36 0.82];
        label = 'S';
    end
    patch(ax, x, y, c, 'EdgeColor', [1 1 1], 'LineWidth', 0.5, 'FaceAlpha', 0.88);
    amid = (a1+a2)/2;
    text(ax, 0.72*outerR*cosd(amid), 0.72*outerR*sind(amid), label, ...
        'Color', 'w', 'FontWeight', 'bold', 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
end

function drawCircle(ax, r, c, lw, ls)
t = linspace(0, 2*pi, 360);
plot(ax, r*cos(t), r*sin(t), 'Color', c, 'LineWidth', lw, 'LineStyle', ls);
end

function drawHall(ax, r, angleDeg, label, color)
x = r*cosd(angleDeg);
y = r*sind(angleDeg);
rot = [cosd(angleDeg), -sind(angleDeg); sind(angleDeg), cosd(angleDeg)];
rect = [-0.85 -0.42; 0.85 -0.42; 0.85 0.42; -0.85 0.42]';
rect = rot * rect + [x; y];
patch(ax, rect(1,:), rect(2,:), color, 'EdgeColor', 'k', 'LineWidth', 1.0);
plot(ax, [0 x], [0 y], ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.0);
text(ax, 1.12*x, 1.12*y, label, 'Color', color, 'FontWeight', 'bold', ...
    'FontSize', 10, 'HorizontalAlignment', 'center');
end

function drawArc(ax, cx, cy, r, a1, a2, color)
t = linspace(deg2rad(a1), deg2rad(a2), 80);
x = cx + r*cos(t);
y = cy + r*sin(t);
plot(ax, x, y, 'Color', color, 'LineWidth', 1.5);
% Arrow head
ah = deg2rad(a2);
p = [cx + r*cos(ah), cy + r*sin(ah)];
tanDir = [-sin(ah), cos(ah)];
radDir = [cos(ah), sin(ah)];
head = [p; p - 0.35*tanDir + 0.18*radDir; p - 0.35*tanDir - 0.18*radDir];
patch(ax, head(:,1), head(:,2), color, 'EdgeColor', color);
end

function drawFlow(ax, labels)
hold(ax, 'on');
n = numel(labels);
x = 0.1; w = 0.8; h = 0.085;
yTop = 0.90;
for i = 1:n
    y = yTop - (i-1)*0.125;
    rectangle(ax, 'Position', [x y w h], 'Curvature', 0.05, ...
        'FaceColor', [0.95 0.97 1.0], 'EdgeColor', [0.18 0.33 0.58], 'LineWidth', 1.1);
    text(ax, x+w/2, y+h/2, labels{i}, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontSize', 10);
    if i < n
        x0 = x + w/2;
        y1 = y - 0.006;
        y2 = y - 0.035;
        plot(ax, [x0 x0], [y1 y2], 'Color', [0.18 0.33 0.58], 'LineWidth', 1.1);
        patch(ax, [x0-0.015 x0+0.015 x0], [y2+0.012 y2+0.012 y2-0.008], ...
            [0.18 0.33 0.58], 'EdgeColor', [0.18 0.33 0.58]);
    end
end
xlim(ax, [0 1]);
ylim(ax, [0 1]);
axis(ax, 'off');
end
