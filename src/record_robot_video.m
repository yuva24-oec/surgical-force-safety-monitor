% ============================================
% SURGICAL ROBOT TRAJECTORY VIDEO v6
% With stomach surface + contact marker
% ============================================
clear; clc;

%% Load robot
robot = loadrobot('universalUR5e', 'DataFormat', 'row');

%% Elbow-up seed configuration
elbowUpConfig = [0, -pi/3, -pi/2, -pi/2, pi/2, 0];

%% Find end effector position in elbow-up config
T_home  = getTransform(robot, elbowUpConfig, 'tool0');
homePos = T_home(1:3, 4);
fprintf('Elbow-up end effector position:\n');
fprintf('  X=%.3f  Y=%.3f  Z=%.3f\n', ...
        homePos(1), homePos(2), homePos(3));

baseX = homePos(1);
baseY = homePos(2);
baseZ = homePos(3);

%% Waypoints
waypoints = [baseX  baseY  baseZ+0.08;
             baseX  baseY  baseZ+0.04;
             baseX  baseY  baseZ;
             baseX  baseY  baseZ;
             baseX  baseY  baseZ+0.04;
             baseX  baseY  baseZ+0.08];

fprintf('\nWaypoints:\n');
for i = 1:size(waypoints,1)
    fprintf('  WP%d: X=%.3f Y=%.3f Z=%.3f\n', ...
            i, waypoints(i,1), waypoints(i,2), waypoints(i,3));
end

%% Tool orientation — pointing down
Ry180 = [ cos(pi) 0 sin(pi);
          0       1 0;
         -sin(pi) 0 cos(pi)];
toolRotation = rotm2tform(Ry180);

%% IK — chained from elbow-up seed
ik      = inverseKinematics('RigidBodyTree', robot);
weights = [0.1 0.1 0.1 1 1 1];
endEff  = 'tool0';
nWP     = size(waypoints, 1);
qConfig = zeros(nWP, 6);

fprintf('\nSolving IK...\n');
pose = trvec2tform(waypoints(1,:)) * toolRotation;
[qConfig(1,:), info] = ik(endEff, pose, weights, elbowUpConfig);
fprintf('  WP1: %s | elbow=%.3f\n', info.Status, qConfig(1,3));

for i = 2:nWP
    pose = trvec2tform(waypoints(i,:)) * toolRotation;
    [qConfig(i,:), info] = ik(endEff, pose, weights, ...
                               qConfig(i-1,:));
    fprintf('  WP%d: %s | elbow=%.3f\n', ...
            i, info.Status, qConfig(i,3));
end

%% Preview
fprintf('\nPreviewing waypoints...\n');
figP = figure('Name', 'Preview', 'Color', 'black', ...
              'Position', [100 100 800 600]);
axP  = axes('Parent', figP, ...
            'Color', [0.08 0.08 0.08], ...
            'XColor', [0.8 0.8 0.8], ...
            'YColor', [0.8 0.8 0.8], ...
            'ZColor', [0.8 0.8 0.8]);
phaseNames = {'Hover','Descend','Contact','Hold','Lift','Retract'};
for i = 1:nWP
    show(robot, qConfig(i,:), 'Parent', axP, ...
         'PreservePlot', false, 'FastUpdate', false);
    view(axP, 135, 20);
    grid(axP, 'on');
    title(axP, sprintf('WP%d — %s | elbow=%.2f rad', ...
          i, phaseNames{i}, qConfig(i,3)), ...
          'Color', 'white', 'FontSize', 12);
    drawnow;
    pause(0.8);
end
fprintf('Press any key to record...\n');
pause;
close(figP);

%% Interpolate
framesPerSegment = [45, 45, 30, 30, 45];
qFull = [];
for i = 1:nWP-1
    seg = zeros(framesPerSegment(i), 6);
    for j = 1:6
        seg(:,j) = linspace(qConfig(i,j), ...
                             qConfig(i+1,j), ...
                             framesPerSegment(i))';
    end
    qFull = [qFull; seg];
end
totalFrames = size(qFull, 1);
fprintf('Total frames: %d\n', totalFrames);

%% Phase boundaries
cumF   = cumsum(framesPerSegment);
labels = {'Phase 1: Approach', 'Phase 2: Descend', ...
          'Phase 3: Tissue Contact', 'Phase 4: Hold', ...
          'Phase 5: Retract'};

%% Build stomach surface mesh (computed once)
[sx, sy] = meshgrid(...
    linspace(baseX-0.35, baseX+0.35, 40), ...
    linspace(baseY-0.35, baseY+0.35, 40));

% Curved abdomen shape — raised in center
sz = (baseZ - 0.06) + ...
      0.02 * exp(-((sx-baseX).^2 + ...
                    (sy-baseY).^2) / 0.06);

%% Recording figure
fig = figure('Color', 'black', 'Position', [100 100 900 650]);
ax  = axes('Parent', fig, ...
           'Color',     [0.08 0.08 0.08], ...
           'XColor',    [0.8 0.8 0.8], ...
           'YColor',    [0.8 0.8 0.8], ...
           'ZColor',    [0.8 0.8 0.8], ...
           'GridColor', [0.3 0.3 0.3], ...
           'GridAlpha', 0.4);
grid(ax, 'on');
hold(ax, 'on');

%% Draw stomach surface (once — before loop)
surf(ax, sx, sy, sz, ...
     'FaceColor',   [0.80 0.58 0.50], ...
     'EdgeColor',   'none', ...
     'FaceAlpha',   0.80, ...
     'FaceLighting','gouraud');

% Subtle grid lines on surface
surf(ax, sx, sy, sz - 0.001, ...
     'FaceColor', 'none', ...
     'EdgeColor', [0.55 0.35 0.30], ...
     'EdgeAlpha', 0.15);

% Lighting for depth
light(ax, 'Position', [baseX+0.6  baseY+0.4  baseZ+1.2], ...
          'Style',    'infinite', ...
          'Color',    [1.0 0.95 0.90]);
light(ax, 'Position', [baseX-0.4  baseY-0.6  baseZ+0.8], ...
          'Style',    'infinite', ...
          'Color',    [0.35 0.35 0.50]);

% Contact marker (hidden until tool is close)
hContact = plot3(ax, baseX, baseY, baseZ, ...
                 'o', ...
                 'MarkerSize',      8, ...
                 'MarkerFaceColor', [1.0 0.25 0.25], ...
                 'MarkerEdgeColor', 'white', ...
                 'LineWidth',       1.5, ...
                 'Visible',         'off');

% Labels
text(ax, baseX-0.32, baseY-0.30, baseZ+0.55, ...
     'Laparoscopic Force Control Simulation', ...
     'Color',     [0.70 0.70 0.70], ...
     'FontSize',  9, ...
     'FontAngle', 'italic');
text(ax, baseX-0.32, baseY-0.30, baseZ+0.48, ...
     'Tissue: Gastric Wall  |  Safety Threshold: 5.0 N', ...
     'Color',    [0.50 0.80 0.50], ...
     'FontSize', 9);

%% Video writer
v           = VideoWriter('robot_surgical_trajectory.mp4', 'MPEG-4');
v.FrameRate = 30;
v.Quality   = 95;
open(v);

fprintf('Recording...\n');

%% Recording loop
for k = 1:totalFrames

    % Clear everything and redraw from scratch each frame
    cla(ax);
    hold(ax, 'on');

    % 1. Draw stomach surface FIRST (bottom layer)
    surf(ax, sx, sy, sz, ...
         'FaceColor',    [0.80 0.58 0.50], ...
         'EdgeColor',    'none', ...
         'FaceAlpha',    0.85, ...
         'FaceLighting', 'gouraud');

    % Subtle surface grid
    surf(ax, sx, sy, sz - 0.001, ...
         'FaceColor', 'none', ...
         'EdgeColor', [0.55 0.35 0.30], ...
         'EdgeAlpha', 0.15);

    % Relighting every frame
    light(ax, 'Position', [baseX+0.6  baseY+0.4  baseZ+1.2], ...
              'Style',    'infinite', ...
              'Color',    [1.0 0.95 0.90]);
    light(ax, 'Position', [baseX-0.4  baseY-0.6  baseZ+0.8], ...
              'Style',    'infinite', ...
              'Color',    [0.35 0.35 0.50]);

    % 2. Draw robot ON TOP of surface
    show(robot, qFull(k,:), ...
         'Parent',       ax, ...
         'FastUpdate',   false, ...
         'PreservePlot', true);

    % 3. Contact marker
    T_tool = getTransform(robot, qFull(k,:), 'tool0');
    toolZ  = T_tool(3, 4);
    dist   = toolZ - baseZ;

    if dist < 0.03
        markerSize = max(6, 12 - dist*150);
        plot3(ax, baseX, baseY, baseZ, ...
              'o', ...
              'MarkerSize',      markerSize, ...
              'MarkerFaceColor', [1.0 0.2 0.2], ...
              'MarkerEdgeColor', 'white', ...
              'LineWidth',       1.5);
    end

    % 4. Info labels
    text(ax, baseX-0.32, baseY-0.30, baseZ+0.55, ...
         'Laparoscopic Force Control Simulation', ...
         'Color',     [0.70 0.70 0.70], ...
         'FontSize',  9, ...
         'FontAngle', 'italic');
    text(ax, baseX-0.32, baseY-0.30, baseZ+0.48, ...
         'Tissue: Gastric Wall  |  Safety Threshold: 5.0 N', ...
         'Color',    [0.50 0.80 0.50], ...
         'FontSize', 9);

    % 5. Axis styling
    ax.Color    = [0.08 0.08 0.08];
    ax.XColor   = [0.8  0.8  0.8];
    ax.YColor   = [0.8  0.8  0.8];
    ax.ZColor   = [0.8  0.8  0.8];
    grid(ax, 'on');
    view(ax, 135, 20);
    axis(ax, [baseX-0.9  baseX+0.5 ...
              baseY-0.7  baseY+0.7 ...
              -0.1       baseZ+0.7]);

    % 6. Title and progress
    idx = find(k <= cumF, 1, 'first');
    if isempty(idx); idx = length(labels); end
    progress = round((k / totalFrames) * 100);

    title(ax, ...
          sprintf('UR5e Surgical Robot — %s', labels{idx}), ...
          'Color', 'white', 'FontSize', 13, 'FontWeight', 'bold');
    xlabel(ax, sprintf('Progress: %d%%', progress), ...
           'Color', [0.7 0.7 0.7], 'FontSize', 11);

    drawnow;
    frame = getframe(fig);
    writeVideo(v, frame);

    if mod(k, 30) == 0
        fprintf('  %d / %d frames\n', k, totalFrames);
    end
end
%% Finish
close(v);
close(fig);
fprintf('\nDone. Saved: %s\n', ...
        fullfile(pwd, 'robot_surgical_trajectory.mp4'));
