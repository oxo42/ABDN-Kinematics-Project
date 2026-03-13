function ui_robot_v2()
% UI_ROBOT_V2
% Enhanced MATLAB UI for the Dobot Magician Lite.
% Features:
%   - Slider-based forward kinematics view
%   - XYZ target input for inverse kinematics
%   - Elbow-up / elbow-down branch selection
%   - Simple point-to-point animation
%   - Joint + link visualization in 3D
%
% This file is self-contained and does not require the existing ui_robot.m.
% It can be run directly in MATLAB by saving it as ui_robot_v2.m and calling:
%   >> ui_robot_v2

    % ---------- Robot parameters ----------
    robot.L2 = 0.15;   % metres
    robot.L3 = 0.15;   % metres

    % ---------- UI ----------
    fig = uifigure( ...
        'Name', 'Dobot Magician Lite - UI Robot v2', ...
        'Position', [100 80 1100 680]);

    gl = uigridlayout(fig, [1 2]);
    gl.ColumnWidth = {'2.2x', '1x'};

    % Left side: plot
    plotPanel = uipanel(gl, 'Title', '3D Robot View');
    ax = uiaxes(plotPanel, 'Position', [10 10 700 620]);
    title(ax, 'Dobot Magician Lite')
    xlabel(ax, 'X (m)')
    ylabel(ax, 'Y (m)')
    zlabel(ax, 'Z (m)')
    axis(ax, [-0.35 0.35 -0.35 0.35 -0.05 0.35])
    view(ax, 135, 25)
    grid(ax, 'on')
    hold(ax, 'on')

    % Plot handles
    hOrigin = plot3(ax, 0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
    hJ2     = plot3(ax, 0, 0, 0, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);
    hJ3     = plot3(ax, 0, 0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
    hEE     = plot3(ax, 0, 0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

    hLink1 = plot3(ax, [0 0], [0 0], [0 0], 'm-', 'LineWidth', 3);
    hLink2 = plot3(ax, [0 0], [0 0], [0 0], 'm-', 'LineWidth', 3);
    hLink3 = plot3(ax, [0 0], [0 0], [0 0], 'c-', 'LineWidth', 3);

    % Right side: controls
    controlPanel = uipanel(gl, 'Title', 'Controls');
    cg = uigridlayout(controlPanel, [24 2]);
    cg.RowHeight = repmat({24}, 1, 24);
    cg.ColumnWidth = {120, '1x'};

    % ---------- Joint controls ----------
    addHeader(cg, 'Forward Kinematics / Joint Angles', 1);

    [s1, e1] = addSliderRow(cg, '\theta_1 (deg)', 2, -135, 135, 0);
    [s2, e2] = addSliderRow(cg, '\theta_2 (deg)', 3, -5,   80,  30);
    [s3, e3] = addSliderRow(cg, '\theta_3 (deg)', 4, -10,  85,  20);
    [s4, e4] = addSliderRow(cg, '\theta_4 (deg)', 5, -145, 145, 0);

    % ---------- FK output ----------
    addHeader(cg, 'Current End-Effector Position', 7);
    xOut = addReadout(cg, 'X (m)', 8);
    yOut = addReadout(cg, 'Y (m)', 9);
    zOut = addReadout(cg, 'Z (m)', 10);
    elbowOut = addReadout(cg, 'Elbow branch', 11);

    % ---------- IK controls ----------
    addHeader(cg, 'Inverse Kinematics Target', 13);
    xIn = addNumeric(cg, 'Target X (m)', 14, 0.18);
    yIn = addNumeric(cg, 'Target Y (m)', 15, 0.10);
    zIn = addNumeric(cg, 'Target Z (m)', 16, 0.15);

    uilabel(cg, 'Text', 'IK branch', 'HorizontalAlignment', 'right');
    elbowMode = uidropdown(cg, 'Items', {'down', 'up'}, 'Value', 'down');

    solveBtn = uibutton(cg, 'Text', 'Solve IK → Update Sliders', ...
        'ButtonPushedFcn', @(~,~) onSolveIK());

    % ---------- Animation controls ----------
    addHeader(cg, 'Animation', 19);
    durationIn = addNumeric(cg, 'Duration (s)', 20, 2.0);
    stepsIn = addNumeric(cg, 'Steps', 21, 40);

    animateBtn = uibutton(cg, 'Text', 'Animate Move To Target', ...
        'ButtonPushedFcn', @(~,~) onAnimateMove());

    resetBtn = uibutton(cg, 'Text', 'Reset', ...
        'ButtonPushedFcn', @(~,~) onReset());

    helpText = uitextarea(cg, ...
        'Value', { ...
            'Notes:', ...
            '1) Sliders drive forward kinematics.', ...
            '2) Enter X,Y,Z and choose elbow branch for IK.', ...
            '3) Animate uses linear interpolation in joint space.', ...
            '4) This version includes a simple is_elbow_up helper.'}, ...
        'Editable', 'off');
    helpText.Layout.Row = [23 24];
    helpText.Layout.Column = [1 2];

    % Callbacks
    sliders = [s1 s2 s3 s4];
    edits   = [e1 e2 e3 e4];
    for k = 1:numel(sliders)
        sliders(k).ValueChangingFcn = @(src,event) onSliderChanging(k, event.Value);
        sliders(k).ValueChangedFcn  = @(src,~) onSliderChanged();
        edits(k).ValueChangedFcn    = @(src,~) onEditChanged(k);
    end

    updateRobotPlot();

    % ---------- Nested callbacks ----------
    function onSliderChanging(idx, val)
        edits(idx).Value = round(val, 3);
        updateRobotPlot();
    end

    function onSliderChanged()
        for kk = 1:4
            edits(kk).Value = sliders(kk).Value;
        end
        updateRobotPlot();
    end

    function onEditChanged(idx)
        val = edits(idx).Value;
        sliders(idx).Value = val;
        updateRobotPlot();
    end

    function onSolveIK()
        target = [xIn.Value, yIn.Value, zIn.Value];
        isUp = strcmpi(elbowMode.Value, 'up');

        [ok, qDeg, msg] = inverseKinematicsDobot(target, robot, isUp);
        if ~ok
            uialert(fig, msg, 'IK Error');
            return
        end

        sliders(1).Value = qDeg(1);
        sliders(2).Value = qDeg(2);
        sliders(3).Value = qDeg(3);
        sliders(4).Value = qDeg(4);

        for kk = 1:4
            edits(kk).Value = sliders(kk).Value;
        end

        updateRobotPlot();
    end

    function onAnimateMove()
        target = [xIn.Value, yIn.Value, zIn.Value];
        isUp = strcmpi(elbowMode.Value, 'up');
        [ok, qTargetDeg, msg] = inverseKinematicsDobot(target, robot, isUp);
        if ~ok
            uialert(fig, msg, 'IK Error');
            return
        end

        qStart = [s1.Value s2.Value s3.Value s4.Value];
        nSteps = max(2, round(stepsIn.Value));
        duration = max(0.1, durationIn.Value);

        path = zeros(nSteps, 4);
        for j = 1:4
            path(:, j) = linspace(qStart(j), qTargetDeg(j), nSteps);
        end

        dt = duration / (nSteps - 1);
        for ii = 1:nSteps
            s1.Value = path(ii, 1);
            s2.Value = path(ii, 2);
            s3.Value = path(ii, 3);
            s4.Value = path(ii, 4);
            for kk = 1:4
                edits(kk).Value = sliders(kk).Value;
            end
            updateRobotPlot();
            pause(dt);
        end
    end

    function onReset()
        s1.Value = 0;   s2.Value = 30;
        s3.Value = 20;  s4.Value = 0;
        for kk = 1:4
            edits(kk).Value = sliders(kk).Value;
        end
        updateRobotPlot();
    end

    function updateRobotPlot()
        qDeg = [s1.Value, s2.Value, s3.Value, s4.Value];
        qRad = deg2rad(qDeg);

        [pts, T] = forwardKinematicsDobot(qRad, robot); %#ok<ASGLU>
        p0 = pts(1, :);
        p2 = pts(2, :);
        p3 = pts(3, :);
        p4 = pts(4, :);

        hOrigin.XData = p0(1); hOrigin.YData = p0(2); hOrigin.ZData = p0(3);
        hJ2.XData = p2(1);     hJ2.YData = p2(2);     hJ2.ZData = p2(3);
        hJ3.XData = p3(1);     hJ3.YData = p3(2);     hJ3.ZData = p3(3);
        hEE.XData = p4(1);     hEE.YData = p4(2);     hEE.ZData = p4(3);

        hLink1.XData = [p0(1) p2(1)];
        hLink1.YData = [p0(2) p2(2)];
        hLink1.ZData = [p0(3) p2(3)];

        hLink2.XData = [p2(1) p3(1)];
        hLink2.YData = [p2(2) p3(2)];
        hLink2.ZData = [p2(3) p3(3)];

        hLink3.XData = [p3(1) p4(1)];
        hLink3.YData = [p3(2) p4(2)];
        hLink3.ZData = [p3(3) p4(3)];

        xOut.Value = round(p4(1), 4);
        yOut.Value = round(p4(2), 4);
        zOut.Value = round(p4(3), 4);
        elbowOut.Value = ternary(isElbowUp(qRad(3)), 'up', 'down');

        drawnow limitrate
    end
end

% ========================= Local helper functions =========================

function addHeader(gl, txt, row)
    h = uilabel(gl, 'Text', txt, 'FontWeight', 'bold');
    h.Layout.Row = row;
    h.Layout.Column = [1 2];
end

function [s, e] = addSliderRow(gl, labelText, row, mn, mx, val)
    l = uilabel(gl, 'Text', labelText, 'Interpreter', 'latex', ...
        'HorizontalAlignment', 'right');
    l.Layout.Row = row;
    l.Layout.Column = 1;

    inner = uigridlayout(gl, [1 2]);
    inner.ColumnWidth = {'3x', 70};
    inner.Layout.Row = row;
    inner.Layout.Column = 2;

    s = uislider(inner, 'Limits', [mn mx], 'Value', val);
    e = uieditfield(inner, 'numeric', 'Limits', [mn mx], 'Value', val);
end

function out = addReadout(gl, labelText, row)
    l = uilabel(gl, 'Text', labelText, 'HorizontalAlignment', 'right');
    l.Layout.Row = row;
    l.Layout.Column = 1;

    out = uieditfield(gl, 'text', 'Editable', 'off');
    out.Layout.Row = row;
    out.Layout.Column = 2;
end

function out = addNumeric(gl, labelText, row, val)
    l = uilabel(gl, 'Text', labelText, 'HorizontalAlignment', 'right');
    l.Layout.Row = row;
    l.Layout.Column = 1;

    out = uieditfield(gl, 'numeric', 'Value', val);
    out.Layout.Row = row;
    out.Layout.Column = 2;
end

function [points, T_all] = forwardKinematicsDobot(q, robot)
% q = [theta1 theta2 theta3 theta4] in radians

    th1 = q(1); th2 = q(2); th3 = q(3); th4 = q(4);

    A1 = dh(0,         pi/2, 0, th1);
    A2 = dh(robot.L2,  0,    0, th2);
    A3 = dh(robot.L3,  0,    0, th3);
    Ap = dh(0,         pi/2, 0, -th2 - th3); % passive link
    A4 = dh(0,         0,    0, th4);

    T01 = A1;
    T02 = A1 * A2;
    T03 = A1 * A2 * A3;
    T04 = A1 * A2 * A3 * Ap * A4;

    p0 = [0 0 0];
    p2 = T02(1:3, 4).';
    p3 = T03(1:3, 4).';
    p4 = T04(1:3, 4).';

    points = [p0; p2; p3; p4];
    T_all = struct('T01', T01, 'T02', T02, 'T03', T03, 'T04', T04);
end

function [ok, qDeg, msg] = inverseKinematicsDobot(targetXYZ, robot, elbowUp)
% Returns qDeg = [theta1 theta2 theta3 theta4] in degrees.
% Simple geometric IK for the current project model.

    x = targetXYZ(1);
    y = targetXYZ(2);
    z = targetXYZ(3);

    L2 = robot.L2;
    L3 = robot.L3;

    r = sqrt(x^2 + y^2);
    h = sqrt(r^2 + z^2);

    % Reachability check
    if h > (L2 + L3) + 1e-9
        ok = false;
        qDeg = [0 0 0 0];
        msg = sprintf('Target is outside reachable workspace. h = %.4f m > %.4f m.', h, L2 + L3);
        return
    end

    c3 = (r^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3);
    c3 = max(min(c3, 1), -1);
    s3mag = sqrt(max(0, 1 - c3^2));

    if elbowUp
        s3 = -s3mag;
    else
        s3 = +s3mag;
    end

    theta1 = atan2(y, x);
    theta3 = atan2(s3, c3);

    % Standard 2-link planar IK
    theta2 = atan2(z, r) - atan2(L3 * s3, L2 + L3 * c3);

    % Keep wrist neutral by default; passive link already helps maintain orientation
    theta4 = 0;

    qDeg = rad2deg([theta1 theta2 theta3 theta4]);
    ok = true;
    msg = '';
end

function tf = isElbowUp(theta3)
% A simple helper aligned with John's idea.
% Negative theta3 is interpreted as elbow-up in this project convention.
    tf = theta3 < 0;
end

function T = dh(a, alpha, d, theta)
    T = [ ...
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta); ...
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta); ...
        0,           sin(alpha),             cos(alpha),            d; ...
        0,           0,                      0,                     1];
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end
