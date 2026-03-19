function ui_robot()
% UI_ROBOT
% Enhanced MATLAB UI for the Dobot Magician Lite based on wireframe.
% Features:
%   - Dual Radian/Degree inputs for joints with auto-synchronization
%   - 3D Robot visualization
%   - T04 matrix display
%   - IK target input

% Initialize robot object
d = dobot(0, deg2rad(30), deg2rad(-5), 0, 0.5, 0.5, 0.5);

% ---------- UI Setup ----------
fig = uifigure('Name', 'Dobot Magician Lite - UI Robot', 'Position', [60 60 1720 760], ...
    'WindowKeyPressFcn', @(src, event) onWindowKeyPress(event));
mainLayout = uigridlayout(fig, [1 2]);
mainLayout.ColumnWidth = {'1.55x', '1.35x'};

% --- Left: 3D Plot ---
plotPanel = uipanel(mainLayout, 'Title', '3D Plot of Robot');
ax = uiaxes(plotPanel, 'Position', [10 10 800 640]);
view(ax, 135, 25);
grid(ax, 'on');
grid(ax, 'minor');
hold(ax, 'on');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
axis(ax, [-0.3 0.3 -0.3 0.3 -0.03 0.3]);

% Plot Handles
hLinks = plot3(ax, zeros(1,5), zeros(1,5), zeros(1,5), '-', 'Color', [0.7, 0.7, 0.7], 'LineWidth', 2);
hOrigin = plot3(ax, 0, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
hJ1 = plot3(ax, 0, 0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
hJ2 = plot3(ax, 0, 0, 0, 'yo', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
hJ3 = plot3(ax, 0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hJ4 = plot3(ax, 0, 0, 0, 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
legend(ax, {'Links', 'World Origin', 'Joint 1', 'Joint 2', 'Joint 3', 'End Effector'}, 'Location', 'northeastoutside');

% --- Right: Controls & Displays ---
controlPanel = uipanel(mainLayout, 'Title', 'Controls & Displays');
controlLayout = uigridlayout(controlPanel, [2 1]);
controlLayout.RowHeight = {430, 180};
controlLayout.Padding = [8 8 8 8];

cg = uigridlayout(controlLayout, [13 5]);
cg.RowHeight = {24, 24, 24, 24, 24, 24, 10, 24, 24, 24, 24, 24, '1x'};
cg.ColumnWidth = {40, '1x', '1x', 50, '2x'};
cg.Layout.Row = 1;
cg.Layout.Column = 1;

% Joint Angles & Dynamics Section
addHeader(cg, 'Joint Angles', 1, [1 3]);
addHeader(cg, 'Dynamics Inputs', 1, [4 5]);

lblRad = uilabel(cg, 'Text', 'Radians', 'HorizontalAlignment', 'center');
lblRad.Layout.Row = 2; lblRad.Layout.Column = 2;
lblDeg = uilabel(cg, 'Text', 'Degrees', 'HorizontalAlignment', 'center');
lblDeg.Layout.Row = 2; lblDeg.Layout.Column = 3;

% Create 4 joint rows
[radFields, degFields] = deal(cell(1,4));
for i = 1:4
    idx = i;
    row = 2 + idx;
    lblTheta = uilabel(cg, 'Text', sprintf('\\theta_%d', idx), 'Interpreter', 'latex', 'HorizontalAlignment', 'right');
    lblTheta.Layout.Row = row; lblTheta.Layout.Column = 1;

    radFields{idx} = uieditfield(cg, 'numeric', 'Value', 0, ...
        'ValueChangedFcn', @(src, event) updateFromRad(idx));
    radFields{idx}.Layout.Row = row; radFields{idx}.Layout.Column = 2;

    degFields{idx} = uieditfield(cg, 'numeric', 'Value', 0, ...
        'ValueChangedFcn', @(src, event) updateFromDeg(idx));
    degFields{idx}.Layout.Row = row; degFields{idx}.Layout.Column = 3;
end

% Dynamics fields
mFields = cell(1,3);
defaultMasses = [d.M1, d.M2, d.M3];
for i = 1:3
    idx = i;
    row = 2 + idx;
    lblM = uilabel(cg, 'Text', sprintf('M%d', idx), 'HorizontalAlignment', 'right');
    lblM.Layout.Row = row; lblM.Layout.Column = 4;
    mFields{idx} = uieditfield(cg, 'numeric', 'Value', defaultMasses(idx), ...
        'ValueChangedFcn', @(src, event) updateMassFromUI());
    mFields{idx}.Layout.Row = row; mFields{idx}.Layout.Column = 5;
end

% IK Target Section
addHeader(cg, 'Inverse Kinematics', 8, [1 5]);
lblX = uilabel(cg, 'Text', 'X', 'HorizontalAlignment', 'right');
lblX.Layout.Row = 9; lblX.Layout.Column = 1;
xIn = uieditfield(cg, 'numeric', 'Value', 0.2); xIn.Layout.Row = 9; xIn.Layout.Column = [2 3];

lblY = uilabel(cg, 'Text', 'Y', 'HorizontalAlignment', 'right');
lblY.Layout.Row = 10; lblY.Layout.Column = 1;
yIn = uieditfield(cg, 'numeric', 'Value', 0.0); yIn.Layout.Row = 10; yIn.Layout.Column = [2 3];

lblZ = uilabel(cg, 'Text', 'Z', 'HorizontalAlignment', 'right');
lblZ.Layout.Row = 11; lblZ.Layout.Column = 1;
zIn = uieditfield(cg, 'numeric', 'Value', 0.15); zIn.Layout.Row = 11; zIn.Layout.Column = [2 3];

lblElbow = uilabel(cg, 'Text', 'elbow', 'HorizontalAlignment', 'right');
lblElbow.Layout.Row = 12; lblElbow.Layout.Column = 1;
elbowMode = uidropdown(cg, 'Items', {'up', 'down'}, 'Value', 'up');
elbowMode.Layout.Row = 12; elbowMode.Layout.Column = [2 3];

% Time & Buttons
lblTime = uilabel(cg, 'Text', 'time', 'HorizontalAlignment', 'right');
lblTime.Layout.Row = 9; lblTime.Layout.Column = 4;
timeIn = uieditfield(cg, 'numeric', 'Value', 1.0);
timeIn.Layout.Row = 9; timeIn.Layout.Column = 5;

btnSolve = uibutton(cg, 'Text', 'Solve', 'ButtonPushedFcn', @(~,~) onSolveIK());
btnSolve.Layout.Row = 10; btnSolve.Layout.Column = [4 5];

btnAnimate = uibutton(cg, 'Text', 'Animate', 'ButtonPushedFcn', @(~,~) onAnimateIK());
btnAnimate.Layout.Row = 11; btnAnimate.Layout.Column = [4 5];

displayGrid = uigridlayout(controlLayout, [1 3]);
displayGrid.Layout.Row = 2;
displayGrid.Layout.Column = 1;
displayGrid.ColumnWidth = {'1x', '1x', '1x'};
displayGrid.ColumnSpacing = 8;
displayGrid.Padding = [0 0 0 0];

t04Display = addDisplayPanel(displayGrid, 'Transformation', 1);
jacDisplay = addDisplayPanel(displayGrid, 'Jacobian', 2);
dynDisplay = addDisplayPanel(displayGrid, 'Tau / Terms', 3);

% Initial Sync
syncUI();
updateRobotPlot();

% ---------- Functions ----------

    function updateFromRad(idx)
        val = radFields{idx}.Value;
        degFields{idx}.Value = rad2deg(val);
        updateRobotFromUI();
    end

    function updateFromDeg(idx)
        val = degFields{idx}.Value;
        radFields{idx}.Value = deg2rad(val);
        updateRobotFromUI();
    end

    function updateMassFromUI()
        try
            applyMassInputs();
            updateRobotPlot();
        catch ME
            uialert(fig, ME.message, 'Dynamics Input Error');
            syncUI();
        end
    end

    function onWindowKeyPress(event)
        focusedObj = fig.CurrentObject;
        for j = 1:4
            if isequal(focusedObj, degFields{j})
                switch event.Key
                    case 'uparrow'
                        degFields{j}.Value = degFields{j}.Value + 1;
                        updateFromDeg(j);
                    case 'downarrow'
                        degFields{j}.Value = degFields{j}.Value - 1;
                        updateFromDeg(j);
                end
                break;
            elseif isequal(focusedObj, radFields{j})
                switch event.Key
                    case 'uparrow'
                        radFields{j}.Value = radFields{j}.Value + 0.01;
                        updateFromRad(j);
                    case 'downarrow'
                        radFields{j}.Value = radFields{j}.Value - 0.01;
                        updateFromRad(j);
                end
                break;
            end
        end
    end

    function updateRobotFromUI()
        thetas = [radFields{1}.Value, ...
            radFields{2}.Value, ...
            radFields{3}.Value, ...
            radFields{4}.Value];

        try
            applyMassInputs();
            d.setJointAngles(thetas);
            syncUI(); % Keep IK fields in sync with manual joint movement
            updateRobotPlot();
        catch ME
            uialert(fig, ME.message, 'Joint Limit Error')
            syncUI()
        end
    end

    function syncUI()
        % Sync joint angle fields
        thetas = [d.Theta1, d.Theta2, d.Theta3, d.Theta4];
        for j = 1:4
            radFields{j}.Value = thetas(j);
            degFields{j}.Value = rad2deg(thetas(j));
        end

        for j = 1:3
            mFields{j}.Value = d.(['M' num2str(j)]);
        end

        % Sync IK target fields
        pos = d.xyz();
        roundingTol = 100000;
        xIn.Value = floor(pos(1) * roundingTol) / roundingTol;
        yIn.Value = floor(pos(2) * roundingTol) / roundingTol;
        zIn.Value = floor(pos(3) * roundingTol) / roundingTol;

        % Sync elbow mode dropdown
        if d.elbowUp()
            elbowMode.Value = 'up';
        else
            elbowMode.Value = 'down';
        end
    end

    function updateRobotPlot()
        % Get joint locations
        p0 = [0, 0, 0];
        p1 = d.jointLoc(1);
        p2 = d.jointLoc(2);
        p3 = d.jointLoc(3);
        p4 = d.jointLoc(4);

        % Update link lines
        pts = [p0; p1; p2; p3; p4];
        hLinks.XData = pts(:,1);
        hLinks.YData = pts(:,2);
        hLinks.ZData = pts(:,3);

        % Update individual joint markers
        hOrigin.XData = p0(1); hOrigin.YData = p0(2); hOrigin.ZData = p0(3);
        hJ1.XData = p1(1); hJ1.YData = p1(2); hJ1.ZData = p1(3);
        hJ2.XData = p2(1); hJ2.YData = p2(2); hJ2.ZData = p2(3);
        hJ3.XData = p3(1); hJ3.YData = p3(2); hJ3.ZData = p3(3);
        hJ4.XData = p4(1); hJ4.YData = p4(2); hJ4.ZData = p4(3);

        % Update T04 Display
        T04 = d.transform(1,4);
        setTextAreaContent(t04Display, formattedDisplayText(T04));

        % Update Jacobian Display
        J = d.jacobian();
        setTextAreaContent(jacDisplay, formattedDisplayText(J));

        % Update Dynamics Display using the current configuration and UI masses.
        [tau, terms] = d.inverseDynamics();
        dynText = formatDynamicsDisplay(tau, terms);
        setTextAreaContent(dynDisplay, dynText);

        drawnow limitrate;
    end

    function onSolveIK()
        target = [xIn.Value, yIn.Value, zIn.Value];
        isUp = strcmpi(elbowMode.Value, 'up');
        try
            applyMassInputs();
            d.setEndEffector(target, isUp);
            syncUI();
            updateRobotPlot();
        catch ME
            uialert(fig, ME.message, 'IK Error');
        end
    end

    function onAnimateIK()
        target = [xIn.Value, yIn.Value, zIn.Value];
        isUp = strcmpi(elbowMode.Value, 'up');
        applyMassInputs();

        % Use a temporary dobot to calculate target joint angles
        temp_robot = dobot();
        try
            temp_robot.setEndEffector(target, isUp);
            target_thetas = [temp_robot.Theta1, temp_robot.Theta2, temp_robot.Theta3, temp_robot.Theta4];
        catch ME
            uialert(fig, ME.message, 'IK Target Unreachable');
            return;
        end

        start_thetas = [d.Theta1, d.Theta2, d.Theta3, d.Theta4];

        T = max(0.1, timeIn.Value); % Ensure animation time is at least 0.1s
        fps = 30;
        num_steps = max(2, ceil(T * fps));

        % Generate a smooth trajectory using a cosine easing function
        t = linspace(0, 1, num_steps);
        ease = (1 - cos(pi * t)) / 2;

        thetas_traj = zeros(num_steps, 4);
        for j = 1:4
            thetas_traj(:, j) = start_thetas(j) + (target_thetas(j) - start_thetas(j)) * ease;
        end

        % Disable buttons during animation
        btnSolve.Enable = 'off';
        btnAnimate.Enable = 'off';

        % Run the animation loop
        for stepIdx = 1:num_steps
            try
                d.setJointAngles(thetas_traj(stepIdx, :));
                syncUI();
                updateRobotPlot();
                pause(1/fps); % Control animation speed
            catch ME
                uialert(fig, ['Animation aborted: ' ME.message], 'Joint Limit Error');
                break;
            end
        end

        % Re-enable buttons
        btnSolve.Enable = 'on';
        btnAnimate.Enable = 'on';
    end

    function addHeader(gl, txt, row, col)
        h = uilabel(gl, 'Text', txt, 'FontWeight', 'bold');
        h.Layout.Row = row;
        if nargin > 3
            h.Layout.Column = col;
        else
            h.Layout.Column = [1 3];
        end
    end

    function ta = addDisplayPanel(parent, titleText, col)
        panel = uipanel(parent, 'Title', titleText);
        panel.Layout.Row = 1;
        panel.Layout.Column = col;

        panelGrid = uigridlayout(panel, [1 1]);
        panelGrid.Padding = [4 4 4 4];
        panelGrid.RowHeight = {'1x'};
        panelGrid.ColumnWidth = {'1x'};

        ta = uitextarea(panelGrid, 'Editable', 'off', ...
            'FontName', 'monospaced', 'FontSize', 9);
        ta.Layout.Row = 1;
        ta.Layout.Column = 1;
    end

    function applyMassInputs()
        d.M1 = mFields{1}.Value;
        d.M2 = mFields{2}.Value;
        d.M3 = mFields{3}.Value;
        d.validateMass(d.M1, 'M1');
        d.validateMass(d.M2, 'M2');
        d.validateMass(d.M3, 'M3');
    end

    function setTextAreaContent(area, content)
        area.Value = splitlines(string(content));
    end

    function txt = formatDynamicsDisplay(tau, terms)
        txt = sprintf([ ...
            'tau =\n%s\n\n' ...
            'M =\n%s\n\n' ...
            'coriolis =\n%s\n\n' ...
            'gravity =\n%s\n\n' ...
            'q =\n%s\n\n' ...
            'qd =\n%s\n\n' ...
            'qdd =\n%s'], ...
            roundedDisplayText(tau), ...
            roundedDisplayText(terms.M), ...
            roundedDisplayText(terms.coriolis), ...
            roundedDisplayText(terms.gravity), ...
            roundedDisplayText(terms.q), ...
            roundedDisplayText(terms.qd), ...
            roundedDisplayText(terms.qdd));
    end

    function txt = roundedDisplayText(value)
        txt = formattedDisplayText(round(value, 3));
    end
end
