function ui_robot()
% UI_ROBOT
% Enhanced MATLAB UI for the Dobot Magician Lite based on wireframe.
% Features:
%   - Dual Radian/Degree inputs for joints with auto-synchronization
%   - 3D Robot visualization
%   - T04 matrix display
%   - IK target input

% Initialize robot object
d = dobot(0, deg2rad(30), deg2rad(-20), 0);

% ---------- UI Setup ----------
fig = uifigure('Name', 'Dobot Magician Lite - UI Robot', 'Position', [100 80 1200 700], ...
    'WindowKeyPressFcn', @(src, event) onWindowKeyPress(event));
mainLayout = uigridlayout(fig, [1 2]);
mainLayout.ColumnWidth = {'2.5x', '1x'};

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

% --- Right: Controls ---
controlPanel = uipanel(mainLayout, 'Title', 'Controls');
cg = uigridlayout(controlPanel, [22 3]);
cg.RowHeight = repmat({24}, 1, 22);
cg.ColumnWidth = {80, '1x', '1x'};

% Joint Angles Section
addHeader(cg, 'Joint Angles', 1);
lblRad = uilabel(cg, 'Text', 'Radians', 'HorizontalAlignment', 'center');
lblRad.Layout.Row = 2; lblRad.Layout.Column = 2;
lblDeg = uilabel(cg, 'Text', 'Degrees', 'HorizontalAlignment', 'center');
lblDeg.Layout.Row = 2; lblDeg.Layout.Column = 3;

% Create 4 joint rows
[radFields, degFields] = deal(cell(1,4));
for i = 1:4
    row = 2 + i;
    lblTheta = uilabel(cg, 'Text', sprintf('\\theta_%d', i), 'Interpreter', 'latex', 'HorizontalAlignment', 'right');
    lblTheta.Layout.Row = row;

    radFields{i} = uieditfield(cg, 'numeric', 'Value', 0, 'ValueChangedFcn', @(src, event) updateFromRad(i));
    radFields{i}.Layout.Row = row; radFields{i}.Layout.Column = 2;

    degFields{i} = uieditfield(cg, 'numeric', 'Value', 0, ...
        'ValueChangedFcn', @(src, event) updateFromDeg(i));
    degFields{i}.Layout.Row = row; degFields{i}.Layout.Column = 3;
end

% IK Target Section
addHeader(cg, 'Inverse Kinematics Target', 8);
lblX = uilabel(cg, 'Text', 'X (m)', 'HorizontalAlignment', 'right');
lblX.Layout.Row = 9;
xIn = uieditfield(cg, 'numeric', 'Value', 0.2); xIn.Layout.Row = 9; xIn.Layout.Column = 2;

lblY = uilabel(cg, 'Text', 'Y (m)', 'HorizontalAlignment', 'right');
lblY.Layout.Row = 10; lblY.Layout.Column = 1;
yIn = uieditfield(cg, 'numeric', 'Value', 0.0); yIn.Layout.Row = 10; yIn.Layout.Column = 2;

lblZ = uilabel(cg, 'Text', 'Z (m)', 'HorizontalAlignment', 'right');
lblZ.Layout.Row = 11; lblZ.Layout.Column = 1;
zIn = uieditfield(cg, 'numeric', 'Value', 0.15); zIn.Layout.Row = 11; zIn.Layout.Column = 2;

lblElbow = uilabel(cg, 'Text', 'elbow', 'HorizontalAlignment', 'right');
lblElbow.Layout.Row = 12; lblElbow.Layout.Column = 1;
elbowMode = uidropdown(cg, 'Items', {'up', 'down'}, 'Value', 'up');
elbowMode.Layout.Row = 12; elbowMode.Layout.Column = [2 3];

btnSolve = uibutton(cg, 'Text', 'Solve IK', 'ButtonPushedFcn', @(~,~) onSolveIK());
btnSolve.Layout.Row = 13; btnSolve.Layout.Column = [2 3];

% T04 Display Section
addHeader(cg, 'T04 Matrix / Position', 15);
t04Display = uitextarea(cg, 'Editable', 'off', 'FontName', 'monospaced', 'FontSize', 10);
t04Display.Layout.Row = [16 20];
t04Display.Layout.Column = [1 3];

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
            d.setJointAngles(thetas);
            updateRobotPlot();
        catch ME
            uialert(fig, ME.message, 'Joint Limit Error')
            syncUI()
        end
    end

    function syncUI()
        thetas = [d.Theta1, d.Theta2, d.Theta3, d.Theta4];
        for j = 1:4
            radFields{j}.Value = thetas(j);
            degFields{j}.Value = rad2deg(thetas(j));
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
        tStr = sprintf('T04 Matrix:\n%s\n\nPosition [X Y Z]:\n[%.4f, %.4f, %.4f]', ...
            formattedDisplayText(T04), p4(1), p4(2), p4(3));
        t04Display.Value = tStr;

        drawnow limitrate;
    end

    function onSolveIK()
        target = [xIn.Value, yIn.Value, zIn.Value];
        isUp = strcmpi(elbowMode.Value, 'up');
        try
            d.setEndEffector(target, isUp);
            syncUI();
            updateRobotPlot();
        catch ME
            uialert(fig, ME.message, 'IK Error');
        end
    end

    function addHeader(gl, txt, row)
        h = uilabel(gl, 'Text', txt, 'FontWeight', 'bold');
        h.Layout.Row = row;
        h.Layout.Column = [1 3];
    end
end
