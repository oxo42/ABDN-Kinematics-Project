classdef dobot < handle
    %ROBOT Dobot Magician Lite
    %   This class represents a simplified model of the Dobot Magician Lite. We
    %   are modelling it as an open chain rather than a series of 4-bar
    %   linkages.  There is a passive join to ensure the end effector remains
    %   parallel to the ground
    %
    %   All angles are in Radians
    %   All distances are in metres

    properties
        % Joint angles (radians)
        %
        % Represents the angular position of each joint in radians.
        % Theta1: Base rotation
        % Theta2: Shoulder joint
        % Theta3: Elbow joint
        % Theta4: Wrist rotation

        Theta1
        Theta2
        Theta3
        Theta4
        L1 = 0.107 % Link from base to joint 1
        L2 = 0.15 % Link between joints 1 and 2
        L3 = 0.15 % Link between joints 2 and 3
        JointLimits = [
            deg2rad(-135), deg2rad(135); % theta1
            deg2rad(-5),   deg2rad(80);  % theta2
            deg2rad(-10),  deg2rad(85);  % theta3
            deg2rad(-145), deg2rad(145) % theta4
            ];
        % Dynamic parameters arbitraly chosen
        m1 = 0.20    % equivalent mass for upper arm
        m2 = 0.20    % equivalent mass for forearm
        m3 = 0.05    % equivalent mass for wrist / tool / payload

        lc2 = 0.075  % COM of upper arm from shoulder
        lc3 = 0.075  % COM of forearm from elbow

        I2 = 1e-3    % inertia of upper arm about COM
        I3 = 1e-3    % inertia of forearm about COM
        J0 = 1e-3    % base rotary inertia
        J4 = 1e-4    % wrist/tool rotary inertia

        g = 9.81
    end

    methods
        function obj = dobot(theta1, theta2, theta3, theta4)
            %DOBOT Construct an instance of this class
            arguments
                theta1 (1,1) double = 0
                theta2 (1,1) double = 0
                theta3 (1,1) double = 0
                theta4 (1,1) double = 0
            end
            obj.Theta1 = theta1;
            obj.Theta2 = theta2;
            obj.Theta3 = theta3;
            obj.Theta4 = theta4;
        end

        function E = elbowUp(obj)
            %ELBOWUP Returns true if the elbow is in the "up" configuration.
            %   The elbow up configuration is defined as Theta3 < 0.
            %
            %   Returns:
            %       E (logical): True if elbow is up, false otherwise.

            E = obj.Theta3 < 0;
        end

        function validateJointLimits(obj, thetas)
            if nargin < 2
                thetas = [obj.Theta1 obj.Theta2 obj.Theta3 obj.Theta4];
            end

            lim = obj.JointLimits;
            for k = 1:4
                if thetas(k) < lim(k,1) || thetas(k) > lim(k,2)
                    error(['Joint %d angle is outside allowed limits.\n\n' ...
                        'Value: %.2f deg\nAllowed: [%.2f deg , %.2f deg]'], ...
                        k, rad2deg(thetas(k)), ...
                        rad2deg(lim(k,1)), rad2deg(lim(k,2)));
                end
            end
        end

        function setJointAngles(obj, thetas)
            obj.validateJointLimits(thetas);
            obj.Theta1 = thetas(1);
            obj.Theta2 = thetas(2);
            obj.Theta3 = thetas(3);
            obj.Theta4 = thetas(4);
        end


        function A = jointMatrix(obj, i)
            % Returns the homogenous transformation matrix for a specific
            % joint
            switch i
                case 1
                    A = dh(0, pi/2, obj.L1, obj.Theta1);
                case 2
                    A = dh(obj.L2, 0, 0, obj.Theta2);
                case 3
                    A = dh(obj.L3, 0, 0, obj.Theta3);
                case 'p'
                    % The passive joint that rotates the end effector
                    % parallel to the ground
                    % We rotate into the end effector frame here (alpha) after
                    % we do the parallel to ground roatation
                    A = dh(0, pi/2, 0, - obj.Theta2 - obj.Theta3);
                case 4
                    A = dh(0, 0, 0, obj.Theta4);
                otherwise
                    error("unknown joint")
            end
        end


        function T = transform(obj, i, j)
            %TRANSFORM Calculates the numerical homogeneous transformation matrix.
            %   Evaluates the homogenous transformation matrices to return a
            %   tranformation matrix frame i to frame j.
            %
            % Parameters:
            %   i (int): Start frame index.
            %   j (int): End frame index.
            %
            % Returns:
            %   T (4x4 double): Numeric homogeneous transformation matrix.

            T = obj.jointMatrix(i);
            for k = i+1:j
                if k == 4
                    % If we go to the end effector we need to include the
                    % passive joint to rotate the end effector parallel to
                    % the ground
                    T = T * obj.jointMatrix('p');
                end
                T = T * obj.jointMatrix(k);
            end
        end

        function loc = jointLoc(obj, j)
            %JOINTLOC Returns the XYZ coordinates of joint j in the base frame.
            %
            % Parameters:
            %   j (int): Joint number (1 to 4).
            %
            % Returns:
            %   loc (1x3 double): XYZ coordinates of joint j in the base frame.

            T = obj.transform(1,j);
            loc = T(1:3, 4)';
        end

        function loc = xyz(obj)
            %XYZ Returns the XYZ coordinates of the end effector in the base frame.
            %
            % Returns:
            %   loc (1x3 double): XYZ coordinates of the end effector.
            loc = obj.o()';
        end

        function loc = o(obj, n)
            %o returns the xyz coordinates as a 3x1
            arguments
                obj
                n=4 % The joint number
            end
            if n == 0
                loc = [0;0;0];  % Origin is o0
            else
                T = obj.transform(1,n);
                loc = T(1:3, 4);
            end
        end

        function obj = setEndEffector(obj, xyz, elbowUp)
            %SETENDEFFECTOR Inverse kinematics solver for the Dobot Magician Lite.
            %   Calculates the joint angles required to position the end effector at the
            %   specified XYZ coordinates.
            %
            % Parameters:
            %   xyz (1x3 double): Desired XYZ coordinates of the end effector.
            %   elbowUp (logical, optional):  True for elbow-up configuration (default), false for elbow-down.
            %
            % Returns:
            %   None (modifies the object's joint angles).
            %
            % Throws:
            %   Error if the target is outside the reachable workspace.
            % xyz is a 1x3 matrix
            arguments
                obj
                xyz
                elbowUp = true
            end

            x = xyz(1);
            y = xyz(2);
            z_rel = xyz(3) - obj.L1; % Target Z relative to shoulder
            l2 = obj.L2;
            l3 = obj.L3;

            r = sqrt(x^2 + y^2);
            h = sqrt(r^2 + z_rel^2);

            % % Reachability check
            if h > (l2 + l3) + 1e-9
                error('Target is outside reachable workspace. h = %.4f m > %.4f m.', h, l2 + l3);
            end
            %
            c3 = (r^2 + z_rel^2 - l2^2 - l3^2) / (2 * l2 * l3);
            c3 = max(min(c3, 1), -1);

            obj.Theta1 = atan2(y, x);

            s3 = sqrt(1-c3^2);
            if elbowUp
                s3 = -s3;
            end
            obj.Theta3 = atan2(s3, c3);

            if elbowUp
                obj.Theta2 = atan2(z_rel,r) + acos(h/(2 * l2));
            else
                obj.Theta2 = atan2(z_rel,r) - acos(h/(2 * l2));
            end

            % Check limits before finalizing
            obj.validateJointLimits();
        end

        function R = z(obj, i)
            %Z Returns the Z-axis of the rotation matrix for frame i in the base frame.
            %
            % Parameters:
            %   i (int): Frame number (0 to 4).
            %
            % Returns:
            %   R (3x1 double): Z-axis of the rotation matrix.


            if i == 0
                R = [0 0 1]';
            else
                R = obj.transform(1, i);
                R = R(1:3, 3);
            end
        end

        function J = jacobian(obj)
            %JACOBIAN Calculates the Geometric Jacobian matrix.
            %   The Geometric Jacobian relates joint velocities to end-effector linear
            %   and angular velocities.
            %   Jv = zi-1 X (On - Oi-1)
            %   Jw = Zi-1
            %
            % Returns:
            %   J (6x4 double): Geometric Jacobian matrix.
            %   Rows 1-3: Linear velocity Jacobian
            %   Rows 4-6: Angular velocity Jacobian
            %
            jv = @(i) cross(obj.z(i-1), obj.o(4) - obj.o(i-1));
            jw = @(i) obj.z(i-1);

            Jv = [jv(1) jv(2) jv(3) jv(4)];
            Jw = [jw(1) jw(2) jw(3) jw(4)];
            J = [Jv ; Jw];
        end

        function Ja = analyticalJacobian(obj)
            %ANALYTICALJACOBIAN Calculates the Analytical Jacobian matrix.
            %   The Analytical Jacobian relates joint velocities to the task vector:
            %   [x; y; z; psi]
            %   where psi = theta1 + theta4 (end-effector orientation)
            %
            % Returns:
            %   Ja (4x4 double): Analytical Jacobian matrix.
            %   Row 1: Linear velocity in X
            %   Row 2: Linear velocity in Y
            %   Row 3: Linear velocity in Z
            %   Row 4: Angular velocity around Z (psi)
            %
            % Assumptions:
            %   L2 = obj.L2 m
            % L3 = 0.15 m
            % Lt = 0

            q1 = obj.Theta1;
            q2 = obj.Theta2;
            q3 = obj.Theta3;
            % Does not affect the location of the end effector. Rotation only
            % q4 = obj.Theta4;
            l2 = obj.L2;
            l3 = obj.L3;

            r = l2*cos(q2) + l3*cos(q2 + q3);

            Ja = [
                -r*sin(q1),  cos(q1)*(-l2*sin(q2) - l3*sin(q2 + q3)),  -l3*sin(q2 + q3)*cos(q1),  0;
                r*cos(q1),   sin(q1)*(-l2*sin(q2) - l3*sin(q2 + q3)),  -l3*sin(q2 + q3)*sin(q1),  0;
                0,           l2*cos(q2) + l3*cos(q2 + q3),              l3*cos(q2 + q3),          0;
                1,           0,                                          0,                       1
                ];
        end

        function detJa = singularityDet(obj)
            %SINGULARITYDET Calculates the determinant of the Analytical Jacobian.
            %   The determinant is used to detect singularities in the robot's
            %   configuration.  The robot is singular when detJa = 0.
            %
            % Returns:
            %   detJa (double): Determinant of the Analytical Jacobian matrix.
            %
            % See also:
            %   isSingular

            q2 = obj.Theta2;
            q3 = obj.Theta3;
            l2 = obj.L2;
            l3 = obj.L3;

            r = l2*cos(q2) + l3*cos(q2 + q3);

            detJa = l2 * l3 * sin(q3) * r;
        end

        function tf = isSingular(obj, tol)
            %ISSINGULAR Determines if the robot is in a singular configuration.
            %   A singular configuration occurs when the determinant of the Jacobian is
            %   close to zero.
            %
            % Returns:
            %   tf (logical): True if the robot is in a singular configuration, false otherwise.
            %

            arguments
                obj
                tol = 1e-6
            end

            tf = abs(obj.singularityDet()) < tol;
        end

        function [cond1, cond2] = singularityConditions(obj, tol)
            %SINGULARITYCONDITIONS Returns the two singularity conditions separately.
            %   These conditions can be used to analyze the robot's singularities in more detail.
            %
            % Returns:
            %   cond1 (logical): True if sin(theta3) is close to zero.
            %   cond2 (logical): True if L2*cos(theta2) + L3*cos(theta2+theta3) is close to zero.
            %
            % Details:
            %   cond1: sin(theta3) = 0
            %   cond2: L2*cos(theta2) + L3*cos(theta2+theta3) = 0


            arguments
                obj
                tol = 1e-6
            end

            q2 = obj.Theta2;
            q3 = obj.Theta3;

            l2 = obj.L2;
            l3 = obj.L3;

            cond1 = abs(sin(q3)) < tol;
            cond2 = abs(l2*cos(q2) + l3*cos(q2 + q3)) < tol;
        end
    end
end

function matrix = dh(a, alpha, d, theta)
%DH_LINK_TO_TRANSFORMATION_MATRIX Turn DH link parameters into a homogenous transformation matricx
%   Detailed explanation goes here
%   a: metres
%   alpha: radians
%   d: metres
%   theta: radians
arguments (Input)
    a
    alpha
    d
    theta
end

arguments (Output)
    matrix
end

matrix = [
    cos(theta)  -1*sin(theta)*cos(alpha)    sin(theta)*sin(alpha)       a*cos(theta) ;
    sin(theta)  cos(theta)*cos(alpha)       -1*cos(theta)*sin(alpha)    a*sin(theta) ;
    0           sin(alpha)                  cos(alpha)                  d            ;
    0           0                           0                           1
    ];
end
