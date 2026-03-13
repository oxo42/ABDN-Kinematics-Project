classdef dobot < handle
    %ROBOT Dobot Magician Lite
    %   Detailed explanation goes here

    properties
        Theta1
        Theta2
        Theta3
        Theta4
    end

    methods
        function obj = dobot()
            %ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.Theta1 = 0;
            obj.Theta2 = 0;
            obj.Theta3 = 0;
            obj.Theta4 = 0;
        end

        function A = joint_matrix(obj, i)
            % Returns the homogenous transformation matrix for a specific
            % joint
            syms theta1 theta2 theta3 theta4
            switch i
                case 1
                    A = dh_link_to_transformation_matrix(0, pi/2, 0, theta1);
                case 2
                    A = dh_link_to_transformation_matrix(0.15, 0, 0, theta2);
                case 3
                    A = dh_link_to_transformation_matrix(0.15, 0, 0, theta3);
                case 'p'
                    % The passive joint that rotates the end effector
                    % parallel to the ground
                    A = dh_link_to_transformation_matrix(0, pi/2, 0, - theta2 - theta3); 
                case 4
                    A = dh_link_to_transformation_matrix(0, 0, 0, theta4); 
                otherwise
                    error("unknown joint")
            end
        end

        function T = transform_equation(obj, i, j)
            % returns the transformation matrix in symbolic equation form
            % for joint j from frame i so the end effector would be 0, 4
            T = obj.joint_matrix(i);
            for k = i+1:j
                if k == 4
                    % If we go to the end effector we need to include the
                    % passive joint to rotate the end effector parallel to
                    % the ground
                    T = T * obj.joint_matrix('p');
                end
                T = T * obj.joint_matrix(k);
            end
        end

        function T = transform(obj, i, j)
            t = obj.transform_equation(i, j);
            func = matlabFunction(t);
            thetas = [obj.Theta1 obj.Theta2, obj.Theta3, obj.Theta4];
            args = num2cell(thetas(1:j));
            T = func(args{:});
        end

        function loc = joint_loc(obj, j)
            % Returns a matrix of XYZ coordinates of joint j in the base
            % frame
            T = obj.transform(1,j);
            loc = T(1:3, 4)';
        end

        function loc = xyz(obj)
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
        
        function obj = set_end_effector(obj, xyz)
            % Inverse kinematics
            % xyz is a 1x3 matrix
            % TODO Figure out elbow up/down
            x = xyz(1);
            y = xyz(2);
            z = xyz(3);

            r=sqrt(x^2+y^2);
            h=sqrt(r^2+z^2);
            t=sqrt((0.15^2)-((h/2)^2));

            % Todo check this if it's beyond 90 deg
            thet1 = atan2(y,x);
            thet2up = atan2(z,r)+acos(h/0.3);
            thet2down = atan2(z,r)-acos(h/0.3);

            C3 = (((r^2)+(z^2)-(0.15^2)-(0.15^2))/(2*0.15*0.15));
            S3down = sqrt(1-C3^2);
            S3up = -S3down;
            thet_3up = rad2deg(atan2(S3up,C3));
            thet_3down = rad2deg(atan2(S3down,C3));

            obj.Theta1 = thet1;
            obj.Theta2 = thet2down;
            obj.Theta3 = thet_3down;
        end

        function R = z(obj, i)
            % Gives the z-rotation segment of the rotation matrix for the
            % grame
            if i == 0
                R = [0 0 1]';
            else    
                R = obj.transform(1, i);
                R = R(1:3, 3);
            end
        end

        function J = jacobian(obj)
            % Jv = zi-1 X (On - Oi-1)
            % Jw = Zi-1

            jv = @(i) cross(obj.z(i-1), obj.o(4) - obj.o(i-1));
            jw = @(i) obj.z(i-1);

            Jv = [jv(1) jv(2) jv(3) jv(4)];
            Jw = [jw(1) jw(2) jw(3) jw(4)];
            J = [Jv ; Jw];
        end
    end
end
