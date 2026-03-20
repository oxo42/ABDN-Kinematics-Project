classdef dobot_tests < matlab.unittest.TestCase

    methods (TestClassSetup)
    end

    methods (TestMethodSetup)
    end

    methods (Test)
        function testConstructor(testCase)
            d = dobot(0.1, 0.2, 0.3, 0.4);
            testCase.verifyEqual(d.Theta1, 0.1);
            testCase.verifyEqual(d.Theta2, 0.2);
            testCase.verifyEqual(d.Theta3, 0.3);
            testCase.verifyEqual(d.Theta4, 0.4);
        end

        function testInverseKinematics(testCase)
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(-5);

            b = dobot();
            b.setEndEffector(a.xyz());
            testCase.verifyEqual(b.Theta1, a.Theta1, 'Theta1', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta2, a.Theta2, 'Theta2', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta3, a.Theta3, 'Theta3', AbsTol=1e-5)
        end

        function testInverseKinematicsElbowDown(testCase)
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(5);

            b = dobot();
            b.setEndEffector(a.xyz(), false);  % elbowUp = false
            testCase.verifyEqual(b.Theta1, a.Theta1, 'Theta1', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta2, a.Theta2, 'Theta2', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta3, a.Theta3, 'Theta3', AbsTol=1e-5)
        end

        function testTheta1GreaterThan90(testCase)
            a = dobot();
            a.Theta1 = deg2rad(135);

            b = dobot();
            b.setEndEffector(a.xyz());
            testCase.verifyEqual(b.Theta1, a.Theta1, 'Theta1', AbsTol=1e-5);
        end

        function testElbowUp(testCase)
            % verify that when we put the joints in an elbow up position the parameter reflects that
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(-5);
            testCase.assertTrue(a.elbowUp)

        end

        function testElbowDown(testCase)
            % verify that when we put the joints in an elbow down position the parameter reflects that
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(5);
            testCase.assertFalse(a.elbowUp)
        end

        function reachabilityTest(testCase)
            r = dobot();
            % Workspace is shifted by L1 vertically. 0.4 is too far (max reach 0.3)
            call = @() r.setEndEffector([0.4 0 0]);
            testCase.verifyError(call, '');
        end

        function testSingularity(testCase)
            % Test fully extended configuration (theta3 = 0)
            d = dobot(0, 0, 0, 0);
            testCase.assertTrue(d.isSingular(), 'Should be singular when fully extended');
        end

        function testJacobianConsistency(testCase)
            % Compare geometric and analytical Jacobians for linear velocity
            d = dobot(deg2rad(10), deg2rad(20), deg2rad(-5), 0);
            J_geo = d.jacobian();
            J_ana = d.analyticalJacobian();
            
            % Linear velocity parts (first 3 rows) should match
            testCase.verifyEqual(J_geo(1:3, :), J_ana(1:3, :), 'AbsTol', 1e-8);
        end

        function testPassiveJointOrientation(testCase)
            % The passive joint ensures the end effector orientation relative to the 
            % base depends only on Theta1 and Theta4, not the linkage configuration.
            d1 = dobot(deg2rad(10), deg2rad(10), deg2rad(-10), deg2rad(5));
            d2 = dobot(deg2rad(10), deg2rad(30), deg2rad(5), deg2rad(5));
            
            T1 = d1.transform(1, 4);
            T2 = d2.transform(1, 4);
            
            % Rotation matrices should be identical despite different shoulder/elbow angles
            testCase.verifyEqual(T1(1:3, 1:3), T2(1:3, 1:3), 'AbsTol', 1e-8);
        end

        function testBoundaryReachability(testCase)
            d = dobot();
            l2 = d.L2;
            l3 = d.L3;
            l1 = d.L1;
            max_reach = l2 + l3;

            % Test exactly at the limit - should not throw error
            % At max horizontal reach, z should be l1
            d.setEndEffector([max_reach, 0, l1]);
            testCase.verifyEqual(d.xyz(), [max_reach, 0, l1], 'AbsTol', 1e-7);
        end

        function testJointLimitsValid(testCase)
            d = dobot();
            % Test valid joints
            thetas = [0.5, 0.5, 0.1, 0.5];
            d.setJointAngles(thetas);
            testCase.verifyEqual([d.Theta1, d.Theta2, d.Theta3, d.Theta4], thetas);
        end

        function testJointLimitsInvalid(testCase)
            d = dobot();
            % Test limits for each joint
            limits = d.JointLimits;

            % Joint 1 too low
            call = @() d.setJointAngles([limits(1,1)-0.1, 0, 0, 0]);
            testCase.verifyError(call, '');

            % Joint 2 too high
            call = @() d.setJointAngles([0, limits(2,2)+0.1, 0, 0]);
            testCase.verifyError(call, '');

            % Joint 3 too low
            call = @() d.setJointAngles([0, 0, limits(3,1)-0.1, 0]);
            testCase.verifyError(call, '');

            % Joint 4 too high
            call = @() d.setJointAngles([0, 0, 0, limits(4,2)+0.1]);
            testCase.verifyError(call, '');
        end

        function testIKJointLimits(testCase)
            d = dobot();
            % A point that might be reachable but violates joint limits
            % Target far behind the robot (Joint 1 limit is 135 deg)
            target = [-0.2, 0, 0.1]; % Behind base
            call = @() d.setEndEffector(target);
            testCase.verifyError(call, '');
        end

        function testValidateJointLimitsValid(testCase)
            d = dobot();
            % Test valid joints explicitly calling validateJointLimits
            thetas = [0.5, 0.5, 0.1, 0.5];
            % Should not throw an error
            d.validateJointLimits(thetas);
        end

        function testValidateJointLimitsInvalid(testCase)
            d = dobot();
            limits = d.JointLimits;

            % Joint 1 too low
            call = @() d.validateJointLimits([limits(1,1)-0.1, 0, 0, 0]);
            testCase.verifyError(call, '');
            
            % Joint 2 too high
            call = @() d.validateJointLimits([0, limits(2,2)+0.1, 0, 0]);
            testCase.verifyError(call, '');
            
            % Joint 3 too low
            call = @() d.validateJointLimits([0, 0, limits(3,1)-0.1, 0]);
            testCase.verifyError(call, '');
            
            % Joint 4 too high
            call = @() d.validateJointLimits([0, 0, 0, limits(4,2)+0.1]);
            testCase.verifyError(call, '');
        end

        function testValidateJointLimitsNoArgs(testCase)
            d = dobot();
            % Default angles are 0, which are valid
            d.validateJointLimits();

            % Set an invalid angle directly (bypassing setJointAngles)
            limits = d.JointLimits;
            d.Theta1 = limits(1,1) - 0.1;
            call = @() d.validateJointLimits();
            testCase.verifyError(call, '');
        end

        function testSetJointAngles(testCase)
            d = dobot();
            thetas = [0.1, 0.2, -0.1, 0.4];
            d.setJointAngles(thetas);
            testCase.verifyEqual(d.Theta1, 0.1);
            testCase.verifyEqual(d.Theta2, 0.2);
            testCase.verifyEqual(d.Theta3, -0.1);
            testCase.verifyEqual(d.Theta4, 0.4);
            
            % Check that it errors on invalid angles
            invalid_thetas = [5, 0, 0, 0]; % Theta1 limit is ~2.35
            call = @() d.setJointAngles(invalid_thetas);
            testCase.verifyError(call, '');
        end

        function testSingularityConditions(testCase)
            % Condition 1: sin(theta3) == 0
            d = dobot(0, 0, 0, 0); % theta3 = 0
            [cond1, cond2] = d.singularityConditions();
            testCase.assertTrue(cond1);
            testCase.assertFalse(cond2); % arm is outstretched, r is not 0
            
            % Condition 2: L2*cos(theta2) + L3*cos(theta2+theta3) == 0 (wrist above base)
            % Find angles where r = 0.
            % L2 = L3 = 0.15. If theta2 = pi/2 and theta3 = 0, cos(pi/2)=0.
            d = dobot(0, pi/2, 0, 0); 
            [cond1, cond2] = d.singularityConditions();
            testCase.assertTrue(cond1); % sin(0) = 0
            testCase.assertTrue(cond2); % r = 0

            % Not singular
            d = dobot(0, 0.1, -0.1, 0);
            [cond1, cond2] = d.singularityConditions();
            testCase.assertFalse(cond1);
            testCase.assertFalse(cond2);
        end

        function testValidateMass(testCase)
            d = dobot();
            % Valid mass
            d.validateMass(1.5, 'TestMass');
            d.validateMass(0, 'TestMass');
            
            % Invalid mass
            testCase.verifyError(@() d.validateMass(-1, 'TestMass'), 'dobot:InvalidMass');
            testCase.verifyError(@() d.validateMass([], 'TestMass'), 'dobot:InvalidMass');
            testCase.verifyError(@() d.validateMass([1 2], 'TestMass'), 'dobot:InvalidMass');
            testCase.verifyError(@() d.validateMass(NaN, 'TestMass'), 'dobot:InvalidMass');
        end

        function testParseDynamicState(testCase)
            d = dobot(0.1, 0.2, 0.3, 0.4);
            
            % Empty 'q' should return current joint angles
            q_default = d.parseDynamicState([], 'q');
            testCase.verifyEqual(q_default, [0.1; 0.2; 0.3; 0.4]);
            
            % Empty 'qd' or 'qdd' should return zeros
            qd_default = d.parseDynamicState([], 'qd');
            testCase.verifyEqual(qd_default, zeros(4,1));
            
            % Valid state
            state = d.parseDynamicState([1, 2, 3, 4], 'q');
            testCase.verifyEqual(state, [1; 2; 3; 4]);
            
            % Invalid state length
            testCase.verifyError(@() d.parseDynamicState([1, 2, 3], 'q'), 'dobot:InvalidDynamicState');
        end

        function testGravityVector(testCase)
            d = dobot();
            d.M1 = 1; d.M2 = 1; d.M3 = 1;
            
            % When pointing straight out horizontally (theta2=0, theta3=0)
            % Gravity should exert maximum torque on joint 2
            q = [0; 0; 0; 0];
            G = d.gravityVector(q);
            
            % Expected gravity on joint 2 (supporting M1, M2, M3)
            % G2 = g * ( (m1*lc2 + (m2+m3)*L2) + (m2*lc3 + m3*L3) )
            % lc2=0.075, L2=0.15, lc3=0.075, L3=0.15
            expected_G2 = 9.81 * ( (1*0.075 + 2*0.15) + (1*0.075 + 1*0.15) );
            testCase.verifyEqual(G(2), expected_G2, 'AbsTol', 1e-6);
            
            % Expected gravity on joint 3 (supporting M2, M3)
            % G3 = g * (m2*lc3 + m3*L3)
            expected_G3 = 9.81 * (1*0.075 + 1*0.15);
            testCase.verifyEqual(G(3), expected_G3, 'AbsTol', 1e-6);
            
            % Joint 1 and 4 are vertical axes, gravity should not affect them
            testCase.verifyEqual(G(1), 0);
            testCase.verifyEqual(G(4), 0);
        end

        function testInertiaMatrixProperties(testCase)
            d = dobot();
            q = [0.1; 0.2; 0.3; 0.4];
            M = d.inertiaMatrix(q);
            
            % Inertia matrix should be 4x4
            testCase.verifyEqual(size(M), [4, 4]);
            
            % Inertia matrix should be symmetric
            testCase.verifyEqual(M, M', 'AbsTol', 1e-10);
            
            % Inertia matrix should be positive definite (all eigenvalues > 0)
            eigenvalues = eig(M);
            testCase.assertTrue(all(eigenvalues > 0), 'Inertia matrix must be positive definite');
            
            % Base and wrist rotate independently of shoulder/elbow in this model
            testCase.verifyEqual(M(1,2), 0);
            testCase.verifyEqual(M(1,3), 0);
        end

        function testCoriolisVectorZeros(testCase)
            d = dobot();
            q = [0.1; 0.2; 0.3; 0.4];
            
            % If velocities are zero, coriolis vector must be zero
            qd = [0; 0; 0; 0];
            Cqd = d.coriolisVector(q, qd);
            testCase.verifyEqual(Cqd, zeros(4,1));
        end

        function testInverseDynamicsStatic(testCase)
            d = dobot(0.1, 0.2, 0.3, 0.4);
            
            % In a static case (velocity=0, acceleration=0)
            % Torques should exactly equal the gravity vector
            qd = zeros(4,1);
            qdd = zeros(4,1);
            
            tau = d.inverseDynamics(qd, qdd);
            G = d.gravityVector(); % Uses current angles by default
            
            testCase.verifyEqual(tau, G, 'AbsTol', 1e-10);
        end

        function testInverseDynamicsWithTerms(testCase)
             d = dobot(0.1, 0.2, 0.3, 0.4);
             qd = [0.1; 0.2; -0.1; 0];
             qdd = [1; -1; 0.5; 0];
             
             [tau, terms] = d.inverseDynamics(qd, qdd);
             
             % Reconstruct tau from terms to verify consistency
             tau_reconstructed = terms.M * terms.qdd + terms.coriolis + terms.gravity;
             testCase.verifyEqual(tau, tau_reconstructed, 'AbsTol', 1e-10);
             
             % Verify term sizes
             testCase.verifyEqual(size(terms.M), [4,4]);
             testCase.verifyEqual(size(terms.coriolis), [4,1]);
             testCase.verifyEqual(size(terms.gravity), [4,1]);
        end
    end
end
