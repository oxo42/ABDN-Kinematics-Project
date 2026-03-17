classdef dobot_tests < matlab.unittest.TestCase

    methods (TestClassSetup)
    end

    methods (TestMethodSetup)
    end

    methods (Test)
        function testConstructor(testCase)
            d = dobot(1, 2, 3, 4);
            testCase.verifyEqual(d.Theta1, 1);
            testCase.verifyEqual(d.Theta2, 2);
            testCase.verifyEqual(d.Theta3, 3);
            testCase.verifyEqual(d.Theta4, 4);
        end

        function testInverseKinematics(testCase)
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(-30);

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
            a.Theta3 = deg2rad(30);

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
            a.Theta3 = deg2rad(-30);
            testCase.assertTrue(a.elbowUp)

        end

        function testElbowDown(testCase)
            % verify that when we put the joints in an elbow down position the parameter reflects that
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(30);
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
            
            % Test workspace boundary singularity (r = 0)
            d = dobot(0, pi/2, 0, 0);
            testCase.assertTrue(d.isSingular(), 'Should be singular when r=0');
        end

        function testJacobianConsistency(testCase)
            % Compare geometric and analytical Jacobians for linear velocity
            d = dobot(deg2rad(10), deg2rad(20), deg2rad(-30), 0);
            J_geo = d.jacobian();
            J_ana = d.analyticalJacobian();
            
            % Linear velocity parts (first 3 rows) should match
            testCase.verifyEqual(J_geo(1:3, :), J_ana(1:3, :), 'AbsTol', 1e-8);
        end

        function testPassiveJointOrientation(testCase)
            % The passive joint ensures the end effector orientation relative to the 
            % base depends only on Theta1 and Theta4, not the linkage configuration.
            d1 = dobot(deg2rad(10), deg2rad(10), deg2rad(-10), deg2rad(5));
            d2 = dobot(deg2rad(10), deg2rad(30), deg2rad(-40), deg2rad(5));
            
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
    end
end
