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
            b.set_end_effector(a.xyz());
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
            b.set_end_effector(a.xyz(), false);  % elbowUp = false
            testCase.verifyEqual(b.Theta1, a.Theta1, 'Theta1', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta2, a.Theta2, 'Theta2', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta3, a.Theta3, 'Theta3', AbsTol=1e-5)
        end

        function testTheta1GreaterThan90(testCase)
            a = dobot();
            a.Theta1 = deg2rad(135);

            b = dobot();
            b.set_end_effector(a.xyz());
            testCase.verifyEqual(b.Theta1, a.Theta1, 'Theta1', AbsTol=1e-5);
        end

        function testElbowUp(testCase)
            % verify that when we put the joins in an elbow up position the parameter reflects that
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(-30);
            testCase.assertTrue(a.elbowUp)

        end

        function testElbowDown(testCase)
            % verify that when we put the joins in an elbow down position the parameter reflects that
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(30);
            testCase.assertFalse(a.elbowUp)
        end

        function reachabilityTest(testCase)
            r = dobot();
            call = @() r.set_end_effector([0.4 0 0]);
            testCase.verifyError(call, '');
        end
    end
end
