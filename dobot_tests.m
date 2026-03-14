classdef dobot_tests < matlab.unittest.TestCase

    methods (TestClassSetup)
        % Shared setup for the entire test class
    end

    methods (TestMethodSetup)
        % Setup for each test
    end

    methods (Test)
        % Test methods

        function testInverseKinematics(testCase)
            a = dobot();
            a.Theta1 = deg2rad(45);
            a.Theta2 = deg2rad(40);
            a.Theta3 = deg2rad(30);
            a.transform(1,3)

            b = dobot();
            b.set_end_effector(a.xyz());
            b.transform(1,3)
            testCase.verifyEqual(b.Theta1, a.Theta1, 'Theta1', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta2, a.Theta2, 'Theta2', AbsTol=1e-5)
            testCase.verifyEqual(b.Theta3, a.Theta3, 'Theta3', AbsTol=1e-5)
        end
    end

end
