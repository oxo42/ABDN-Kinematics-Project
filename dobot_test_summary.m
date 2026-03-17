diary('test_log.txt');  % Start saving output

disp('===== DOBOT PROJECT TEST SUMMARY =====')

clear classes; clear; clc

%% 1. Constructor Test
disp('--- Constructor Test ---')
r = dobot()

%% 2. Forward Kinematics Test
disp('--- Forward Kinematics ---')
r.Theta1 = deg2rad(30);
r.Theta2 = deg2rad(45);
r.Theta3 = 0;
r.Theta4 = 0;

xyz = r.xyz()

T = r.transform(1,4)

%% 3. Inverse Kinematics Test
disp('--- Inverse Kinematics ---')
s = dobot();
s = s.setEndEffector(xyz);

angles = [s.Theta1 s.Theta2 s.Theta3 s.Theta4]

%% 4. Jacobian Test
disp('--- Jacobian ---')
J = r.jacobian()

Ja = r.analyticalJacobian()

%% 5. Singularity Test
disp('--- Singularity Test ---')
detJa = r.singularityDet()
tf = r.isSingular()

%% 6. Unit Test Suite
disp('--- Unit Tests ---')
results = runtests('dobot_tests.m');
table(results)

disp('===== TESTING COMPLETE =====')

diary off  % Stop saving