function [T04,T02] = dml(theta1, theta2, theta3, theta4)
%UNTITLED For the given input angles will return the output transformation
%matrices
%   T04 is the end effector
%   T02 is the location of Joint 2
arguments (Input)
    theta1
    theta2
    theta3
    theta4
end

arguments (Output)
    T04
    T02
end
  %                                     a, alpha, d, theta
  a1 = dh_link_to_transformation_matrix(0, pi/2, 0, theta1);
  a2 = dh_link_to_transformation_matrix(0.15, 0, 0, theta2);
  a3 = dh_link_to_transformation_matrix(0.15, 0, 0, theta3);
  % passive link to keep the end effector parallel to the ground
  % Comment this to see the effect on the orientation of the end effector
  % Note the alpha rotation into the end effector frame is here
  ap = dh_link_to_transformation_matrix(0, pi/2, 0, - theta2 - theta3); 
  a4 = dh_link_to_transformation_matrix(0, 0, 0, theta4); 
  T02 = a1 * a2;
  T04 = a1 * a2 * a3 * ap * a4;
end
