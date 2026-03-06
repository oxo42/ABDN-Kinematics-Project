function matrix = dh_link_to_transformation_matrix(a, alpha, d, theta)
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