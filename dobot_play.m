r = dobot();
r.Theta2 = deg2rad(45);
r.xyz() %[output:2897a7a8]
r %[output:437789b3]
%%
s = dobot();
s.set_end_effector([0.2121 0 0.2121]) %[output:1a782043]
jacobian = matlabFunction(s.jacobian()) %[output:2aa526ac]
jacobian(s.Theta1, s.Theta2, s.Theta3) %[output:562b849c]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:2897a7a8]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"ans","rows":1,"type":"double","value":[["0.2121","0.0000","0.2121"]]}}
%---
%[output:437789b3]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0.7854\n    Theta3: 0\n    Theta4: 0\n"}}
%---
%[output:1a782043]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0.7680\n    Theta3: 1.9915\n    Theta4: 1\n"}}
%---
%[output:2aa526ac]
%   data: {"dataType":"textualVariable","outputData":{"header":"function_handle with value:","name":"jacobian","value":"    @(theta1,theta2,theta3)reshape([cos(theta3).*(cos(theta1).*sin(theta2).*6.123233995736766e-17+cos(theta2).*sin(theta1)).*(-3.0.\/2.0e+1)-sin(theta3).*(cos(theta1).*cos(theta2).*6.123233995736766e-17-sin(theta1).*sin(theta2)).*(3.0.\/2.0e+1)-cos(theta1).*sin(theta2).*9.184850993605149e-18-cos(theta2).*sin(theta1).*(3.0.\/2.0e+1),cos(theta3).*(cos(theta1).*cos(theta2)-sin(theta1).*sin(theta2).*6.123233995736766e-17).*(3.0.\/2.0e+1)+cos(theta1).*cos(theta2).*(3.0.\/2.0e+1)-sin(theta3).*(cos(theta1).*sin(theta2)+cos(theta2).*sin(theta1).*6.123233995736766e-17).*(3.0.\/2.0e+1)-sin(theta1).*sin(theta2).*9.184850993605149e-18,0.0,cos(theta3).*(cos(theta1).*sin(theta2)+cos(theta2).*sin(theta1).*6.123233995736766e-17).*(-3.0.\/2.0e+1)-sin(theta3).*(cos(theta1).*cos(theta2)-sin(theta1).*sin(theta2).*6.123233995736766e-17).*(3.0.\/2.0e+1)-cos(theta1).*sin(theta2).*(3.0.\/2.0e+1)-cos(theta2).*sin(theta1).*9.184850993605149e-18,cos(theta3).*(cos(theta1).*cos(theta2).*6.123233995736766e-17-sin(theta1).*sin(theta2)).*(3.0.\/2.0e+1)+cos(theta1).*cos(theta2).*9.184850993605149e-18-sin(theta3).*(cos(theta1).*sin(theta2).*6.123233995736766e-17+cos(theta2).*sin(theta1)).*(3.0.\/2.0e+1)-sin(theta1).*sin(theta2).*(3.0.\/2.0e+1),cos(theta2).*(3.0.\/2.0e+1)+cos(theta2).*cos(theta3).*(3.0.\/2.0e+1)-sin(theta2).*sin(theta3).*(3.0.\/2.0e+1),cos(theta3).*(cos(theta1).*sin(theta2)+cos(theta2).*sin(theta1).*6.123233995736766e-17).*(-3.0.\/2.0e+1)-sin(theta3).*(cos(theta1).*cos(theta2)-sin(theta1).*sin(theta2).*6.123233995736766e-17).*(3.0.\/2.0e+1),cos(theta3).*(cos(theta1).*cos(theta2).*6.123233995736766e-17-sin(theta1).*sin(theta2)).*(3.0.\/2.0e+1)-sin(theta3).*(cos(theta1).*sin(theta2).*6.123233995736766e-17+cos(theta2).*sin(theta1)).*(3.0.\/2.0e+1),cos(theta2).*cos(theta3).*(3.0.\/2.0e+1)-sin(theta2).*sin(theta3).*(3.0.\/2.0e+1)],[3,3])\n"}}
%---
%[output:562b849c]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"ans","rows":3,"type":"double","value":[["-0.0000","-0.1601","-0.0559"],["-0.0313","-0.0000","-0.0000"],["0","-0.0313","-0.1392"]]}}
%---
