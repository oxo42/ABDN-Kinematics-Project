r = dobot();
r.Theta2 = deg2rad(45);
r.xyz() %[output:80354e34]
r %[output:9fc53968]
%%
s = dobot();
s.set_end_effector([0.2121 0 0.2121]) %[output:416fe7d2]
s %[output:23810a2b]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:80354e34]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"ans","rows":1,"type":"double","value":[["0.2121","0.0000","0.2121"]]}}
%---
%[output:9fc53968]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0.7854\n    Theta3: 0\n    Theta4: 0\n"}}
%---
%[output:416fe7d2]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0.7680\n    Theta3: 1.9915\n    Theta4: 1\n"}}
%---
%[output:23810a2b]
%   data: {"dataType":"textualVariable","outputData":{"name":"s","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0\n    Theta3: 0\n    Theta4: 0\n"}}
%---
