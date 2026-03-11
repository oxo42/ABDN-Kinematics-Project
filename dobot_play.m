r = dobot();
r.Theta2 = deg2rad(45);
r.xyz() %[output:762860d0]
r %[output:584013ce]
%%
s = dobot();
s.set_end_effector([0.2121 0 0.2121]);
s %[output:9afc30ac]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:762860d0]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"ans","rows":1,"type":"double","value":[["0.2121","0.0000","0.2121"]]}}
%---
%[output:584013ce]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0.7854\n    Theta3: 0\n    Theta4: 0\n"}}
%---
%[output:9afc30ac]
%   data: {"dataType":"textualVariable","outputData":{"name":"s","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0\n    Theta2: 0.7680\n    Theta3: 1.9915\n    Theta4: 1\n"}}
%---
