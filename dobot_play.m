r = dobot();
r.Theta2 = deg2rad(45);
r.Theta1 = deg2rad(30);
r.xyz() %[output:2897a7a8]
transformation = r.transform(1, 4) %[output:1727b8ff]
r %[output:437789b3]
%%
s = dobot();
s.set_end_effector(r.xyz()) %[output:0ae81d76]
s.jacobian() %[output:89882506] %[output:07d4513a]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:2897a7a8]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"ans","rows":1,"type":"double","value":[["0.1837","0.1061","0.2121"]]}}
%---
%[output:1727b8ff]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"transformation","rows":4,"type":"double","value":[["0.8660","0.5000","0.0000","0.1837"],["0.5000","-0.8660","-0.0000","0.1061"],["0","0.0000","-1.0000","0.2121"],["0","0","0","1.0000"]]}}
%---
%[output:437789b3]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0.5236\n    Theta2: 0.7854\n    Theta3: 0\n    Theta4: 0\n"}}
%---
%[output:0ae81d76]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0.5236\n    Theta2: 0.7854\n    Theta3: 1.7075e-06\n    Theta4: 0\n"}}
%---
%[output:89882506]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"J","rows":6,"type":"double","value":[["0","0","0","0"],["0","0","0","0"],["0","0","0","0"],["0","0","0","0"],["0","0","0","0"],["0","0","0","0"]]}}
%---
%[output:07d4513a]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ans","rows":6,"type":"double","value":[["0","0","0","0"],["0","0","0","0"],["0","0","0","0"],["0","0","0","0"],["0","0","0","0"],["0","0","0","0"]]}}
%---
