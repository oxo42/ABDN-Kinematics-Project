r = dobot();
r.Theta2 = deg2rad(45);
r.Theta1 = deg2rad(30);
xyz = r.xyz() %[output:9f1b99b7]
T = r.transform(1, 4) %[output:6e6cfe96]
T1 = r.transform(1, 2) %[output:0fec495d]
r %[output:891bc6b2]
%%
s = dobot();
s.set_end_effector(r.xyz()) %[output:4f4161be]
s.jacobian() %[output:126d382b] %[output:036cfc5e] %[output:6e4b3ea1] %[output:39820a78]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:9f1b99b7]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"xyz","rows":1,"type":"double","value":[["0.1837","0.1061","0.2121"]]}}
%---
%[output:6e6cfe96]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"T","rows":4,"type":"double","value":[["0.8660","0.5000","0.0000","0.1837"],["0.5000","-0.8660","-0.0000","0.1061"],["0","0.0000","-1.0000","0.2121"],["0","0","0","1.0000"]]}}
%---
%[output:0fec495d]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"T1","rows":4,"type":"double","value":[["0.6124","-0.6124","0.5000","0.0919"],["0.3536","-0.3536","-0.8660","0.0530"],["0.7071","0.7071","0.0000","0.1061"],["0","0","0","1.0000"]]}}
%---
%[output:891bc6b2]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0.5236\n    Theta2: 0.7854\n    Theta3: 0\n    Theta4: 0\n"}}
%---
%[output:4f4161be]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"  <a href=\"matlab:helpPopup('dobot')\" style=\"font-weight:bold\">dobot<\/a> with properties:\n\n    Theta1: 0.5236\n    Theta2: 0.7854\n    Theta3: 1.7075e-06\n    Theta4: 0\n"}}
%---
%[output:126d382b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"jv1","rows":3,"type":"double","value":[["-0.1061"],["0.1837"],["0"]]}}
%---
%[output:036cfc5e]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"Jv","rows":3,"type":"double","value":[["-0.1061","-0.1837","-0.0919","0"],["0.1837","-0.1061","-0.0530","0"],["0","0.2121","0.1061","0"]]}}
%---
%[output:6e4b3ea1]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"Jw","rows":3,"type":"double","value":[["0","0.5000","0.5000","0.5000"],["0","-0.8660","-0.8660","-0.8660"],["1.0000","0.0000","0.0000","0.0000"]]}}
%---
%[output:39820a78]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ans","rows":6,"type":"double","value":[["-0.1061","-0.1837","-0.0919","0"],["0.1837","-0.1061","-0.0530","0"],["0","0.2121","0.1061","0"],["0","0.5000","0.5000","0.5000"],["0","-0.8660","-0.8660","-0.8660"],["1.0000","0.0000","0.0000","0.0000"]]}}
%---
