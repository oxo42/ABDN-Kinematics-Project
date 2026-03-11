%We define some geometry parameters% 

Dobot_L2 = 0.15;
Dobot_L3 = 0.15;

syms a alpha d theta theta1 theta2 theta3 theta4 L2 L3 XE YE ZE


%We define the position (X, Y, Z) of the end effector% 
%Using the DH method%

theta_1 = deg2rad(30); %[control:slider:090c]{"position":[19,21]}
theta_2 = deg2rad(30); %[control:slider:0e33]{"position":[19,21]}
theta_3 = deg2rad(20); %[control:slider:062c]{"position":[19,21]}
theta_4 = deg2rad(0); %[control:slider:97cf]{"position":[19,20]}


%DH table%


%Link 1 : (0, pi/2, 0, theta_1);
%Link 2 : (Dobot_L2, 0, 0, theta_2);
%Link 3 : (Dobot_L3, 0, 0, theta_3);
%Link p : (0, pi/2, 0, -theta_2 - theta_3);
%Link 4 : (0, 0, 0, theta_4); % d4 might be something

thetap=-theta2 - theta3;


alpha1=pi/2 %[output:9a64d10f]
alpha2=0 %[output:898ebaeb]
alpha3=0 %[output:6b829eef]
alphap=pi/2 %[output:2094c77a]
alpha4=0 %[output:84fecb9a]
d=0 %[output:21170a27]
a1=0 %[output:4f55ed4d]
a2=0.15 %[output:203c75ca]
a3=0.15 %[output:393252cd]
ap=0 %[output:4dd1b0c0]
a4=0 %[output:11571043]




 MA1 = [
        cos(theta1)  -1*sin(theta1)*cos(alpha1)    sin(theta1)*sin(alpha1)       a1*cos(theta1) ;
        sin(theta1)  cos(theta1)*cos(alpha1)       -1*cos(theta1)*sin(alpha1)    a1*sin(theta1) ; 
        0           sin(alpha1)                  cos(alpha1)                  d            ;
        0           0                           0                           1       
            ];

 MA2 = [
        cos(theta2)  -1*sin(theta2)*cos(alpha2)    sin(theta2)*sin(alpha2)       a2*cos(theta2) ;
        sin(theta2)  cos(theta2)*cos(alpha2)       -1*cos(theta2)*sin(alpha2)    a2*sin(theta2) ; 
        0           sin(alpha2)                  cos(alpha2)                  d            ;
        0           0                           0                           1       
            ];

  MA3 = [
        cos(theta3)  -1*sin(theta3)*cos(alpha3)    sin(theta3)*sin(alpha3)       a3*cos(theta3) ;
        sin(theta3)  cos(theta3)*cos(alpha3)       -1*cos(theta3)*sin(alpha3)    a3*sin(theta3) ; 
        0           sin(alpha3)                  cos(alpha3)                  d            ;
        0           0                           0                           1       
            ];

   MAp = [
        cos(thetap)  -1*sin(thetap)*cos(alphap)    sin(thetap)*sin(alphap)       ap*cos(thetap) ;
        sin(thetap)  cos(thetap)*cos(alphap)       -1*cos(thetap)*sin(alphap)    ap*sin(thetap) ; 
        0           sin(alphap)                  cos(alphap)                  d            ;
        0           0                           0                           1       
            ];

    MA4 = [
        cos(theta4)  -1*sin(theta4)*cos(alpha4)    sin(theta4)*sin(alpha4)       a4*cos(theta4) ;
        sin(theta4)  cos(theta4)*cos(alpha4)       -1*cos(theta4)*sin(alpha4)    a4*sin(theta4) ; 
        0           sin(alpha4)                  cos(alpha4)                  d            ;
        0           0                           0                           1       
            ];



Transmat = MA1*MA2*MA3*MAp*MA4; 

X_def = Transmat(1,4) %[output:6e28d34f]
Y_def = Transmat(2,4) %[output:72bf46cb]
Z_def = Transmat(3,4) %[output:87639ce7]

%[text] And I'm going to convert these symbolic expresions into MATLAB functions (say MLF):
XE_MLF = matlabFunction(X_def, 'Vars', [theta1, theta2, theta3]);
YE_MLF = matlabFunction(Y_def, 'Vars', [theta1, theta2, theta3]);
ZE_MLF = matlabFunction(Z_def, 'Vars', [theta2, theta3]);
%[text] 
%[text] 



%Forward kinematics

X=XE_MLF(theta_1, theta_2, theta_3) %[output:29c9626b]
Y=YE_MLF(theta_1, theta_2, theta_3) %[output:31ebeeb9]
Z=ZE_MLF(theta_2, theta_3) %[output:581ba1d0]


%Inverse kinematics

r=sqrt(X^2+Y^2) %[output:0a1035c0]
h=sqrt(r^2+Z^2) %[output:51138e77]
t=sqrt((0.15^2)-((h/2)^2)) %[output:58d58051]

thet1 = rad2deg(atan2(Y,X)) %[output:29e5bbe9]
thet2up = rad2deg(atan2(Z,r)+acos(h/0.3)) %[output:0d919e02]
thet2down = rad2deg(atan2(Z,r)-acos(h/0.3)) %[output:51d569f2]

C3 = (((r^2)+(Z^2)-(0.15^2)-(0.15^2))/(2*0.15*0.15));
S3down = sqrt(1-C3^2);
S3up = -S3down;
thet_3up = rad2deg(atan2(S3up,C3)) %[output:5b215b87]
thet_3down = rad2deg(atan2(S3down,C3)) %[output:8f3b7593]


%XE_EQ = XE == X_def
%YE_EQ = YE == Y_def
%ZE_EQ = ZE == Z_def

%S = solve([XE_EQ, YE_EQ, ZE_EQ], [theta1, theta2, theta3])


%Compute the Jacobian

Jacob = jacobian( [X_def, Y_def, Z_def],[theta1, theta2, theta3]  ) %[output:064c6f60]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":44.1}
%---
%[control:slider:090c]
%   data: {"defaultValue":0,"label":"theta1","max":100,"min":-100,"run":"Section","runOn":"ValueChanging","step":10}
%---
%[control:slider:0e33]
%   data: {"defaultValue":0,"label":"theta2","max":100,"min":-100,"run":"Section","runOn":"ValueChanging","step":10}
%---
%[control:slider:062c]
%   data: {"defaultValue":0,"label":"theta3","max":100,"min":-100,"run":"Section","runOn":"ValueChanging","step":10}
%---
%[control:slider:97cf]
%   data: {"defaultValue":0,"label":"theta4","max":100,"min":-100,"run":"Section","runOn":"ValueChanging","step":10}
%---
%[output:9a64d10f]
%   data: {"dataType":"textualVariable","outputData":{"name":"alpha1","value":"1.5708"}}
%---
%[output:898ebaeb]
%   data: {"dataType":"textualVariable","outputData":{"name":"alpha2","value":"0"}}
%---
%[output:6b829eef]
%   data: {"dataType":"textualVariable","outputData":{"name":"alpha3","value":"0"}}
%---
%[output:2094c77a]
%   data: {"dataType":"textualVariable","outputData":{"name":"alphap","value":"1.5708"}}
%---
%[output:84fecb9a]
%   data: {"dataType":"textualVariable","outputData":{"name":"alpha4","value":"0"}}
%---
%[output:21170a27]
%   data: {"dataType":"textualVariable","outputData":{"name":"d","value":"0"}}
%---
%[output:4f55ed4d]
%   data: {"dataType":"textualVariable","outputData":{"name":"a1","value":"0"}}
%---
%[output:203c75ca]
%   data: {"dataType":"textualVariable","outputData":{"name":"a2","value":"0.1500"}}
%---
%[output:393252cd]
%   data: {"dataType":"textualVariable","outputData":{"name":"a3","value":"0.1500"}}
%---
%[output:4dd1b0c0]
%   data: {"dataType":"textualVariable","outputData":{"name":"ap","value":"0"}}
%---
%[output:11571043]
%   data: {"dataType":"textualVariable","outputData":{"name":"a4","value":"0"}}
%---
%[output:6e28d34f]
%   data: {"dataType":"symbolic","outputData":{"name":"X_def","value":"\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,{\\left(\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)-\\frac{4967757600021511\\,\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{81129638414606681695789005144064}\\right)}}{20}+\\frac{3\\,\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)}{20}-\\frac{3\\,\\sin \\left(\\theta_3 \\right)\\,{\\left(\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)+\\frac{4967757600021511\\,\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)}{81129638414606681695789005144064}\\right)}}{20}-\\frac{14903272800064533\\,\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{1622592768292133633915780102881280}"}}
%---
%[output:72bf46cb]
%   data: {"dataType":"symbolic","outputData":{"name":"Y_def","value":"\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,{\\left(\\frac{4967757600021511\\,\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{81129638414606681695789005144064}+\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)\\right)}}{20}+\\frac{3\\,\\sin \\left(\\theta_3 \\right)\\,{\\left(\\frac{4967757600021511\\,\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)}{81129638414606681695789005144064}-\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)\\right)}}{20}+\\frac{14903272800064533\\,\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{1622592768292133633915780102881280}+\\frac{3\\,\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)}{20}"}}
%---
%[output:87639ce7]
%   data: {"dataType":"symbolic","outputData":{"name":"Z_def","value":"\\frac{3\\,\\sin \\left(\\theta_2 \\right)}{20}+\\frac{3\\,\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_3 \\right)}{20}+\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,\\sin \\left(\\theta_2 \\right)}{20}"}}
%---
%[output:29c9626b]
%   data: {"dataType":"textualVariable","outputData":{"name":"X","value":"0.1960"}}
%---
%[output:31ebeeb9]
%   data: {"dataType":"textualVariable","outputData":{"name":"Y","value":"0.1132"}}
%---
%[output:581ba1d0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Z","value":"0.1899"}}
%---
%[output:0a1035c0]
%   data: {"dataType":"textualVariable","outputData":{"name":"r","value":"0.2263"}}
%---
%[output:51138e77]
%   data: {"dataType":"textualVariable","outputData":{"name":"h","value":"0.2954"}}
%---
%[output:58d58051]
%   data: {"dataType":"textualVariable","outputData":{"name":"t","value":"0.0260"}}
%---
%[output:29e5bbe9]
%   data: {"dataType":"textualVariable","outputData":{"name":"thet1","value":"30.0000"}}
%---
%[output:0d919e02]
%   data: {"dataType":"textualVariable","outputData":{"name":"thet2up","value":"50.0000"}}
%---
%[output:51d569f2]
%   data: {"dataType":"textualVariable","outputData":{"name":"thet2down","value":"30.0000"}}
%---
%[output:5b215b87]
%   data: {"dataType":"textualVariable","outputData":{"name":"thet_3up","value":"-20.0000"}}
%---
%[output:8f3b7593]
%   data: {"dataType":"textualVariable","outputData":{"name":"thet_3down","value":"20.0000"}}
%---
%[output:064c6f60]
%   data: {"dataType":"symbolic","outputData":{"name":"Jacob","value":"\\begin{array}{l}\n\\left(\\begin{array}{ccc}\n-\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,\\sigma_8 }{20}-\\frac{3\\,\\sin \\left(\\theta_3 \\right)\\,\\sigma_9 }{20}-\\frac{14903272800064533\\,\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{1622592768292133633915780102881280}-\\frac{3\\,\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)}{20} & -\\sigma_6 -\\sigma_3 -\\frac{3\\,\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{20}-\\frac{14903272800064533\\,\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)}{1622592768292133633915780102881280} & -\\sigma_6 -\\sigma_3 \\\\\n\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,\\sigma_7 }{20}+\\frac{3\\,\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)}{20}-\\frac{3\\,\\sin \\left(\\theta_3 \\right)\\,\\sigma_{10} }{20}-\\frac{14903272800064533\\,\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{1622592768292133633915780102881280} & \\sigma_5 +\\frac{14903272800064533\\,\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)}{1622592768292133633915780102881280}-\\sigma_4 -\\frac{3\\,\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{20} & \\sigma_5 -\\sigma_4 \\\\\n0 & \\frac{3\\,\\cos \\left(\\theta_2 \\right)}{20}+\\sigma_2 -\\sigma_1  & \\sigma_2 -\\sigma_1 \n\\end{array}\\right)\\\\\n\\mathrm{}\\\\\n\\textrm{where}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_1 =\\frac{3\\,\\sin \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_3 \\right)}{20}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_2 =\\frac{3\\,\\cos \\left(\\theta_2 \\right)\\,\\cos \\left(\\theta_3 \\right)}{20}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_3 =\\frac{3\\,\\sin \\left(\\theta_3 \\right)\\,\\sigma_7 }{20}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_4 =\\frac{3\\,\\sin \\left(\\theta_3 \\right)\\,\\sigma_8 }{20}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_5 =\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,\\sigma_9 }{20}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_6 =\\frac{3\\,\\cos \\left(\\theta_3 \\right)\\,\\sigma_{10} }{20}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_7 =\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)-\\frac{4967757600021511\\,\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{81129638414606681695789005144064}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_8 =\\frac{4967757600021511\\,\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)}{81129638414606681695789005144064}+\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_9 =\\frac{4967757600021511\\,\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)}{81129638414606681695789005144064}-\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_{10} =\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)+\\frac{4967757600021511\\,\\cos \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_1 \\right)}{81129638414606681695789005144064}\n\\end{array}"}}
%---
