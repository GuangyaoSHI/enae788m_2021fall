%transition function
syms phi theta real
syms Ts real
syms wx wy wz real
%state x = [phi, theta]
d_x = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
       0, cos(phi), -sin(phi)];
f = [phi, theta]'+ Ts*d_x