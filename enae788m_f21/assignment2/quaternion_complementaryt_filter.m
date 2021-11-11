syms q0 qx qy qz real

R_q = [2*q0^2+2*qx^2-1,     2*qx*qy-2*q0*qz,         2*qx*qz+2*q0*qy;
       2*qx*qy+2*q0*qz,       2*q0^2+2*qy^2-1,       2*qy*qz-2*q0*qx;
       2*qx*qz-2*q0*qy,       2*qy*qz+2*q0*qx,         2*q0^2+2*qz^2-1
       ]'
% gravity 
m = [0, 0, 1]
%magetic vector
syms n1 n2 n3 real
n = [n1, n2, n3]

y = [R_q*m'; R_q*n']

X = [diff(y, q0), diff(y, qx), diff(y, qy), diff(y, qz)]


[                       -2*qy,                        2*qz,                       -2*q0,                        2*qx]
[                        2*qx,                        2*q0,                        2*qz,                        2*qy]
[                        4*q0,                           0,                           0,                        4*qz]
[ 4*n1*q0 - 2*n3*qy + 2*n2*qz, 4*n1*qx + 2*n2*qy + 2*n3*qz,           2*n2*qx - 2*n3*q0,           2*n2*q0 + 2*n3*qx]
[ 4*n2*q0 + 2*n3*qx - 2*n1*qz,           2*n3*q0 + 2*n1*qy, 2*n1*qx + 4*n2*qy + 2*n3*qz,           2*n3*qy - 2*n1*q0]
[ 4*n3*q0 - 2*n2*qx + 2*n1*qy,           2*n1*qz - 2*n2*q0,           2*n1*q0 + 2*n2*qz, 2*n1*qx + 2*n2*qy + 4*n3*qz]