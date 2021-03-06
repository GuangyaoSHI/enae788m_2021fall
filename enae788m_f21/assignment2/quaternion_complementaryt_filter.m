%symbolic computation
syms q0 qx qy qz real

%from body to the local world
R_q = [2*q0^2+2*qx^2-1,     2*qx*qy-2*q0*qz,         2*qx*qz+2*q0*qy;
       2*qx*qy+2*q0*qz,       2*q0^2+2*qy^2-1,       2*qy*qz-2*q0*qx;
       2*qx*qz-2*q0*qy,       2*qy*qz+2*q0*qx,         2*q0^2+2*qz^2-1
       ]
% gravity 
m = [0, 0, 1]
%magetic vector
syms n1 n2 n3 real
n = [n1, n2, n3]

y = [R_q'*m'; R_q'*n']

X = [diff(y, q0), diff(y, qx), diff(y, qy), diff(y, qz)]



%compute formulas for yaw
syms phi theta yaw real
syms mx0 my0 mz0 real
syms mx my mz real
syms c s


eqns = [ cos(theta)*c*mx0 + (sin(phi)*sin(theta)*c-cos(phi)*s)*my0 +(cos(phi)*sin(theta)*c+sin(phi)*s)*mz0 == mx, ...
         cos(theta)*s*mx0 + (sin(phi)*sin(theta)*s+cos(phi)*c)*my0 + (cos(phi)*sin(theta)*s-sin(phi)*c)*mz0 == my
         ];

 eq1 = cos(theta)*c*mx0 + (sin(phi)*sin(theta)*c-cos(phi)*s)*my0 +(cos(phi)*sin(theta)*c+sin(phi)*s)*mz0;
 eq2 = cos(theta)*s*mx0 + (sin(phi)*sin(theta)*s+cos(phi)*c)*my0 + (cos(phi)*sin(theta)*s-sin(phi)*c)*mz0;
 diff(eq1, s)
 diff(eq1, c)
 diff(eq2, s)
 diff(eq2, c)
 S = solve(eqns, [c, s])