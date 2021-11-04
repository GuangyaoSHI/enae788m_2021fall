%compute the symbolic state equation

%the orientation at the beginning 
%syms qx_0 qy_0 qz_0 qw_0 real
%quaternion at time step t
syms qx_t qy_t qz_t qw_t real

%compute the rotation quaternion from q_0 to q_t
%q_t * q_0^-1
% S_q0 = [0, -qz_0, qy_0; qz_0, 0, -qx_0; -qy_0, qx_0, 0]; 
% q = [qw_0*eye(3)-S_q0, -[qx_0;qy_0;qz_0]; 
%      [qx_0;qy_0;qz_0]', qw_0 
% ] * [qx_t, qy_t, qz_t, qw_t]'


%test 
%eul2quat([pi/4, pi/3, pi/5])
% qx_0 = 0, qy_0 = 0, qz_0=0, qw_0=1;
% qx_t = 0.1724458, qy_t = 0.54174325, qz_t = 0.06526868, qw_t = 0.82007115
% eval(subs(q))
q = [qx_t, qy_t, qz_t, qw_t];

%compute rotation matix from q_0 to q_t
R_q = [2*q(4)^2+2*q(1)^2-1,     2*q(1)*q(2)-2*q(4)*q(3),   2*q(1)*q(3)+2*q(4)*q(2);
       2*q(1)*q(2)+2*q(4)*q(3), 2*q(4)^2+2*q(2)^2-1,       2*q(2)*q(3)-2*q(4)*q(1);
       2*q(1)*q(3)-2*q(4)*q(2), 2*q(2)*q(3)+2*q(4)*q(1),   2*q(4)^2+2*q(3)^2-1
       ]

%compute rate of quaternion with angular velocity
%complete matrix
% Q_qt = [qx_t, qw_t, -qz_t, qy_t;
%         qy_t, qz_t, qw_t, -qx_t;
%         qz_t, -qy_t, qx_t, qw_t;
%         qw_t, -qx_t, -qy_t, -qz_t;
% ]

%https://physics.stackexchange.com/questions/88398/why-is-body-frame-angular-velocity-nonzero
%https://robotics.stackexchange.com/questions/16266/angular-velocity-output-of-imu
%https://math.stackexchange.com/questions/1832019/what-is-the-angular-velocity-in-an-inertial-frame-given-the-angular-velocity-in

%partial matrix
%need to consider the transformation from body frame angular velocity to 
%inertial frame
Q_qt = [qw_t, -qz_t, qy_t;
        qz_t, qw_t, -qx_t;
        -qy_t, qx_t, qw_t;
        -qx_t, -qy_t, -qz_t;
]


%position at time step t
syms px py pz real

%sample time
syms t_s real

%accelearation input in body frame
syms ax ay az real
a=[ax, ay, az]

%angular velocity state measured in body frame
syms wx wy wz real

%angular noise input 
syms vx vy vz real



%state = [p, a, q, w]
f = [[px, py, pz]'+1/2*t_s^2*R_q'*([ax, ay, az]'); ...
     [ax, ay, az]'; ...
     [qx_t, qy_t, qz_t, qw_t]'+1/2*Q_qt*([wx, wy, wz]'); ...
     [wx, wy, wz]'
]



%initial meg vector in the local world frame
syms mx_0 my_0 mz_0 real

m0 = [mx_0, my_0, mz_0];
%observation
h = [a'; q'; R_q*m0']

% syms x y z
% f = [2*x+3*y+z, 5*x+10*y+3*z]'
% df = diff(f, x)


%diff with respect to states
F = [diff(f, px), diff(f, py), diff(f, pz), ...
    diff(f, ax), diff(f, ay), diff(f, az), ...
    diff(f, qx_t), diff(f, qy_t), ...
    diff(f, qz_t), diff(f, qw_t), ...
    diff(f, wx), diff(f, wy), diff(f, wz)
    ]


[ 1, 0, 0, (t_s^2*(2*qw_t^2 + 2*qx_t^2 - 1))/2,       t_s^2*(qw_t*qz_t + qx_t*qy_t),      -t_s^2*(qw_t*qy_t - qx_t*qz_t), t_s^2*(2*ax*qx_t + ay*qy_t + az*qz_t),             t_s^2*(ay*qx_t - az*qw_t),             t_s^2*(ay*qw_t + az*qx_t), t_s^2*(2*ax*qw_t + ay*qz_t - az*qy_t),       0,       0,       0]
[ 0, 1, 0,      -t_s^2*(qw_t*qz_t - qx_t*qy_t), (t_s^2*(2*qw_t^2 + 2*qy_t^2 - 1))/2,       t_s^2*(qw_t*qx_t + qy_t*qz_t),             t_s^2*(ax*qy_t + az*qw_t), t_s^2*(ax*qx_t + 2*ay*qy_t + az*qz_t),            -t_s^2*(ax*qw_t - az*qy_t), t_s^2*(2*ay*qw_t - ax*qz_t + az*qx_t),       0,       0,       0]
[ 0, 0, 1,       t_s^2*(qw_t*qy_t + qx_t*qz_t),      -t_s^2*(qw_t*qx_t - qy_t*qz_t), (t_s^2*(2*qw_t^2 + 2*qz_t^2 - 1))/2,            -t_s^2*(ay*qw_t - ax*qz_t),             t_s^2*(ax*qw_t + ay*qz_t), t_s^2*(ax*qx_t + ay*qy_t + 2*az*qz_t), t_s^2*(ax*qy_t - ay*qx_t + 2*az*qw_t),       0,       0,       0]
[ 0, 0, 0,                                   1,                                   0,                                   0,                                     0,                                     0,                                     0,                                     0,       0,       0,       0]
[ 0, 0, 0,                                   0,                                   1,                                   0,                                     0,                                     0,                                     0,                                     0,       0,       0,       0]
[ 0, 0, 0,                                   0,                                   0,                                   1,                                     0,                                     0,                                     0,                                     0,       0,       0,       0]
[ 0, 0, 0,                                   0,                                   0,                                   0,                                     1,                                  wz/2,                                 -wy/2,                                  wx/2,  qw_t/2, -qz_t/2,  qy_t/2]
[ 0, 0, 0,                                   0,                                   0,                                   0,                                 -wz/2,                                     1,                                  wx/2,                                  wy/2,  qz_t/2,  qw_t/2, -qx_t/2]
[ 0, 0, 0,                                   0,                                   0,                                   0,                                  wy/2,                                 -wx/2,                                     1,                                  wz/2, -qy_t/2,  qx_t/2,  qw_t/2]
[ 0, 0, 0,                                   0,                                   0,                                   0,                                 -wx/2,                                 -wy/2,                                 -wz/2,                                     1, -qx_t/2, -qy_t/2, -qz_t/2]
[ 0, 0, 0,                                   0,                                   0,                                   0,                                     0,                                     0,                                     0,                                     0,       1,       0,       0]
[ 0, 0, 0,                                   0,                                   0,                                   0,                                     0,                                     0,                                     0,                                     0,       0,       1,       0]


H = [diff(h, px), diff(h, py), diff(h, pz), ...
    diff(h, ax), diff(h, ay), diff(h, az), ...
    diff(h, qx_t), diff(h, qy_t), ...
    diff(h, qz_t), diff(h, qw_t), ...
    diff(h, wx), diff(h, wy), diff(h, wz)
    ]



