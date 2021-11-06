%compute the symbolic state equation

%the orientation at the beginning 
syms qx_0 qy_0 qz_0 qw_0 real
%quaternion at time step t
syms qx_t qy_t qz_t qw_t real
%time step

%compute the rotation quaternion from q_0 to q_t
%q_t * q_0**-1
S_q0 = [0, -qz_0, qy_0; qz_0, 0, -qx_0; -qy_0, qx_0, 0]; 
q = [-[qx_0;qy_0;qz_0], qw_0*eye(3)-S_q0; 
    qw_0, [qx_0;qy_0;qz_0]'
] * [qx_t, qy_t, qz_t, qw_t]'


%compute rotation matix from q_0 to q_t
R_q = [2*q(4)**2+2*q(1)**2-1,     2*q(1)*q(2)-2*q(4)*q(3),   2*q(1)*q(3)+2*q(4)*q(2);
       2*q(1)*q(2)+2*q(4)*q(3), 2*q(4)**2+2*q(2)**2-1,       2*q(2)*q(3)-2*q(4)*q(1);
       2*q(1)*q(3)-2*q(4)*q(2), 2*q(2)*q(3)+2*q(4)*q(1),   2*q(4)**2+2*q(3)**2-1
       ]
   
 
%compute rate of quaternion with angular velocity
%complete matrix
% Q_qt = [qx_t, qw_t, -qz_t, qy_t;
%         qy_t, qz_t, qw_t, -qx_t;
%         qz_t, -qy_t, qx_t, qw_t;
%         qw_t, -qx_t, -qy_t, -qz_t;
% ]

%partial matrix
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

%acceleration noise input
syms wax way waz real 

%angular velocity input
syms wx wy wz real

%angular noise input 
syms vx vy vz real

X_next = [[px, py, pz]'+1/2*t_s**2*R_q'*([ax, ay, az]'+[wax, way, waz]');
           [qx_t, qy_t, qz_t, qw_t]'+1/2*Q_qt*([wx, wy, wz]'+[vx, vy, vz]')
]



% syms x y z
% f = [2*x+3*y+z, 5*x+10*y+3*z]'
% df = diff(f, x)


