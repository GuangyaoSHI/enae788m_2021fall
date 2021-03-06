%transition function
syms phi theta real
syms Ts real
syms wx wy wz real
%state x = [phi, theta]
d_x = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
       0, cos(phi), -sin(phi)];
f = [phi, theta]'+ Ts*d_x*[wx, wy, wz]'
% f = d_x*[wx, wy, wz]'
F = [diff(f, phi), diff(f, theta)]

%syms g real
g =  9.8;
h = (-g)*[-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)]';
H = [diff(h, phi), diff(h, theta)]


%load data
data = load('sensor_data.mat');
%get acceleration data
a = data.a;
a = cell2struct(struct2cell(a), {'imu', 'imu1', 'imu0'})
z = a.imu0;
a_cov = data.a_cov;
a_cov = cell2struct(struct2cell(a_cov), {'imu', 'imu1', 'imu0'})
R = a_cov.imu0;

%get angular velocity data
w = data.w;
w = cell2struct(struct2cell(w), {'imu', 'imu1', 'imu0'});
w_cov = data.w_cov;
w_cov = cell2struct(struct2cell(w_cov), {'imu', 'imu1', 'imu0'})

U = w.imu0(:, :);
%magetic measurement
m = data.m;
%interpolate m
T = data.T;
T = cell2struct(struct2cell(T), {'vicon', 'px4_estimator', 'imu', 'mag', 'imu1', 'imu0'});
mx = interp1(T.mag(:), m(:, 1), T.imu);
my = interp1(T.mag(:), m(:, 2), T.imu);
mz = interp1(T.mag(:), m(:, 3), T.imu);
m = [mx', my', mz'];


%extract estimation from px4 onboard estimator
px4_pose = data.pose;
phi_px4 = [];
theta_px4 = [];
yaw_px4 = [];
for i=1:length(px4_pose)
    pose = px4_pose{i};
    q_xyzw = pose.orientation;
    euler = quat2eul([q_xyzw(4), q_xyzw(1), q_xyzw(2), q_xyzw(3)]);
    phi_px4 = [phi_px4, euler(1)];
    theta_px4 = [theta_px4, euler(2)];
    yaw_px4 = [yaw_px4, euler(3)];
end


%initialize parameters and states
%state [phi, theta]
%assume phi, theta, and psi are approximately equal to zero
%then w_x = d_phi, w_y = d_theta, w_z = d_psi
X = zeros(2, length(T.imu0));
P = diag([1e-8, 1e-8]);
Q = diag([1e-7, 1e-7]);

for k = 2:length(T.imu0)
    Ts = T.imu0(k)-T.imu0(k-1);
    phi = X(1, k-1);
    theta = X(2, k-1);
    wx = U(k-1, 1);
    wy = U(k-1, 2);
    wz = U(k-1, 3);
    x_pred = [phi + Ts*wx + Ts*wz*cos(phi)*tan(theta) + Ts*wy*sin(phi)*tan(theta),
                             theta - Ts*wz*sin(phi) + Ts*wy*cos(phi)]';
    
    F_k = [ Ts*wy*cos(phi)*tan(theta) - Ts*wz*sin(phi)*tan(theta) + 1, Ts*wz*cos(phi)*(tan(theta)^2 + 1) + Ts*wy*sin(phi)*(tan(theta)^2 + 1);
                         - Ts*wy*sin(phi) - Ts*wz*cos(phi),                                                                     1];
    P_pred = F_k * P *F_k' +Q;
    
    
    %prediction 
    phi = x_pred(1);
    theta = x_pred(2);
    
    h =  [g*sin(theta),
 -g*cos(theta)*sin(phi),
 -g*cos(phi)*cos(theta)]';
    y_k = z(k, :)' - h';
    H_k =[0,          g*cos(theta);
        -g*cos(phi)*cos(theta), g*sin(phi)*sin(theta);
        g*cos(theta)*sin(phi), g*cos(phi)*sin(theta)
        ];
    R_k = R(k, :);
    R_k = reshape(R_k, 3, 3);
    S_k = H_k*P_pred*H_k'+R_k;
    K_k = P_pred*H_k'*S_k^-1;
    X(:, k) = x_pred' + K_k*y_k;
    P = (eye(2)-K_k*H_k)*P_pred;
end
   


t0 = T.px4_estimator;
t_px4 = t0 - t0(1);

t0 = T.imu0;
t0 = t0(1);
t = T.imu0-t0;
f = X(1,:);
p1 = plot(t(:), f(:), 'r', 'DisplayName', 'EKF')
hold on 
p2 = plot(t_px4(:), yaw_px4, 'b', 'DisplayName', 'PX4')
ylabel('Roll \phi')
xlabel('time')
lgd = legend('EKF', 'PX4')
lgd.FontSize = 20;
title('Estimate roll Naive EKF vs PX4 onboard estimator')
%saveas(gcf,'EKF_roll.pdf')
hold off

figure(2)
t0 = T.imu0;
t0 = t0(1);
t = T.imu0-t0;
f = X(2,:);
plot(t(:), -f(:), 'r', 'DisplayName', 'EKF')
hold on 
plot(t_px4(:), theta_px4, 'b', 'DisplayName', 'PX4')
ylabel('Pitch \theta')
xlabel('time')
lgd = legend('EKF', 'PX4')
lgd.FontSize = 20;
title('Estimate pitch Naive EKF vs PX4 onboard estimator')
%saveas(gcf,'EKF_roll.pdf')
hold off
