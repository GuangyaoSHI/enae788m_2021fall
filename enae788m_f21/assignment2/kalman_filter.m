%load data
data = load('sensor_data.mat');
%get acceleration data
a = data.a;
a = cell2struct(struct2cell(a), {'imu', 'imu1', 'imu0'})
%get angular velocity data
w = data.w;
w = cell2struct(struct2cell(w), {'imu', 'imu1', 'imu0'})
w_cov = data.w_cov;
w_cov = cell2struct(struct2cell(w_cov), {'imu', 'imu1', 'imu0'})
R = w_cov.imu;
U = w.imu(:, 1:2);
%magetic measurement
m = data.m;
%interpolate m
T = data.T;
T = cell2struct(struct2cell(T), {'vicon', 'px4_estimator', 'imu', 'mag', 'imu1', 'imu0'})
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


%FUSE = ahrsfilter('SampleRate', 200)
%[orientation,angularVelocity] = FUSE(a.imu, w.imu, m)

%https://philsal.co.uk/projects/imu-attitude-estimation
%initialize parameters and states
%state [phi, theta]
%assume phi, theta, and psi are approximately equal to zero
%then w_x = d_phi, w_y = d_theta, w_z = d_psi
X = zeros(2, length(T.imu));

P = diag([1e-8, 1e-8]);
% define state matrix
A = [1,0;
     0, 1];
C = [1, 0;
     0, 1];

 
for k = 2:length(T.imu)
    ts = T.imu(k)-T.imu(k-1);
    B = [ts, 0;0, ts]'; 
    R_k = R(k, :);
    R_k = reshape(R_k, 3, 3);
    R_k = R_k(1:2, 1:2);
    Q_k = ts^2*R_k;
    Q_k = Q_k(1:2, 1:2);
    
    %predict
    x_pred = A*X(:,k-1) + B*U(k-1, :)';
    P_pred = A*P*A' + Q_k;
    
    %update
    y_k = w.imu(k, 1:2)' - C*x_pred;
    S_k = C*P_pred*C'+R_k;
    K_k = P_pred*C'*S_k^-1;
    X(:, k) = x_pred + K_k*y_k;
    P = (eye(2)-K_k*C)*P_pred;
end




t0 = T.px4_estimator;
t_px4 = t0 - t0(1);

t0 = T.imu;
t0 = t0(1);
t = T.imu-t0;
f = X(1,:);
p1 = plot(t(:), f(:), 'r', 'DisplayName', 'KF')
hold on 
p2 = plot(t_px4(:), yaw_px4, 'b', 'DisplayName', 'PX4')
ylabel('Roll \phi')
xlabel('time')
legend('KF', 'PX4')
title('Estimate roll Naive KF vs PX4 onboard estimator')
hold off

figure(2)
t0 = T.imu;
t0 = t0(1);
t = T.imu-t0;
f = X(2,:);
plot(t(:), f(:), 'r', 'DisplayName', 'KF')
hold on 
plot(t_px4(:), theta_px4, 'b', 'DisplayName', 'PX4')
ylabel('Pitch \theta')
xlabel('time')
legend('KF', 'PX4')
title('Estimate pitch Naive KF vs PX4 onboard estimator')
hold off