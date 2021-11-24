clear 
bag = rosbag('20211028_sysid_yaw_heave.bag');

%read outputs orientation and velocity

%orientation
bSel_imu = select(bag, 'Topic', '/mavros/imu/data');
msg_imu = readMessages(bSel_imu);
T_imu = cellfun(@(m) double(m.Header.Stamp.seconds), msg_imu);
% 50 hz 
ds = mean(T_imu(2:end)-T_imu(1:end-1))
freq = 1./ds

Orientation_imu_X = cellfun(@(m) double(m.Orientation.X), msg_imu);
Orientation_imu_Y = cellfun(@(m) double(m.Orientation.Y), msg_imu);
Orientation_imu_Z = cellfun(@(m) double(m.Orientation.Z), msg_imu);
Orientation_imu_W = cellfun(@(m) double(m.Orientation.W), msg_imu);
roll_imu = zeros(length(Orientation_imu_X), 1);
pitch_imu = zeros(length(Orientation_imu_X), 1);
yaw_imu = zeros(length(Orientation_imu_X), 1);
for i=1:length(Orientation_imu_X)
    eulZYX = quat2eul([Orientation_imu_W(i), Orientation_imu_X(i), Orientation_imu_Y(2), Orientation_imu_Z(3)]);
    roll_imu(i) = eulZYX(3);
    pitch_imu(i) = eulZYX(2);
    yaw_imu(i) = eulZYX(1);
end

%velocity
bSel_px4 = select(bag, 'Topic', '/mavros/local_position/velocity_body');
msg_px4 = readMessages(bSel_px4);
T_px4 = cellfun(@(m) double(m.Header.Stamp.seconds), msg_px4);
%1./mean(T_px4(2:end)-T_px4(1:end-1))
Linear_velocity_X_ = cellfun(@(m) double(m.Twist.Linear.X), msg_px4);
Linear_velocity_X =interp1(T_px4, Linear_velocity_X_, T_imu);
%plot(T_px4, Linear_velocity_X_, T_imu, Linear_velocity_X)

Linear_velocity_Y_ = cellfun(@(m) double(m.Twist.Linear.Y), msg_px4);
Linear_velocity_Y =interp1(T_px4, Linear_velocity_Y_, T_imu);
%plot(T_px4, Linear_velocity_Y_, T_imu, Linear_velocity_Y)


Linear_velocity_Z_ = cellfun(@(m) double(m.Twist.Linear.Z), msg_px4);
Linear_velocity_Z =interp1(T_px4, Linear_velocity_Z_, T_imu);
%plot(T_px4, Linear_velocity_Z_, T_imu, Linear_velocity_Z)


Angular_velocity_X_ = cellfun(@(m) double(m.Twist.Angular.X), msg_px4);
Angular_velocity_X =interp1(T_px4, Angular_velocity_X_, T_imu);
%plot(T_px4, Angular_velocity_X_, T_imu, Angular_velocity_X)


Angular_velocity_Y_ = cellfun(@(m) double(m.Twist.Angular.Y), msg_px4);
Angular_velocity_Y =interp1(T_px4, Angular_velocity_Y_, T_imu);
%plot(T_px4, Angular_velocity_Y_, T_imu, Angular_velocity_Y)


Angular_velocity_Z_ = cellfun(@(m) double(m.Twist.Angular.Z), msg_px4);
Angular_velocity_Z =interp1(T_px4, Angular_velocity_Z_, T_imu);
%plot(T_px4, Angular_velocity_Z_, T_imu, Angular_velocity_Z)


%read inputs throttle roll pitch yaw
bSel_input = select(bag, 'Topic', '/mavros/rc/in');
msg_input = readMessages(bSel_input, "DataFormat","struct");
T_input = cellfun(@(m) double(m.Header.Stamp.Sec)+10^-9*double(m.Header.Stamp.Nsec), msg_input);
throttle_ = cellfun(@(m) double(m.Channels(1)), msg_input);
throttle = interp1(T_input, throttle_, T_imu, 'linear', 'extrap');
% plot(T_input, throttle_, T_imu, throttle)
% legend('before', 'after')

roll_ = cellfun(@(m) double(m.Channels(2)), msg_input);
roll = interp1(T_input, roll_, T_imu, 'linear', 'extrap');
% plot(T_input, roll_, T_imu, roll)
% legend('before', 'after')
% plot( roll, roll_imu)
plot(T_imu, roll-mean(roll), T_imu, roll_imu*100)

pitch_ = cellfun(@(m) double(m.Channels(3)), msg_input);
pitch = interp1(T_input, pitch_, T_imu, 'linear', 'extrap');
% plot(T_input, pitch_, T_imu, pitch)

yaw_ = cellfun(@(m) double(m.Channels(4)), msg_input);
yaw = interp1(T_input, yaw_, T_imu, 'linear', 'extrap');
% plot(T_input, yaw_, T_imu, yaw)
plot(T_imu, yaw-mean(yaw), T_imu, (yaw_imu-mean(yaw_imu))*100)
legend('input', 'imu')

delta = 20;
u1 = throttle(20:end-delta);
u2 = roll(20:end-delta);
u3 = pitch(20:end-delta);
u4 = yaw(20:end-delta);
U = [u1, u2, u3, u4];

% y1 = Linear_velocity_X(20:end-delta);
% y2 = Linear_velocity_Y(20:end-delta);
% y3 = Linear_velocity_Z(20:end-delta);
% y4 = Angular_velocity_X(20:end-delta);
% y5 = Angular_velocity_Y(20:end-delta);
% y6 = Angular_velocity_Z(20:end-delta);
% y7 = Orientation_imu_X(20:end-delta);
% y8 = Orientation_imu_Y(20:end-delta);
% y9 = Orientation_imu_Z(20:end-delta);
% y10 = Orientation_imu_W(20:end-delta);
% Y = [y1, y2, y3, y4, y5, y6, y7, y8, y9, y10];

% lateral system identification
% Y = [roll_imu(20:end-delta), pitch_imu(20:end-delta), yaw_imu(20:end-delta), ...
%     Linear_velocity_X(20:end-delta), Linear_velocity_Y(20:end-delta), ...
%     Linear_velocity_Z(20:end-delta), Angular_velocity_X(20:end-delta), ...
%     Angular_velocity_Y(20:end-delta), Angular_velocity_Z(20:end-delta)];
%data = iddata(Y, U, ds, 'InputName', {'throttle', 'roll {in} \phi', 'pitch {in} \theta', 'yaw {in} \psi'},...
%    'OutputName', {'\phi roll out', '\theta pitch out', '\psi yaw out', 'v longitudinal', 'v lateral', 'v heave', 'rate roll', 'rate pitch', 'rate yaw'})

% Y = [roll_imu(20:end-delta), Linear_velocity_Y(20:end-delta)];
% data_lateral = iddata(Y, u2, ds, 'InputName', {'roll {in} \phi'},...
%     'OutputName', {'\phi roll out', 'v lateral'})


% longitudinal system identification
% Y = [roll_imu(20:end-delta), pitch_imu(20:end-delta), yaw_imu(20:end-delta), ...
%     Linear_velocity_X(20:end-delta), Linear_velocity_Y(20:end-delta), ...
%     Linear_velocity_Z(20:end-delta)];
% data_longitudinal = iddata(Y, U, ds, 'InputName', {'throttle', 'roll_{in}$\phi$', 'pitch_{in}$\theta$', 'yaw_{in}$\psi$'},...
%     'OutputName', {'roll_out', 'pitch_out', 'yaw_out', 'v_longitudinal', 'v_lateral', 'v_heave', 'rate_roll', 'rate_pitch', 'rate_yaw'})
% Y = [pitch_imu(20:end-delta), Linear_velocity_X(20:end-delta)];
% data_longitudinal = iddata(Y, u3, ds, 'InputName', {'pitch {in} \theta'},...
%     'OutputName', {'\theta pitch out', 'v longitudinal'})

% yaw and heave
delta = 20;
u4 = yaw(20:end-delta);
Y = Angular_velocity_Z(20:end-delta);
tt = floor(size(Y, 1)/2)
data_yaw = iddata(Y(1:tt), u4(1:tt), ds,'InputName', {'Yaw input \psi'},...
    'OutputName', {'Yaw rate output'})

delta = 500;
u1 = throttle(20:end-delta);
Y = Linear_velocity_Z(20:end-delta);
data_heave = iddata(Y(tt:end), u1(tt:end), ds,'InputName', {'Heave input'},...
    'OutputName', {'Heave velocity output'})
