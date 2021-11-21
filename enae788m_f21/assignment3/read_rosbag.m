bag = rosbag('20211028_sysid_yaw_heave.bag');

%read outputs orientation and velocity

%orientation
bSel_imu = select(bag, 'Topic', '/mavros/imu/data');
msg_imu = readMessages(bSel_imu);
T_imu = cellfun(@(m) double(m.Header.Stamp.seconds), msg_imu);
Orientation_imu_X = cellfun(@(m) double(m.Orientation.X), msg_imu);
Orientation_imu_Y = cellfun(@(m) double(m.Orientation.Y), msg_imu);
Orientation_imu_Z = cellfun(@(m) double(m.Orientation.Z), msg_imu);
Orientation_imu_W = cellfun(@(m) double(m.Orientation.W), msg_imu);

%velocity
bSel_px4 = select(bag, 'Topic', '/mavros/local_position/velocity_body');
msg_px4 = readMessages(bSel_px4);
T_px4 = cellfun(@(m) double(m.Header.Stamp.seconds), msg_px4);
Linear_velocity_X = cellfun(@(m) double(m.Twist.Linear.X), msg_px4);
Linear_velocity_Y = cellfun(@(m) double(m.Twist.Linear.Y), msg_px4);
Linear_velocity_Z = cellfun(@(m) double(m.Twist.Linear.Z), msg_px4);
Angular_velocity_X = cellfun(@(m) double(m.Twist.Angular.X), msg_px4);
Angular_velocity_Y = cellfun(@(m) double(m.Twist.Angular.Y), msg_px4);
Angular_velocity_Z = cellfun(@(m) double(m.Twist.Angular.Z), msg_px4);

%read inputs throttle roll pitch yaw
bSel_input = select(bag, 'Topic', '/mavros/rc/in');
msg_input = readMessages(bSel_input, "DataFormat","struct");
