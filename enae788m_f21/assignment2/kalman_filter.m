%load data
data = load('sensor_data.mat')
%get acceleration data
a = data.a
a = cell2struct(struct2cell(a), {'imu', 'imu1', 'imu0'})
%get angular velocity data
w = data.w
w = cell2struct(struct2cell(w), {'imu', 'imu1', 'imu0'})
%magetic measurement
m = data.m
%interpolate m
T = data.T
T = cell2struct(struct2cell(T), {'vicon', 'px4_estimator', 'imu', 'mag', 'imu1', 'imu0'})
mx = interp1(T.mag(:), m(:, 1), T.imu)
my = interp1(T.mag(:), m(:, 2), T.imu)
mz = interp1(T.mag(:), m(:, 3), T.imu)
m = [mx', my', mz'];

