import sys

sys.path.extend(['/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2'])
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from EKF_tools import *
import copy
import tf

# refrences
# http://wiki.ros.org/tf2/Tutorials/Quaternions


# bag_position = rosbag.Bag('/home/guangyao/Downloads/20211012_positionctl_test.bag')
bag_orientation = rosbag.Bag('/home/guangyao/Downloads/20211012_orientationctrl_test.bag')
# bag_handhold = rosbag.Bag('/home/guangyao/Downloads/20211012_handheld.bag')

# read data
bag_info = read_bag(bag_orientation)

aligned_IMU = alignment_interp(bag_info)


# acceleration
acc_x = aligned_IMU['a']['/mavros/imu/data_raw']['ax']
acc_y = aligned_IMU['a']['/mavros/imu/data_raw']['ay']
acc_z = aligned_IMU['a']['/mavros/imu/data_raw']['az']

# angular velocity
omega_x = aligned_IMU['w']['/mavros/imu/data_raw']['wx']
omega_y = aligned_IMU['w']['/mavros/imu/data_raw']['wy']
omega_z = aligned_IMU['w']['/mavros/imu/data_raw']['wz']
# magetic field
m = aligned_IMU['m']
# estimate phi and theta with complimentary filter
# horizon
T = aligned_IMU['T']
H = len(T)

# phi
phi = np.zeros(H)
# theta
theta = np.zeros(H)
# psi
psi = np.zeros(H)

# initialize roll and pitch
phi[0] = 0
theta[0] = 0
psi[0] = 0

# alpha
alpha = 0.01

for k in range(1, H):
    t = T[k] - T[k - 1]

    ax = acc_x[k]
    ay = acc_y[k]
    az = acc_z[k]
    g = 9.8

    px = omega_x[k]
    qy = omega_y[k]
    rz = omega_z[k]
    mx = m['mx'][k]
    my = m['my'][k]
    mz = m['mz'][k]
    # estimate by acce data
    phi_acc = np.arctan2(ay, -az)
    theta_acc = np.arcsin(ay / float(g))

    # estimate by gyro
    # derivative of phi at k-1
    d_phi = px + qy * np.sin(phi[k - 1]) * np.tan(theta[k - 1]) + rz * np.cos(phi[k - 1]) * np.tan(theta[k - 1])
    d_theta = qy * np.cos(phi[k - 1]) - rz * np.sin(phi[k - 1])

    phi_gyro = phi[k - 1] + t * d_phi
    theta_gyro = theta[k - 1] + t * d_theta
    phi[k] = alpha * phi_acc + (1 - alpha) * phi_gyro
    theta[k] = alpha * theta_acc + (1 - alpha) * theta_gyro
    psi[k] = np.arctan2(-float(mz * np.sin(phi[k]) - my * np.cos(phi[k])), float(mx * np.cos(theta[k]) +
                                                                                my * np.sin(theta[k]) * np.sin(phi[k]) +
                                                                                 mz * np.sin(theta[k]) * np.cos(phi[k])))

plot_CF_vicon(bag_info, T, phi, theta, psi)
