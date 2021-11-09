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
#bag_orientation = rosbag.Bag('/home/guangyao/Downloads/20211012_orientationctrl_test.bag')
bag_handhold = rosbag.Bag('/home/guangyao/Downloads/20211012_handheld.bag')

# read data
bag_info = read_bag(bag_handhold)

aligned_IMU = alignment_interp(bag_info)


# acceleration
acc_x = aligned_IMU['a']['/mavros/imu/data_raw']['ax']
h1 = acc_x
acc_y = aligned_IMU['a']['/mavros/imu/data_raw']['ay']
h2 = acc_y
acc_z = aligned_IMU['a']['/mavros/imu/data_raw']['az']
h3 = acc_z
# angular velocity
omega_x = aligned_IMU['w']['/mavros/imu/data_raw']['wx']

omega_y = aligned_IMU['w']['/mavros/imu/data_raw']['wy']

omega_z = aligned_IMU['w']['/mavros/imu/data_raw']['wz']

# magetic field
mag = aligned_IMU['m']
b1 = mag['mx']
b2 = mag['my']
b3 = mag['mz']



# estimate phi and theta with complimentary filter
# horizon
T = aligned_IMU['T']
H = len(T)

q = np.zeros((4, H))
# initialize quaternion
q[:, 0] = np.array([1, 0, 0, 0])
# initialize gravity
m = np.array([0, 0, -1])
# initialize mag
n = np.array([b1[0], b2[0], b3[0]])/np.linalg.norm(np.array([b1[0], b2[0], b3[0]]))

# filter gain
K = 50
for t in range(1, H):
    # delta t
    delta_t = T[t] - T[t-1]
    # normalize magnetic vector
    b_norm = np.linalg.norm(np.array([b1[t-1], b2[t-1], b3[t-1]]))
    y0 = np.array([h1[t-1], h2[t-1], h3[t-1], b1[t-1]/b_norm, b2[t-1]/b_norm, b3[t-1]/b_norm])
    #estimation of q at t-1
    q_est = quaternion(q[0, t-1], q[1, t-1], q[2, t-1], q[3, t-1])
    h_pre = q_est.rotate(m)
    b_pre = q_est.rotate(n)
    y_pre = np.hstack((h_pre, b_pre))

    #error
    err = y0 - y_pre
    # matrix X
    q0 = q_est.q0
    qx = q_est.qx
    qy = q_est.qy
    qz = q_est.qz
    n1 = n[0]
    n2 = n[1]
    n3 = n[2]
    X = np.array([[                        2*qy,                        2*qz,                        2*q0,                        2*qx],
                  [-2*qx,                 -2*q0,                        2*qz,                        2*qy],
                  [4*q0,                           0,                           0,                        4*qz],
                  [4*n1*q0 + 2*n3*qy - 2*n2*qz, 4*n1*qx + 2*n2*qy + 2*n3*qz,           2*n3*q0 + 2*n2*qx,           2*n3*qx - 2*n2*q0],
                  [4*n2*q0 - 2*n3*qx + 2*n1*qz,           2*n1*qy - 2*n3*q0, 2*n1*qx + 4*n2*qy + 2*n3*qz,           2*n1*q0 + 2*n3*qy],
                  [4*n3*q0 + 2*n2*qx - 2*n1*qy,           2*n2*q0 + 2*n1*qz,           2*n2*qz - 2*n1*q0, 2*n1*qx + 2*n2*qy + 4*n3*qz]])
    XX = np.matmul(np.matmul(X.transpose(), X), X.transpose())
    delta_q = XX.dot(err)
    dq_err = K*delta_q
    dq = q_est.product(quaternion(0, omega_x[t-1], omega_y[t-1], omega_z[t-1]))
    q_t = (dq+dq_err)*delta_t + q[:, t-1]
    q_t = q_t/np.linalg.norm(q_t)
    q[:, t] = q_t


plot_CF_quaternion(bag_info, T, q)