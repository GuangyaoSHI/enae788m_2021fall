import sys

sys.path.extend(['/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2'])
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from EKF_tools import *
import copy
import scipy

# refrences
# http://wiki.ros.org/tf2/Tutorials/Quaternions


bag_position = rosbag.Bag('/home/guangyao/Downloads/20211012_positionctl_test.bag')
# bag_orientation = rosbag.Bag('/home/guangyao/Downloads/20211012_orientationctrl_test.bag')
# bag_handhold = rosbag.Bag('/home/guangyao/Downloads/20211012_handheld.bag')

# read data
bag_info = read_bag(bag_position)
scipy.io.savemat('sensor_data.mat', bag_info)
# plot vicon and on-board estimator
plot_ground_truth(bag_info)

# align the data in time, here we use the time step of mavros/imu/mag as the reference since its freq is the lowest

# {'T': T, 'q': q, 'q_cov': q_cov, 'w': w, 'w_cov': w_cov, 'a': a, 'a_cov': a_cov, 'm': m, 'm_cov': m_cov}
aligned_info = alignment(bag_info)

# horizon
T = aligned_info['T']
H = len(T)

# state are [p, a, q, w] 13-dim
# estimation state  X_{k|k}
X = np.zeros((7, H))
# covariance matrices
P = []

# initialize the estimator
# x, y, z
X[0:3, 0] = np.array([0, 0, 0])

# qx, qy, qz, qw
X[3:7, 0] = np.array([0, 0, 0, 1])


# initialize covariance matrix
P.append(np.eye(7)*0.1)

# prediction state X_{k|k-1}
X_p = copy.deepcopy(X)

for k in range(1, H):
    # predicted state estimation
    x_p = pred_state(k, T, X, aligned_info)
    #print 'predict x_p: {}'.format(x_p)
    X_p[:, k] = x_p
    # predicted covariance estimate
    P_k_1 = pred_covariance(k, T, X, P, aligned_info)
    # update measurement residual
    y_k = update_meas_res(k, aligned_info, x_p)
    # update innovation covariance and compute Kalman gain
    KH = update_inno_cov(k, x_p, P_k_1, aligned_info)
    K_k = KH[0]
    H_k = KH[1]
    # update state estimate
    x = x_p + K_k.dot(y_k)
    #print 'estimate x: {}'.format(x)
    X[:, k] = x
    # update covariance estimate
    P_k = np.matmul((np.eye(7)-np.matmul(K_k, H_k)), P_k_1)
    P.append(P_k)


plot_EKF_vicon_px4(bag_info, T, X)





