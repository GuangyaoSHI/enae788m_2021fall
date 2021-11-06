import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from scipy.linalg import block_diag

# A = np.array([1,2,3,5])
# A = np.reshape(A, (2,2))
# B = np.array([i for i in range(1, 10)])
# B = np.reshape(B, (3, 3))
# block_diag(A, B)
#refrences
#http://wiki.ros.org/tf2/Tutorials/Quaternions

def read_bag(bag):
    # time stamps
    T = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[], '/mavros/imu/mag':[], '/mavros/local_position/pose':[], '/vicon/m500_joec/m500_joec':[]}
    # orientations/quaternions      will be stored as a numpy array 4x1
    q = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[]}
    # quaternion covariance
    q_cov = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[]}
    # angular velocity
    w = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[]}
    # angular velocity covariance
    w_cov = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[]}
    # linear acceleration
    a = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[]}
    # linear acceleration covariance
    a_cov = {'/mavros/imu/data_raw':[], '/imu0':[], '/imu1':[]}
    # magetic field
    m = []
    # magnetic field covariance
    m_cov = []
    # onboard estimation
    pose = []
    # vicon system
    vicon = []
    ros_topics = ['/mavros/imu/data_raw', '/imu0', '/imu1']
    for ros_topic in ros_topics:
        for topic, msg, t in bag.read_messages(topics=ros_topic):
            T[ros_topic].append(t.to_time())
            q[ros_topic].append(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
            q_cov[ros_topic].append(msg.orientation_covariance)
            w[ros_topic].append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
            w_cov[ros_topic].append(msg.angular_velocity_covariance)
            a[ros_topic].append(np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
            a_cov[ros_topic].append(msg.linear_acceleration_covariance)

    for topic, msg, t in bag.read_messages(topics='/mavros/imu/mag'):
        T['/mavros/imu/mag'].append(t.to_time())
        m.append(np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]))
        m_cov.append(msg.magnetic_field_covariance)

    for topic, msg, t in bag.read_messages(topics='/mavros/local_position/pose'):
        T['/mavros/local_position/pose'].append(t.to_time())
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        pose.append({'position':position, 'orientation':orientation})

    for topic, msg, t in bag.read_messages(topics='/vicon/m500_joec/m500_joec'):
        T['/vicon/m500_joec/m500_joec'].append(t.to_time())
        translation = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        rotation = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        vicon.append({'translation':translation, 'rotation':rotation})
    return {'T':T, 'q':q, 'q_cov':q_cov, 'w':w, 'w_cov':w_cov, 'a':a, 'a_cov':a_cov, 'm':m, 'm_cov':m_cov, 'pose':pose,
            'vicon':vicon}


def alignment(bag_info):
    # use the time stamp of the magnetic field as reference
    T = bag_info['T']['/mavros/imu/mag']
    # magetic field
    m = bag_info['m']
    # magnetic field covariance
    m_cov = bag_info['m_cov']

    # rotation/quaternion
    q = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # quaternion covariance
    q_cov = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # angular velocity
    w = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # angular velocity covariance
    w_cov = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # linear acceleration
    a = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # linear acceleration covariance
    a_cov = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}

    # time stamps for imus
    T_imu = {}
    for topic in bag_info['T']:
        if topic == '/mavros/imu/mag':
            continue
        else:
            T_imu[topic] = np.array(bag_info['T'][topic])

    ros_topics = ['/mavros/imu/data_raw', '/imu0', '/imu1']
    for t in T:
        for ros_topic in ros_topics:
            index = np.argmax(T_imu[topic]>=t)
            if index == 0:
                # there are two possible results
                # one is that the first element is greater than t
                if T_imu[topic][index] >= t:
                    q[ros_topic].append(bag_info['q'][topic][index])
                    q_cov[ros_topic].append(bag_info['q_cov'][topic][index])
                    w[ros_topic].append(bag_info['w'][topic][index])
                    w_cov[ros_topic].append(bag_info['w_cov'][topic][index])
                    a[ros_topic].append(bag_info['a'][topic][index])
                    a_cov[ros_topic].append(bag_info['a_cov'][topic][index])
                else:
                    # all elements are less than t
                    # use the last element
                    q[ros_topic].append(bag_info['q'][topic][-1])
                    q_cov[ros_topic].append(bag_info['q_cov'][topic][-1])
                    w[ros_topic].append(bag_info['w'][topic][index])
                    w_cov[ros_topic].append(bag_info['w_cov'][topic][-1])
                    a[ros_topic].append(bag_info['a'][topic][index])
                    a_cov[ros_topic].append(bag_info['a_cov'][topic][-1])
            else:
                q[ros_topic].append(bag_info['q'][topic][index-1])
                q_cov[ros_topic].append(bag_info['q_cov'][topic][index-1])
                w[ros_topic].append(bag_info['w'][topic][index])
                w_cov[ros_topic].append(bag_info['w_cov'][topic][index-1])
                a[ros_topic].append(bag_info['a'][topic][index])
                a_cov[ros_topic].append(bag_info['a_cov'][topic][index-1])

    return {'T': T, 'q': q, 'q_cov': q_cov, 'w': w, 'w_cov': w_cov, 'a': a, 'a_cov': a_cov, 'm': m, 'm_cov': m_cov}


def pred_state(k, T, X, aligned_info):
    # time intervals
    t = T[k]-T[k-1]
    qx = X[3, k-1]
    qy = X[4, k-1]
    qz = X[5, k-1]
    qw = X[6, k-1]
    R_q = np.array([[2*qw**2+2*qx**2-1,  2*qx*qy-2*qw*qz, 2*qx*qz+2*qw*qy],
                    [2*qx*qy+2*qw*qz,  2*qw**2+2*qy**2-1, 2*qy*qz-2*qw*qx],
                    [2*qx*qz-2*qw*qy,  2*qy*qz+2*qw*qx, 2*qw**2+2*qz**2-1]])
    # from body to the local world
    R_q = R_q.transpose()
    # test
    # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html

    Q_q = np.array([[qw, -qz, qy],
                    [qz, qw, -qx],
                    [-qy, qx, qw],
                    [-qx, -qy, -qz]])
    # compute acceleration input
    a = 1/3*(aligned_info['a']['/mavros/imu/data_raw'][k-1]+aligned_info['a']['/imu0'][k-1]+aligned_info['a']['/imu1'][k-1])
    # compute angular velocity input
    w = 1/3*(aligned_info['w']['/mavros/imu/data_raw'][k-1]+aligned_info['w']['/imu0'][k-1]+aligned_info['w']['/imu1'][k-1])
    # next time step
    p_kk = X[0:3, k-1] + 1/2*t**2*R_q.dot(a)

    q_kk = X[3:7, k-1] + 1/2*t*Q_q.dot(w)
    return np.hstack((p_kk, q_kk))


def pred_covariance(k, T, X, P, aligned_info):
    t_s = T[k] - T[k-1]
    # compute acceleration input
    a = 1 / 3 * (aligned_info['a']['/mavros/imu/data_raw'][k - 1] + aligned_info['a']['/imu0'][k - 1] +
                 aligned_info['a']['/imu1'][k - 1])
    # compute angular velocity input
    w = 1 / 3 * (aligned_info['w']['/mavros/imu/data_raw'][k - 1] + aligned_info['w']['/imu0'][k - 1] +
                 aligned_info['w']['/imu1'][k - 1])
    ax = a[0]
    ay = a[1]
    az = a[2]
    qx = X[3, k-1]
    qy = X[4, k-1]
    qz = X[5, k-1]
    qw = X[6, k-1]
    wx = w[0]
    wy = w[1]
    wz = w[2]
    R_q = np.array([[2 * qw ** 2 + 2 * qx ** 2 - 1, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy],
                    [2 * qx * qy + 2 * qw * qz, 2 * qw ** 2 + 2 * qy ** 2 - 1, 2 * qy * qz - 2 * qw * qx],
                    [2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 2 * qw ** 2 + 2 * qz ** 2 - 1]])

    Q_q = np.array([[qw, -qz, qy],
                    [qz, qw, -qx],
                    [-qy, qx, qw],
                    [-qx, -qy, -qz]])
    F = np.array([
        [1, 0, 0, 2 * ax * qx * t_s ** 2 + ay * qy * t_s ** 2 + az * qz * t_s ** 2, ay * qx * t_s ** 2 - az * qw * t_s ** 2,
         ay * qw * t_s ** 2 + az * qx * t_s ** 2, 2 * ax * qw * t_s ** 2 + ay * qz * t_s ** 2 - az * qy * t_s ** 2],
        [
            0, 1, 0, ax * qy * t_s ** 2 + az * qw * t_s ** 2, ax * qx * t_s ** 2 + 2 * ay * qy * t_s ** 2 + az * qz * t_s ** 2, az * qy * t_s ** 2 - ax * qw * t_s ** 2, 2 * ay * qw * t_s ** 2 - ax * qz * t_s ** 2 + az * qx * t_s ** 2],
        [
            0, 0, 1, ax * qz * t_s ** 2 - ay * qw * t_s ** 2, ax * qw * t_s ** 2 + ay * qz * t_s ** 2, ax * qx * t_s ** 2 + ay * qy * t_s ** 2 + 2 * az * qz * t_s ** 2, ax * qy * t_s ** 2 - ay * qx * t_s ** 2 + 2 * az * qw * t_s ** 2],
        [0, 0, 0, 1, wz / 2, -wy / 2, wx / 2],
        [0, 0, 0, -wz / 2, 1, wx / 2, wy / 2],
        [0, 0, 0, wy / 2, -wx / 2, 1, wz / 2],
        [0, 0, 0, -wx / 2, -wy / 2, -wz / 2, 1]
                 ])


    # compute covariance matrix of acceleration
    sigma_a1 = aligned_info['a_cov']['/mavros/imu/data_raw'][k-1]
    sigma_a1 = np.reshape(sigma_a1, (3, 3))
    sigma_w1 = aligned_info['w_cov']['/mavros/imu/data_raw'][k-1]
    sigma_w1 = np.reshape(sigma_w1, (3, 3))
    sigma_a2 = aligned_info['a_cov']['/imu0'][k-1]
    sigma_a2 = np.reshape(sigma_a1, (3, 3))
    sigma_w2 = aligned_info['w_cov']['/mavros/imu/data_raw'][k-1]
    sigma_w2 = np.reshape(sigma_w2, (3, 3))
    sigma_a3 = aligned_info['a_cov']['/imu1'][k-1]
    sigma_a3 = np.reshape(sigma_a1, (3, 3))
    sigma_w3 = aligned_info['w_cov']['/mavros/imu/data_raw'][k-1]
    sigma_w3 = np.reshape(sigma_w3, (3, 3))

    sigma_a = 1 / 9 * (sigma_a1 + sigma_a2 + sigma_a3)
    sigma_w = 1 / 9 * (sigma_w1 + sigma_w2 + sigma_w3)
    #Q_k
    Q_k = block_diag(1/4*t_s**4*np.matmul(np.matmul(R_q.transpose(), sigma_a), R_q), 1/4*t_s**2*np.matmul(np.matmul(Q_q, sigma_w), Q_q.transpose()))
    P_pred = np.matmul(np.matmul(F, P[k-1]), F.transpose()+Q_k)
    return P_pred


def update_meas_res(k, aligned_info, x_p):
    mx_0 = aligned_info['m'][0][0]
    my_0 = aligned_info['m'][0][1]
    mz_0 = aligned_info['m'][0][2]
    qx = x_p[3]
    qy = x_p[4]
    qz = x_p[5]
    qw = x_p[6]
    g = np.array([0, 0, -9.8])
    R_q = np.array([[2*qw**2+2*qx**2-1,  2*qx*qy-2*qw*qz, 2*qx*qz+2*qw*qy],
                    [2*qx*qy+2*qw*qz,  2*qw**2+2*qy**2-1, 2*qy*qz-2*qw*qx],
                    [2*qx*qz-2*qw*qy,  2*qy*qz+2*qw*qx, 2*qw**2+2*qz**2-1]])

    h = np.hstack( (R_q.dot(g), R_q.dot(g), R_q.dot(g), R_q.dot(np.array([mx_0, my_0, mz_0])) ) )
    z_imu_raw = np.hstack((aligned_info['a']['/mavros/imu/data_raw'][k]))
    z_imu0 = np.hstack((aligned_info['a']['/imu0'][k]))
    z_imu1 = np.hstack((aligned_info['a']['/imu1'][k]))
    z = np.hstack((z_imu_raw, z_imu0, z_imu1, aligned_info['m'][k]))
    return z-h


def update_inno_cov(k, x_p, P_k_1, aligned_info):
    mx_0 = aligned_info['m'][0][0]
    my_0 = aligned_info['m'][0][1]
    mz_0 = aligned_info['m'][0][2]
    qx = x_p[3]
    qy = x_p[4]
    qz = x_p[5]
    qw = x_p[6]
    g = 9.8
    H = np.array([
        [0, 0, 0, -2 * g * qz, -2 * g * qw, -2 * g * qx, -2 * g * qy],
        [0, 0, 0, 2 * g * qw, -2 * g * qz, -2 * g * qy, 2 * g * qx],
        [0, 0, 0, 0, 0, -4 * g * qz, -4 * g * qw],
        [0, 0, 0, -2 * g * qz, -2 * g * qw, -2 * g * qx, -2 * g * qy],
        [0, 0, 0, 2 * g * qw, -2 * g * qz, -2 * g * qy, 2 * g * qx],
        [0, 0, 0, 0, 0, -4 * g * qz, -4 * g * qw],
        [0, 0, 0, -2 * g * qz, -2 * g * qw, -2 * g * qx, -2 * g * qy],
        [0, 0, 0, 2 * g * qw, -2 * g * qz, -2 * g * qy, 2 * g * qx],
        [0, 0, 0, 0, 0, -4 * g * qz, -4 * g * qw],
        [
            0, 0, 0, 4 * mx_0 * qx + 2 * my_0 * qy + 2 * mz_0 * qz, 2 * my_0 * qx + 2 * mz_0 * qw, 2 * mz_0 * qx - 2 * my_0 * qw, 4 * mx_0 * qw - 2 * my_0 * qz + 2 * mz_0 * qy],
        [
            0, 0, 0, 2 * mx_0 * qy - 2 * mz_0 * qw, 2 * mx_0 * qx + 4 * my_0 * qy + 2 * mz_0 * qz, 2 * mx_0 * qw + 2 * mz_0 * qy, 4 * my_0 * qw + 2 * mx_0 * qz - 2 * mz_0 * qx],
        [
            0, 0, 0, 2 * my_0 * qw + 2 * mx_0 * qz, 2 * my_0 * qz - 2 * mx_0 * qw, 2 * mx_0 * qx + 2 * my_0 * qy + 4 * mz_0 * qz, 2 * my_0 * qx - 2 * mx_0 * qy + 4 * mz_0 * qw]
    ])

    # compute R_k
    sigma_a1 = aligned_info['a_cov']['/mavros/imu/data_raw'][k]
    sigma_a1 = np.reshape(sigma_a1, (3, 3))
    sigma_w1 = aligned_info['w_cov']['/mavros/imu/data_raw'][k]
    sigma_w1 = np.reshape(sigma_w1, (3, 3))
    sigma_a2 = aligned_info['a_cov']['/imu0'][k]
    sigma_a2 = np.reshape(sigma_a1, (3, 3))
    sigma_w2 = aligned_info['w_cov']['/mavros/imu/data_raw'][k]
    sigma_w2 = np.reshape(sigma_w2, (3, 3))
    sigma_a3 = aligned_info['a_cov']['/imu1'][k]
    sigma_a3 = np.reshape(sigma_a1, (3, 3))
    sigma_w3 = aligned_info['w_cov']['/mavros/imu/data_raw'][k]
    sigma_w3 = np.reshape(sigma_w3, (3, 3))
    # sigma_m = aligned_info['m_cov'][k]
    # sigma_m = np.reshape(sigma_m, (3, 3))
    sigma_m = np.diag([1.98e-8, 1.98e-8, 1.98e-8])
    # 21 x 21
    R_k = block_diag(sigma_a1, sigma_a2, sigma_a3, sigma_m)
    # 21 x 21
    S_k = np.matmul(np.matmul(H, P_k_1), H.transpose())+R_k
    # 13 x 21
    K_k = np.matmul(np.matmul(P_k_1, H.transpose()), np.linalg.inv(S_k))

    return [K_k, H]


def plot_ground_truth(bag_info):
    T_estimator = bag_info['T']['/mavros/local_position/pose']
    T_estimator = np.array(T_estimator)
    T_vicon = bag_info['T']['/vicon/m500_joec/m500_joec']
    T_vicon = np.array(T_vicon)
    # align time
    t_0 = min(T_estimator[0], T_vicon[0])
    fig, axs = plt.subplots(7, 1)

    X_estimator = [pose['position'][0] for pose in bag_info['pose']]
    Y_estimator = [pose['position'][1] for pose in bag_info['pose']]
    Z_estimator = [pose['position'][2] for pose in bag_info['pose']]

    qX_estimator = [pose['orientation'][0] for pose in bag_info['pose']]
    qY_estimator = [pose['orientation'][1] for pose in bag_info['pose']]
    qZ_estimator = [pose['orientation'][2] for pose in bag_info['pose']]
    qW_estimator = [pose['orientation'][3] for pose in bag_info['pose']]

    X_vicon = [pose['translation'][0] for pose in bag_info['vicon']]
    Y_vicon = [pose['translation'][1] for pose in bag_info['vicon']]
    Z_vicon = [pose['translation'][2] for pose in bag_info['vicon']]

    qX_vicon = [pose['rotation'][0] for pose in bag_info['vicon']]
    qY_vicon = [pose['rotation'][1] for pose in bag_info['vicon']]
    qZ_vicon = [pose['rotation'][2] for pose in bag_info['vicon']]
    qW_vicon = [pose['rotation'][3] for pose in bag_info['vicon']]


    #x
    axs[0].plot(T_estimator-t_0, X_estimator, label='estimator')
    axs[0].plot(T_vicon-t_0, X_vicon, label='vicon')
    axs[0].set_ylabel('x')
    axs[0].legend()

    #y
    axs[1].plot(T_estimator-t_0, Y_estimator, label='estimator')
    axs[1].plot(T_vicon-t_0, Y_vicon, label='vicon')
    axs[1].set_ylabel('y')
    axs[1].legend()

    #z
    axs[2].plot(T_estimator-t_0, Z_estimator, label='estimator')
    axs[2].plot(T_vicon-t_0, Z_vicon, label='vicon')
    axs[2].set_ylabel('z')
    axs[2].legend()

    #qx
    axs[3].plot(T_estimator-t_0, qX_estimator, label='estimator')
    axs[3].plot(T_vicon-t_0, qX_vicon, label='vicon')
    axs[3].set_ylabel('qx')
    axs[3].legend()

    #qy
    axs[4].plot(T_estimator-t_0, qY_estimator, label='estimator')
    axs[4].plot(T_vicon-t_0, qY_vicon, label='vicon')
    axs[4].set_ylabel('qy')
    axs[4].legend()

    #qz
    axs[5].plot(T_estimator-t_0, qZ_estimator, label='estimator')
    axs[5].plot(T_vicon-t_0, qZ_vicon, label='vicon')
    axs[5].set_ylabel('qz')
    axs[5].legend()

    # qw
    axs[6].plot(T_estimator - t_0, qW_estimator, label='estimator')
    axs[6].plot(T_vicon - t_0, qW_vicon, label='vicon')
    axs[6].set_ylabel('qw')
    axs[6].set_xlabel('time')
    axs[6].legend()

    fig.tight_layout()
    plt.show()
    fig.savefig('/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2/px4_vicon.pdf', pad_inches=0)


def plot_EKF_vicon_px4(bag_info, T, X):
    T_estimator = bag_info['T']['/mavros/local_position/pose']
    T_estimator = np.array(T_estimator)
    T_vicon = bag_info['T']['/vicon/m500_joec/m500_joec']
    T_vicon = np.array(T_vicon)
    T_EKF = np.array(T)
    # align time
    t_0 = min(T_estimator[0], T_vicon[0], T_EKF[0])
    fig, axs = plt.subplots(7, 1)

    X_estimator = [pose['position'][0] for pose in bag_info['pose']]
    Y_estimator = [pose['position'][1] for pose in bag_info['pose']]
    Z_estimator = [pose['position'][2] for pose in bag_info['pose']]

    qX_estimator = [pose['orientation'][0] for pose in bag_info['pose']]
    qY_estimator = [pose['orientation'][1] for pose in bag_info['pose']]
    qZ_estimator = [pose['orientation'][2] for pose in bag_info['pose']]
    qW_estimator = [pose['orientation'][3] for pose in bag_info['pose']]

    X_vicon = [pose['translation'][0] for pose in bag_info['vicon']]
    Y_vicon = [pose['translation'][1] for pose in bag_info['vicon']]
    Z_vicon = [pose['translation'][2] for pose in bag_info['vicon']]

    qX_vicon = [pose['rotation'][0] for pose in bag_info['vicon']]
    qY_vicon = [pose['rotation'][1] for pose in bag_info['vicon']]
    qZ_vicon = [pose['rotation'][2] for pose in bag_info['vicon']]
    qW_vicon = [pose['rotation'][3] for pose in bag_info['vicon']]


    #x
    axs[0].plot(T_estimator-t_0, X_estimator, label='estimator')
    axs[0].plot(T_vicon-t_0, X_vicon, label='vicon')
    axs[0].plot(T_EKF-t_0, X[0, :], label='EKF')
    axs[0].set_ylabel('x')
    axs[0].legend()

    #y
    axs[1].plot(T_estimator-t_0, Y_estimator, label='estimator')
    axs[1].plot(T_vicon-t_0, Y_vicon, label='vicon')
    axs[1].plot(T_EKF - t_0, X[1, :], label='EKF')
    axs[1].set_ylabel('y')
    axs[1].legend()

    #z
    axs[2].plot(T_estimator-t_0, Z_estimator, label='estimator')
    axs[2].plot(T_vicon-t_0, Z_vicon, label='vicon')
    axs[2].plot(T_EKF - t_0, X[2, :], label='EKF')
    axs[2].set_ylabel('z')
    axs[2].legend()

    #qx
    axs[3].plot(T_estimator-t_0, qX_estimator, label='estimator')
    axs[3].plot(T_vicon-t_0, qX_vicon, label='vicon')
    axs[3].plot(T_EKF - t_0, X[3, :], label='EKF')
    axs[3].set_ylabel('qx')
    axs[3].legend()

    #qy
    axs[4].plot(T_estimator-t_0, qY_estimator, label='estimator')
    axs[4].plot(T_vicon-t_0, qY_vicon, label='vicon')
    axs[4].plot(T_EKF - t_0, X[4, :], label='EKF')
    axs[4].set_ylabel('qy')
    axs[4].legend()

    #qz
    axs[5].plot(T_estimator-t_0, qZ_estimator, label='estimator')
    axs[5].plot(T_vicon-t_0, qZ_vicon, label='vicon')
    axs[5].plot(T_EKF - t_0, X[5, :], label='EKF')
    axs[5].set_ylabel('qz')
    axs[5].legend()

    # qw
    axs[6].plot(T_estimator - t_0, qW_estimator, label='estimator')
    axs[6].plot(T_vicon - t_0, qW_vicon, label='vicon')
    axs[6].plot(T_EKF - t_0, X[6, :], label='EKF')
    axs[6].set_ylabel('qw')
    axs[6].set_xlabel('time')
    axs[6].legend()

    fig.tight_layout()
    plt.show()
    fig.savefig('/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2/EKF_px4_vicon.pdf', pad_inches=0)



def align_imu(bag_info):
    # use the time stamp of the magnetic field as reference
    T = bag_info['T']['/mavros/imu/data_raw']

    # rotation/quaternion
    q = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # quaternion covariance
    q_cov = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # angular velocity
    w = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # angular velocity covariance
    w_cov = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # linear acceleration
    a = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}
    # linear acceleration covariance
    a_cov = {'/mavros/imu/data_raw': [], '/imu0': [], '/imu1': []}

    # time stamps for imus
    T_imu = {}
    for topic in bag_info['T']:
        if topic == '/mavros/imu/data_raw':
            continue
        else:
            T_imu[topic] = np.array(bag_info['T'][topic])

    ros_topics = ['/mavros/imu/data_raw', '/imu0', '/imu1']
    for t in T:
        for ros_topic in ros_topics:
            index = np.argmax(T_imu[topic]>=t)
            if index == 0:
                # there are two possible results
                # one is that the first element is greater than t
                if T_imu[topic][index] >= t:
                    q[ros_topic].append(bag_info['q'][topic][index])
                    q_cov[ros_topic].append(bag_info['q_cov'][topic][index])
                    w[ros_topic].append(bag_info['w'][topic][index])
                    w_cov[ros_topic].append(bag_info['w_cov'][topic][index])
                    a[ros_topic].append(bag_info['a'][topic][index])
                    a_cov[ros_topic].append(bag_info['a_cov'][topic][index])
                else:
                    # all elements are less than t
                    # use the last element
                    q[ros_topic].append(bag_info['q'][topic][-1])
                    q_cov[ros_topic].append(bag_info['q_cov'][topic][-1])
                    w[ros_topic].append(bag_info['w'][topic][index])
                    w_cov[ros_topic].append(bag_info['w_cov'][topic][-1])
                    a[ros_topic].append(bag_info['a'][topic][index])
                    a_cov[ros_topic].append(bag_info['a_cov'][topic][-1])
            else:
                q[ros_topic].append(bag_info['q'][topic][index-1])
                q_cov[ros_topic].append(bag_info['q_cov'][topic][index-1])
                w[ros_topic].append(bag_info['w'][topic][index])
                w_cov[ros_topic].append(bag_info['w_cov'][topic][index-1])
                a[ros_topic].append(bag_info['a'][topic][index])
                a_cov[ros_topic].append(bag_info['a_cov'][topic][index-1])

    return {'T': T, 'q': q, 'q_cov': q_cov, 'w': w, 'w_cov': w_cov, 'a': a, 'a_cov': a_cov}




def plot_CF_vicon(bag_info, T, roll, pitch, yaw):
    T_estimator = bag_info['T']['/mavros/local_position/pose']
    T_estimator = np.array(T_estimator)
    T_vicon = bag_info['T']['/vicon/m500_joec/m500_joec']
    T_vicon = np.array(T_vicon)
    T_CF = np.array(T)
    # align time
    t_0 = min(T_estimator[0], T_vicon[0], T_CF[0])
    fig, axs = plt.subplots(3, 1)


    # angles from the onboard estimator
    roll_est = []
    pitch_est = []
    yaw_est = []
    for pose in bag_info['pose']:
        angles = tf.transformations.euler_from_quaternion(pose['orientation'])
        roll_est.append(angles[0])
        pitch_est.append(angles[1])
        yaw_est.append(angles[2])


    # angles from the vicon
    # angles from the onboard estimator
    roll_vi = []
    pitch_vi = []
    yaw_vi = []
    for pose in bag_info['vicon']:
        angles = tf.transformations.euler_from_quaternion(pose['rotation'])
        roll_vi.append(angles[0])
        pitch_vi.append(angles[1])
        yaw_vi.append(angles[2])

    #roll
    axs[0].plot(T_estimator-t_0, roll_est, label='estimator')
    axs[0].plot(T_vicon-t_0, roll_vi, label='vicon')
    axs[0].plot(T_CF-t_0, roll, label='CF')
    axs[0].set_ylabel('roll')
    axs[0].legend()

    #pitch
    axs[1].plot(T_estimator-t_0, pitch_est, label='estimator')
    axs[1].plot(T_vicon-t_0, pitch_vi, label='vicon')
    axs[1].plot(T_CF - t_0, pitch, label='CF')
    axs[1].set_ylabel('pitch')
    axs[1].legend()

    #yaw
    axs[2].plot(T_estimator-t_0, yaw_est, label='estimator')
    axs[2].plot(T_vicon-t_0, yaw_vi, label='vicon')
    axs[2].plot(T_CF - t_0, yaw, label='CF')
    axs[2].set_ylabel('yaw')
    axs[2].legend()


    fig.tight_layout()
    plt.show()
    fig.savefig('/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2/CF_px4_vicon.pdf', pad_inches=0)