import sys

sys.path.extend(['/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2'])
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from EKF_tools import *
import copy


# refrences
# http://wiki.ros.org/tf2/Tutorials/Quaternions


bag_position = rosbag.Bag('/home/guangyao/Downloads/20211012_positionctl_test.bag')
# bag_orientation = rosbag.Bag('/home/guangyao/Downloads/20211012_orientationctrl_test.bag')
# bag_handhold = rosbag.Bag('/home/guangyao/Downloads/20211012_handheld.bag')

# read data
bag_info = read_bag(bag_position)

aligned_IMU = align_imu(bag_info)

# estimate roll and pitch with complimentary filter
