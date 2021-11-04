import os
os.chdir('/home/guangyao/Github/enae788m_2021fall/enae788m_f21/assignment2')
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf
from EKF_tools import *
#refrences
#http://wiki.ros.org/tf2/Tutorials/Quaternions


bag_position = rosbag.Bag('/home/guangyao/Downloads/20211012_positionctl_test.bag')
# bag_orientation = rosbag.Bag('/home/guangyao/Downloads/20211012_orientationctrl_test.bag')
# bag_handhold = rosbag.Bag('/home/guangyao/Downloads/20211012_handheld.bag')

T, q, q_cov, w, w_cov, a, a_cov = read_bag(bag_position)

