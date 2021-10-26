#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32 
from nav_msgs.msg import Odometry

def callback(msg): 
  print (msg.header)
  print (msg.child_frame_id)
  print ('pose: ')
  print (msg.pose.pose)
  print ('Covariance: ')
  print (msg.pose.covariance)

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
