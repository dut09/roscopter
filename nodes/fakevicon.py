#!/usr/bin/env python

# Tao Du
# taodu@csail.mit.edu
# Nov 20, 2015
# Fake vicon output. A simple publisher to the topic TaoCopter.

import roslib; roslib.load_manifest('roscopter')
import rospy
from mit_msgs.msg import MocapPosition
from geometry_msgs.msg import Vector3
import sys, struct, time, os

if __name__ == '__main__':
  try:
    pub = rospy.Publisher('TaoCopter', MocapPosition, queue_size = 50)
    rospy.init_node('fakevicon', anonymous = True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
      pub.publish(MocapPosition('', 0, Vector3(4000, 2000, 1000), None))
      rate.sleep()
  except rospy.ROSInterruptException:
    pass 
