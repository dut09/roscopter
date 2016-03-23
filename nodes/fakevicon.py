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
from optparse import OptionParser
import time
import math
from transformations import *

parser = OptionParser("fakevicon.py [options]")
parser.add_option("--name", dest = "name", default = "TaoCopter",
                  type = "string", help = "copter models.")
parser.add_option("--rate", dest = "rate", default = 50, type = "int",
                  help = "stream rate")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
  try:
    pub = rospy.Publisher(opts.name, MocapPosition, queue_size = 50)
    rospy.init_node('fakevicon', anonymous = True)
    rate = rospy.Rate(opts.rate)
    sample_count = 0.0
    A_xyz = 300.0
    b_xyz = 1500.0
    w_xyz = 2 * math.pi / 6
    A_rpy = 0.5
    w_rpy = math.pi / 4
    while not rospy.is_shutdown():
      t = time.time()
      x = A_xyz * math.sin(w_xyz * t) + b_xyz
      y = A_xyz * math.sin(w_xyz * t + math.pi / 3) + b_xyz
      z = A_xyz * math.sin(w_xyz * t + math.pi / 3 * 2) + b_xyz
      roll = A_rpy * math.sin(w_rpy * t)
      pitch = A_rpy * math.sin(w_rpy * t + math.pi / 3)
      yaw = A_rpy * math.sin(w_rpy * t + math.pi / 3 * 2)
      R2 = euler_matrix(roll, pitch, yaw, 'sxyz')
      R = numpy.dot(numpy.array([[0.0, 1.0, 0.0, 0.0],
                                 [1.0, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, -1.0, 0.0],
                                 [0.0, 0.0, 0.0, 1.0]]), R2)
      angle, direc, _ = rotation_from_matrix(R)
      pub.publish(MocapPosition('fake vicon data', sample_count, Vector3(x, y, z), \
                                Vector3(direc[0] * angle, direc[1] * angle, direc[2] * angle)))
      sample_count = sample_count + 1.0
      rate.sleep()
  except rospy.ROSInterruptException:
    pass 
