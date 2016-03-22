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
    A = 300.0
    b = 1500.0
    w = 2 * math.pi / 6
    while not rospy.is_shutdown():
      t = time.time()
      x = A * math.sin(w * t) + b
      y = A * math.sin(w * t + math.pi / 3) + b
      z = A * math.sin(w * t + math.pi / 3 * 2) + b
      pub.publish(MocapPosition('fake vicon data', sample_count, Vector3(x, y, z), Vector3(1.0, 2.0, 4.0)))
      sample_count = sample_count + 1.0
      rate.sleep()
  except rospy.ROSInterruptException:
    pass 
