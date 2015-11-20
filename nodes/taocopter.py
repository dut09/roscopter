#!/usr/bin/env python

# Tao Du
# taodu@csail.mit.edu
# Nov 20, 2015
# Modified from Usman Qayyum and Josiah Khor's PID position controller. 

# A controller to hold the current position of the copter by using VICON
# measurement. This is done by overwriting the RC inputs.

import roslib; roslib.load_manifest('roscopter')
import rospy
from mit_msgs.msg import MocapPosition
from roscopter.msg import RC
import sys, struct, time, os

# Global data definition
rate = 50
# The desired position of the copter, all in millimeters.
# Positive z means higher altitude.
x_target = 0
y_target = 0
z_target = 700
# Current position.
x_current = 0
y_current = 0
z_current = 0
# Use z_threshold to determine whether the VICON data is ready.
z_threshold = 20
# Command at rest.
command_rest = [1455, 1455, 1050, 1450, 1685, 1500, 1500, 1500]

def UpdateVicon(data):
  global x_current
  global y_current
  global z_current
  x_current = data.translational.x
  y_current = data.translational.y
  z_current = data.translational.z

def PublishCommand():
  pub = rospy.Publisher('/send_rc', RC, queue_size = 50)
  r = rospy.Rate(rate)
  while not rospy.is_shutdown():
    command = [];
    if z_current <= z_threshold:
      command = command_rest 
    else:
      #TODO: Based on the current VICON data, determine the signals we would
      # like to send.
      command = [0, 0, 0, 0, 0, 0, 0, 0];
    pub.publish(command)
    r.sleep()

# The program begins here.
if __name__ == '__main__':
  try:
    rospy.init_node("taocopter", anonymous = True)
    rospy.Subscriber("/TaoCopter", MocapPosition, UpdateVicon)
    PublishCommand()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
