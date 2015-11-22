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
from roscopter.srv import ChangeMode
import sys, struct, time, os

###############################################################################
# Global data definition
###############################################################################
rate = 50
dt = 1.0 / rate
# The desired position of the copter, all in millimeters.
# Positive z means higher altitude.
x_target = 0
y_target = 0
z_target = 1000
# Current position.
x_current = 0
y_current = 0
z_current = 0
# Store the old error to compute the derivatives
x_error_old = 0
y_error_old = 0
# Use z_threshold to determine whether the VICON data is ready.
z_threshold = 20
# Command at rest.
command_rest = [1455, 1455, 1050, 1450, 1685, 1500, 1500, 1500]
is_land_mode = False
LAND_MODE = 1815
ALTHOLD_MODE = 1685
# PID parameters for roll, pitch and altitude. We leave yaw uncontrolled and
# rely on the user to align the copter and the world coordinates. This could be
# resolved by using axisangles from the VICON data, but we just want to keep
# this example simple at the beginning.
rp_p = 0.5
rp_i = 0.0
rp_d = 0.5
z_p = 0.2
z_i = 0.1
z_d = 0.0
z_integral = 0.0
p_max = 75 # The max value for the proportional gain.
i_max = 100 # The max value for the integral gain.
d_max = 150 # The max value for the derivative gain.

channel_1 = 0
channel_2 = 0
channel_3 = 0
channel_4 = 0

def Clamp(val, lower, upper):
  return min(max(val, lower), upper)

def UpdateVicon(data):
  global x_current
  global y_current
  global z_current
  x_current = data.translational.x
  y_current = data.translational.y
  z_current = data.translational.z

def UpdateRC(data):
  global channel_1
  global channel_2
  global channel_3
  global channel_4 
  channel_1 = data.channel[0]
  channel_2 = data.channel[1]
  channel_3 = data.channel[2]
  channel_4 = data.channel[3]

def PublishCommand():
  global z_integral
  global x_error_old
  global y_error_old
  pub = rospy.Publisher('/send_rc', RC, queue_size = 50)
  r = rospy.Rate(rate)
  while not rospy.is_shutdown():
    command = list(command_rest)
    if z_current > z_threshold:
      # Let's implement a super simple P controller here to hold the altitude.
      x_error = x_target - x_current
      y_error = y_target - y_current
      z_error = z_target - z_current
      # Compute the derivatives.
      x_dev = (x_error - x_error_old) / dt
      y_dev = (y_error - y_error_old) / dt
      # Update old errors.
      x_error_old = x_error
      y_error_old = y_error
      # The convention of our RC:
      # channel 1: roll: 1000(left), 2000(right).
      # channel 2: pitch: 1000(up), 2000(down).
      # channel 3: throttle: 1000(down), 2000(up).
      # channel 4: yaw: 1000(left), 2000(right).
      # channel 5: flight mode: 1685(AltHold), 1815(Land).
      if z_current > 600:
        # Update roll for x hold.
        command[0] = command_rest[0] - Clamp(rp_p * x_error, -p_max, p_max) \
                     - Clamp(rp_d * x_dev, -d_max, d_max)
        # Update pitch for y hold.
        command[1] = command_rest[1] + Clamp(rp_p * y_error, -p_max, p_max) \
                     + Clamp(rp_d * y_dev, -d_max, d_max)
      # Update the throttle value for altitude hold.
      z_integral = z_integral + z_error * dt
      command[2] = 1450 + Clamp(z_p * z_error, -150, 150) + \
                   Clamp(z_i * z_integral, -i_max, i_max)
    if is_land_mode:
      command[4] = LAND_MODE
    else:
      command[4] = ALTHOLD_MODE
    print x_current, y_current, z_current, channel_1, channel_2, channel_3, channel_4
    pub.publish(command)
    r.sleep()

# Service: change the flight mode.
# 0: Land mode.
# 1: AltHolde mode.
def SetFlightMode(req):
  global is_land_mode
  if req.mode == 0:
    is_land_mode = True 
  else:
    is_land_mode = False
  return 0

# The program begins here.
if __name__ == '__main__':
  try:
    rospy.init_node("taocopter", anonymous = True)
    # ServiceL change the flight mode.
    rospy.Service('setmode', ChangeMode, SetFlightMode)
    rospy.Subscriber("/TaoCopter", MocapPosition, UpdateVicon)
    rospy.Subscriber("/rc", RC, UpdateRC)
    PublishCommand()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
