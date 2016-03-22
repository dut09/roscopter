#!/usr/bin/env python
import roslib; roslib.load_manifest('roscopter')
import rospy
import time
from std_msgs.msg import String, Header
from mit_msgs.msg import MocapPosition
from std_srvs.srv import *
from transformations import *
from roscopter.msg import *
from roscopter.srv import *
from math import *
import sys, struct, time, os, math
from pymavlink import mavutil
from PID import *
from low_pass_filter import *
import datetime

###############################################################################
# Global variables definition.
###############################################################################
# RC transmitter used by NRL.
now = datetime.datetime.now()
state_file_name = 'logs-vicon/state_' + now.isoformat() + '.txt'
state_file = open(state_file_name, 'w')

###############################################################################
# Function definition.
###############################################################################
def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)"
          % (m.target_system, m.target_system))

def remap(value, in_low, in_high, out_low, out_high):
    return (value - in_low) / (in_high - in_low) * (out_high - out_low) \
           + out_low

def clamp(value, low, high):
    return max(low, min(value, high))

def rad_to_deg(value):
    return value / pi * 180.0

def deg_to_rad(value):
    return value / 180.0 * pi

def get_vicon_data(data):
    # Get the position info, in meters.
    x = data.translational.x / 1000.0 # Faked East direction.
    y = data.translational.y / 1000.0 # Faked North direction.
    z = data.translational.z / 1000.0 # Faked "meters above sea level."
    x, y, z = y, x, -z
    # Unfortunately the inertia frame in the simulator is not the same as in
    # VICON, so we have to convert x, y, z manually. Specifically, in VICON
    # we set x to right, y to front, and z to up. In our simulator, however,
    # x is front, y is right, and z is down.

    # Get the rotational angle.
    ax = data.axisangle.x
    ay = data.axisangle.y
    az = data.axisangle.z
    theta = math.sqrt(ax * ax + ay * ay + az * az)
    # Get the rotational axis. If theta is close to zero, set the axis to
    # [0, 0, 1] and theta to 0.
    if math.fabs(theta) < 1e-5:
      theta = 0.0
      ax = 0.0
      ay = 0.0
      az = 1.0
    else:
      ax = ax / theta
      ay = ay / theta
      az = az / theta
    # Note that (ax, ay, az) is defined in the WORLD frame, and if we rotate
    # the WORLD frame along (ax, ay, az) by theta, we will get our BODY frame.
    # Now here is a descrepency: we would like to switch our WORLD frame to
    # North(x)-East(y)-Down(z) frame so that it is aligned with our BODY frame.
    R = rotation_matrix(theta, [ax, ay, az])
    # Now each column in R represents the corresponding axis of the BODY frame
    # in the WORLD frame.
    R2 = numpy.dot(numpy.array([[0.0, 1.0, 0.0, 0.0],
                               [1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, -1.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0]]), R)
    # Now R represents the transformations between the BODY frame and the faked
    # North-East-Down frame. Specifically, R * [1 0 0 0]' returns the x axis of
    # the BODY frame in the NED frame. Now if we solve the Euler angles from R
    # we should be able to get the faked yaw angle. All angles are in radians.
    roll, pitch, yaw = euler_from_matrix(R2, 'sxyz') 

    # Get current time.
    current_time = time.time()

    '''
    # For testing.
    Rroll = rotation_matrix(roll, [1.0, 0.0, 0.0])
    Rpitch = rotation_matrix(pitch, [0.0, 1.0, 0.0])
    Ryaw = rotation_matrix(yaw, [0.0, 0.0, 1.0])
    if not numpy.allclose(R2, numpy.dot(numpy.dot(Ryaw, Rpitch), Rroll)):
      print 'panic!'
    # Compute the x and y offset.
    # In the body frame roll is x, pitch is y.
    world_x_offset = (dest_x - x)
    world_y_offset = (dest_y - y)
    body_x_offset = world_x_offset * sin(yaw) + world_y_offset * cos(yaw)
    body_y_offset= world_x_offset * cos(yaw) + world_y_offset * -sin(yaw)

    # For testing.
    A = 300.0
    b = 1500.0
    w = 2 * pi / 6
    t = time.time()
    x = A * sin(w * t) + b
    y = A * sin(w * t + pi / 3) + b
    z = A * sin(w * t + pi / 3 * 2) + b
    roll = A * sin(w * t + pi) + b
    pitch = A * sin(w * t + pi / 3 * 4) + b
    yaw = A * sin(w * t + pi / 3 * 5) + b
    '''
    state_file.write('%f, %f, %f, %f, %f, %f, %f\n' \
            % (current_time, x, y, z, roll, pitch, yaw))
    master.mav.vision_position_estimate_send(0,
            x, y, z, roll, pitch, yaw)

def set_arm(req):
    master.arducopter_arm()
    return []

def set_disarm(req):
    master.arducopter_disarm()
    return []

def mainloop():
    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)

###############################################################################
# Main script.
###############################################################################
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                '../mavlink/pymavlink'))
# Parse input commands.
from optparse import OptionParser
parser = OptionParser("position_controller.py [options]")
parser.add_option("--baudrate", dest = "baudrate", type = 'int',
                  help = "master port baud rate", default = 57600)
parser.add_option("--device", dest = "device", default = "/dev/ttyUSB0",
                  help = "serial device")
parser.add_option("--rate", dest = "rate", default = 50, type = 'int',
                  help = "requested stream rate")
parser.add_option("--source-system", dest = 'SOURCE_SYSTEM', type = 'int',
                  default = 255, help = 'MAVLink source system for this GCS')
parser.add_option("--name", dest = "name", default = "TaoCopter",
                  type = 'string', help = "copter models.")
(opts, args) = parser.parse_args()

# Create a mavlink serial instance.
master = mavutil.mavlink_connection(opts.device, baud = opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

# Set up publishers and subscribers.
rospy.Subscriber(opts.name, MocapPosition, get_vicon_data)

# Define service callbacks.
arm_service = rospy.Service('arm', Empty, set_arm)
disarm_service = rospy.Service('disarm', Empty, set_disarm)
# Wait for the heartbeat msg to find the system ID.
wait_heartbeat(master)

# Waiting for 10 seconds for the system to be ready.
print("Sleeping for 10 seconds to allow system, to be ready")
rospy.sleep(10)
print("Sending all stream request for rate %u" % opts.rate)

master.mav.request_data_stream_send(master.target_system,
                                    master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                    opts.rate, 1)

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
