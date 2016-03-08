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
alt_holde_mode = 964
land_mode = 1514
stabilize_mode = 2064
default_ch6 = 964
default_ch7 = 1189
default_ch8 = 964
# Radio min/max.
roll_radio_min = 1103.0
roll_radio_max = 1924.0
pitch_radio_min = 1103.0
pitch_radio_max = 1924.0
throttle_radio_min = 1104.0
throttle_radio_max = 1924.0
yaw_radio_min = 1103.0
yaw_radio_max = 1924.0
# Position bound.
x_min = -1.0
x_max = 1.0
y_min = -1.0
y_max = 1.0
z_min = 0.5
z_max = -1.0
# Low pass filters.
linear_rate_lpf = LowPassFilter(3, 10)
angular_rate_lpf = LowPassFilter(3, 10)
motor_output_lpf = LowPassFilter(6, 20)
# LQR info: we have 12 states in total:
# x, y, z, roll, pitch, yaw, v_x, v_y, v_z, roll_rate, pitch_rate, yaw_rate.
X0 = numpy.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# To be determined by the measurement.
u0 = numpy.array([2.9240, 4.9958, 4.0495, 4.0497, 2.0716])
# K matrix in LQR: u = -K(x - x0) + u0.
# The dimension of K should be (# of motors) x 12.
K = numpy.array([[  0.5029,    0.2439,   -0.3922,    2.1638,   -4.2673,    0.3435,    0.8306,    0.4102,   -0.6313,    0.5757,   -1.2255,    1.4144],
                 [  0.5069,   -0.3721,   -0.5489,   -3.4562,   -4.2667,   -0.4058,    0.8365,   -0.6322,   -0.9183,   -1.0447,   -1.1222,   -1.7047],
                 [ -0.4937,    0.2439,   -0.4141,    2.1752,    4.2025,   -0.6218,   -0.8156,    0.4101,   -0.7006,    0.6337,    1.2488,   -2.5987],
                 [ -0.4963,   -0.3719,   -0.5474,   -3.4776,    4.1798,    0.5530,   -0.8194,   -0.6324,   -0.8831,   -1.1093,    1.0844,    2.2792],
                 [  0.0023,    0.7773,   -0.2716,    7.0425,   -0.0286,    0.1576,    0.0041,    1.3129,   -0.4365,    2.0304,   -0.0288,    0.6455]])
# Used for computing velocity.
neg_infinity = -10000.0
last_x = neg_infinity
last_y = neg_infinity
last_z = neg_infinity
vx = 0.0
vy = 0.0
vz = 0.0
last_roll = neg_infinity
last_pitch = neg_infinity
last_yaw = neg_infinity
rollspeed = 0.0
pitchspeed = 0.0
yawspeed = 0.0
last_time = neg_infinity
# Log files.
now = datetime.datetime.now()
state_file_name = 'logs/state_' + now.isoformat() + '.txt'
control_output_file_name = 'logs/control_' + now.isoformat() + '.txt'
fix_point_file_name = 'logs/fix_point_' + now.isoformat() + '.txt'
state_file = open(state_file_name, 'w')
control_output_file = open(control_output_file_name, 'w')
fix_point_file = open(fix_point_file_name, 'w')

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

def thrust_to_pwm(value):
    if value <= 0.0:
        return 1000.0
    # pwm ranges from 1000 to 2000.
    # define pwm2 = pwm / 1000.
    # thrust = 15.15 * pwm2 ^ 2 - 28.6 * pwm + 13.14
    a = 15.15
    b = -28.6
    c = 13.14 - value
    delta = b * b - 4 * a * c
    if delta < 0.0:
        return 1000.0
    pwm2 = (-b + sqrt(delta)) / 2.0 / a
    # Clamp our pwm value so that it is between 1000.0 and 1700.0.
    return clamp(pwm2 * 1000.0, 1000.0, 1700.0)

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

    # Calculate linear and angular velocity.
    global last_x, last_y, last_z, last_roll, last_pitch, last_yaw, last_time
    global vx, vy, vz, rollspeed, pitchspeed, yawspeed
    global linear_rate_lpf, angular_rate_lpf, motor_output_lpf
    # Get current time.
    current_time = time.time()
    if last_time == neg_infinity:
        last_x = x
        last_y = y
        last_z = z
        last_roll = roll
        last_pitch = pitch
        last_yaw = yaw
        last_time = current_time
    elif current_time - last_time > 0.04:
        # Don't update until 0.1s has passed.
        dt = current_time - last_time
        vx = (x - last_x) / dt
        vy = (y - last_y) / dt
        vz = (z - last_z) / dt
        rollspeed = (roll - last_roll) / dt
        pitchspeed = (pitch - last_pitch) / dt
        yawspeed = (yaw - last_yaw) / dt
        last_x = x
        last_y = y
        last_z = z
        last_roll = roll
        last_pitch = pitch
        last_yaw = yaw
        last_time = current_time
    vx, vy, vz = linear_rate_lpf.output(numpy.array([vx, vy, vz]))
    rollspeed, pitchspeed, yawspeed = angular_rate_lpf.output( \
            numpy.array([rollspeed, pitchspeed, yawspeed]))

    X = numpy.array([x, y, z, roll, pitch, yaw,
        vx, vy, vz, rollspeed, pitchspeed, yawspeed])
    u = -K.dot(X - X0) + u0
    motor_outputs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(u.size):
        motor_outputs[i] = thrust_to_pwm(u[i])
    filtered_pwm = motor_output_lpf.output(numpy.array(motor_outputs))
    for i in range(u.size):
        motor_outputs[i] = filtered_pwm[i]

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

    # Convert x and y offsets into desired roll and pitch angles.
    global roll_pid
    global pitch_pid
    roll_pid.SetPoint = body_y_offset
    pitch_pid.SetPoint = -body_x_offset
    roll_pid.setWindup(5.0)
    pitch_pid.setWindup(5.0)
    roll_pid.update(0.0)
    pitch_pid.update(0.0)
    desired_roll = clamp(roll_pid.output, roll_min, roll_max)
    desired_pitch = clamp(pitch_pid.output, pitch_min, pitch_max)
    desired_yaw = 0.0
    '''
    '''
    # For testing.
    A = 300.0
    b = 1500.0
    w = 2 * pi / 6
    t = time.time()
    motor_outputs[0] = A * sin(w * t) + b
    motor_outputs[1] = A * sin(w * t + pi / 3) + b
    motor_outputs[2] = A * sin(w * t + pi / 3 * 2) + b
    motor_outputs[3] = A * sin(w * t + pi) + b
    motor_outputs[4] = A * sin(w * t + pi / 3 * 4) + b
    motor_outputs[5] = A * sin(w * t + pi / 3 * 5) + b
    '''
    state_file.write('%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n' \
            % (current_time, x, y, z, roll, pitch, yaw, \
            vx, vy, vz, rollspeed, pitchspeed, yawspeed))
    control_output_file.write('%f, %f, %f, %f, %f, %f, %f\n' \
            % (current_time, motor_outputs[0], motor_outputs[1], motor_outputs[2], \
            motor_outputs[3], motor_outputs[4], motor_outputs[5]))
    fix_point_file.write('%f, %f, %f, %f\n' \
            % (current_time, X0[0], X0[1], X0[2]))
    master.mav.attitude_send(0,
            motor_outputs[0],
            motor_outputs[1],
            motor_outputs[2],
            motor_outputs[3],
            motor_outputs[4],
            motor_outputs[5])

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
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_RAW" :
                global X0
                # x->roll, y->pitch, z->throttle
                x_desired = remap(msg.chan1_raw, roll_radio_min, roll_radio_max,\
                                  x_min, x_max)
                y_desired = remap(msg.chan2_raw, pitch_radio_min, pitch_radio_max,\
                                  y_min, y_max)
                z_desired = remap(msg.chan3_raw, throttle_radio_min, throttle_radio_max,\
                                  z_min, z_max)
                X0[0], X0[1], X0[2] = x_desired, y_desired, z_desired


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
