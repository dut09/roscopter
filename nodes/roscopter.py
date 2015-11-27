#!/usr/bin/env python
import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from mit_msgs.msg import MocapPosition
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from transformations import *
import roscopter.msg
import sys,struct,time,os,math

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))


from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyUSB0", help="serial device")
parser.add_option("--rate", dest="rate", default=50, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")

(opts, args) = parser.parse_args()

from pymavlink import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


#This does not work yet because APM does not have it implemented
#def mav_control(data):
#    '''
#    Set roll, pitch and yaw.
#    roll                      : Desired roll angle in radians (float)
#    pitch                     : Desired pitch angle in radians (float)
#    yaw                       : Desired yaw angle in radians (float)
#    thrust                    : Collective thrust, normalized to 0 .. 1 (float)
#    '''    
#    master.mav.set_roll_pitch_yaw_thrust_send(master.target_system, master.target_component,
#                                                                data.roll, data.pitch, data.yaw, data.thrust)
#
#    print ("sending control: %s"%data)


def send_rc(data):
    master.mav.rc_channels_override_send(master.target_system, master.target_component,data.channel[0],data.channel[1],data.channel[2],data.channel[3],data.channel[4],data.channel[5],data.channel[6],data.channel[7])
    print ("sending rc: %s"%data)

# Fake GPS and compass data.
def send_vicon(data):
    # Get the translation and rotation data.
    x = data.translational.x # Faked East direction.
    y = data.translational.y # Faked North direction.
    z = data.translational.z # Millimeters, faked "meters above sea level."
    # The radius of the earth in millimeters.
    radius = 6378.0 * 1000.0 * 1000.0
    # Faked latitude.
    latitude = y * 1.0 / (radius + z) * 180.0 / math.pi * 1e7
    # Faked longitude.
    longitude = x * 1.0 / (radius + z) * 180.0 / math.pi * 1e7
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
    # we should be able to get the faked yaw angle.
    roll, pitch, yaw = euler_from_matrix(R2, 'sxyz') 
    '''
    # For testing.
    Rroll = rotation_matrix(roll, [1.0, 0.0, 0.0])
    Rpitch = rotation_matrix(pitch, [0.0, 1.0, 0.0])
    Ryaw = rotation_matrix(yaw, [0.0, 0.0, 1.0])
    if not numpy.allclose(R2, numpy.dot(numpy.dot(Ryaw, Rpitch), Rroll)):
      print 'panic!'
    '''
    # Send faked data.
    master.mav.hil_state_send(time.time() * 1000 * 1000, # Microseconds since UNIX epoch.
                              roll,
                              pitch,
                              yaw,
                              0.0, # Don't use rollspeed.
                              0.0, # Don't use pitchspeed.
                              0.0, # Don't use yawspeed.
                              latitude,
                              longitude,
                              z, # Altitude.
                              0.0, # Don't use groundspeed.
                              0.0,
                              0.0,
                              0.0, # Don't use acceleration.
                              0.0,
                              0.0)

#service callbacks
#def set_mode(mav_mode):
#    master.set_mode_auto()

def set_arm(req):
    master.arducopter_arm()
    return []

def set_disarm(req):
    master.arducopter_disarm()
    return []

pub_gps = rospy.Publisher('gps', NavSatFix, queue_size=50)
#pub_imu = rospy.Publisher('imu', Imu)
#pub_rc = rospy.Publisher('rc', roscopter.msg.RC, queue_size=50)
pub_state = rospy.Publisher('state', roscopter.msg.State, queue_size=50)
#pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD, queue_size=50)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude, queue_size=50)
#pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU, queue_size=50)
# Fake GPS data.
rospy.Subscriber("TaoCopter", MocapPosition, send_vicon)
if opts.enable_control:
    #rospy.Subscriber("control", roscopter.msg.Control , mav_control)
    rospy.Subscriber("send_rc", roscopter.msg.RC , send_rc)

#define service callbacks
arm_service = rospy.Service('arm',Empty,set_arm)
disarm_service = rospy.Service('disarm',Empty,set_disarm)


#state
gps_msg = NavSatFix()



def mainloop():
    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        #print msg.get_type()
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else: 
            msg_type = msg.get_type()
            '''
            if msg_type == "RC_CHANNELS_RAW" :
                pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]) 
            '''
            if msg_type == "HEARTBEAT":
                pub_state.publish(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                                  msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                                  mavutil.mode_string_v10(msg))
            '''
            if msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)
            '''
            if msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix=NavSatStatus.STATUS_FIX
                pub_gps.publish(NavSatFix(latitude = msg.lat/1e07,
                                          longitude = msg.lon/1e07,
                                          altitude = msg.alt/1e03,
                                          status = NavSatStatus(status=fix, service = NavSatStatus.SERVICE_GPS)
                                          ))
            #pub.publish(String("MSG: %s"%msg))
            if msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)


            if msg_type == "LOCAL_POSITION_NED" :
                print "Local Pos: (%f %f %f) , (%f %f %f)" %(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            '''
            if msg_type == "RAW_IMU" :
                pub_raw_imu.publish (Header(), msg.time_usec,
                                     msg.xacc, msg.yacc, msg.zacc,
                                     msg.xgyro, msg.ygyro, msg.zgyro,
                                     msg.xmag, msg.ymag, msg.zmag)
            '''



# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)


# waiting for 10 seconds for the system to be ready
print("Sleeping for 5 seconds to allow system, to be ready")
rospy.sleep(5)
print("Sending all stream request for rate %u" % opts.rate)
#for i in range(0, 3):

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

#master.mav.set_mode_send(master.target_system, 
if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
