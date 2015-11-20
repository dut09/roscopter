#-----------------------------------------------------------------------#
# PID position controller with VICON            			#
# Authors:								#
#	@ Usman Qayyum							#
#	@ Josiah Khor							#
# The protocol for ardu-ground control via Vicon Measurements		#
# For DIY community							#
#-----------------------------------------------------------------------#

#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('roscopter')
import rospy
import roscopter.msg
import sys,struct,time,os
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from pymavlink import mavutil
from geometry_msgs.msg import TransformStamped
import tf
send_odometry = TransformStamped()
pub_odom = rospy.Publisher('/controller_send', TransformStamped, queue_size=10)

#device = "/dev/ttyACM0"
#baud=115200

device = "/dev/ttyUSB0"
baud=57600
rate=50

# Connect to APM?
connect_APM=True

if connect_APM:
    master = mavutil.mavlink_connection(device, baud)
if device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat...")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

pub_state = rospy.Publisher('state', roscopter.msg.State, queue_size=10)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD, queue_size=10)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude, queue_size=10)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU, queue_size=10)
pub_gps = rospy.Publisher('gps', NavSatFix, queue_size=10)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC, queue_size=10)

gps_msg = NavSatFix()

# Initialise global variables for subscribing to Vicon/Kinect
vicon_x=0; vicon_y=0; vicon_z=0
vicon_roll=0; vicon_pitch=0; vicon_yaw=0; vicon_yaw_send=0

hz_counter = 0 

# Initialise reference position #
home_x = 0.0; home_y = 0.0; home_z = 0.0

# Set the PID values #
deg2pi = 3.14159/180.0
I_ex_MAX = 2; I_ey_MAX = 2; I_ez_MAX = 2				 # randomly picked atm

roll_P = .04 #0.4*deg2pi; 
roll_I = 0.01; roll_D = 300; 


roll_MAX = 12*deg2pi        


pitch_P = roll_P #0.4*deg2pi;
pitch_I = roll_I; pitch_D = roll_D; pitch_MAX = roll_MAX

 
thrust_P = 1.0; thrust_I = 0.0; thrust_D = 0; thrust_MAX = 0.1  	 # arbitrary range
yaw_P = 0.25; yaw_I = 0.0; yaw_D = 0.0; yaw_MAX = 20*deg2pi


# Sub-functions #

## Calculate error between XYZ coordinates ##
def calc_pos_error(home_x,home_y,home_z,pos_x,pos_y,pos_z):
    err_x = pos_x - home_x
    err_y = pos_y - home_y
    err_z = pos_z - home_z
    return (err_x, err_y, err_z)

## Update position values ##
def update_pos_values(old_pos_x, old_pos_y, old_pos_z,
		      new_pos_x, new_pos_y, new_pos_z,
		      first_time):
    if first_time:
	return (new_pos_x, new_pos_y, new_pos_z,
		 new_pos_x, new_pos_y, new_pos_z,)
    else:
	return (old_pos_x, old_pos_y, old_pos_z,
		 new_pos_x, new_pos_y, new_pos_z,)

## Set home values from Vicon/Kinect data ##
def set_home():
    global home_x
    global home_y
    global home_z
    global home_yaw
    global home_roll
    global home_pitch
    home_x = vicon_x
    home_y = vicon_y
    home_z = vicon_z
    home_roll = vicon_roll
    home_pitch = vicon_pitch
    home_yaw = vicon_yaw

## Function to update vicon values (in parallel with main loop) ##
def get_vicon(vicon_msg):
    global vicon_x
    global vicon_y
    global vicon_z
    global vicon_roll
    global vicon_pitch
    global vicon_yaw
    global vicon_yaw_send
    global home_x
    global home_y
    global hz_counter

    if 1: #hz_counter==1: 
	    tmp_home_x = home_x
	    tmp_home_y = home_y
	    tmp_vicon_x = vicon_msg.transform.translation.x
	    tmp_vicon_y = vicon_msg.transform.translation.y
	    vicon_z = vicon_msg.transform.translation.z

	   # if hz_counter < 4:
	   #	hz_counter = hz_counter + 1

	    rx = vicon_msg.transform.rotation.x
	    ry = vicon_msg.transform.rotation.y
	    rz = vicon_msg.transform.rotation.z
	    rw = vicon_msg.transform.rotation.w
	   
	    (vicon_roll, vicon_pitch, vicon_yaw) = tf.transformations.euler_from_quaternion([rx, ry, rz, rw])
	    
	    if vicon_yaw>3.14159:
		vicon_yaw=vicon_yaw-(2*3.14159)
	   
	    vicon_x  = math.cos(vicon_yaw)*tmp_vicon_x  + math.sin(vicon_yaw)*tmp_vicon_y 
	    vicon_y  = -math.sin(vicon_yaw)*tmp_vicon_x  + math.cos(vicon_yaw)*tmp_vicon_y
	 #   home_x  = math.cos(vicon_yaw)*tmp_home_x  + math.sin(vicon_yaw)*tmp_home_y 
	 #   home_y  = -math.sin(vicon_yaw)*tmp_home_x  + math.cos(vicon_yaw)*tmp_home_y  
	  
	    if vicon_yaw < 0:
	       vicon_yaw_send = vicon_yaw + 3.14159*2
	    

# Main loop where we implement PID control #
def mainloop():
    print("Waiting")
    global hz_counter
    hz_counter=1;
    trate = 1.0/rate
    first_time_imu=True
    first_time_pos=True
    earliercmdtime = 0
    ## Set home values from Vicon/Kinect data ##
    rospy.sleep(.1)
    set_home()
    
    ## Integration and differentiation error initialized ##
    I_ex=0.0; I_ey=0.0; I_ez=0.0
    D_ex=0.0; D_ey=0.0; D_ez=0.0
    new_err_x=0.0; new_err_y=0.0; new_err_z=0.0;

    ## IMU values initialized ##
    imu_roll=0.0; imu_pitch=0.0; first_yaw=0.0;
    imu_yaw=0.0; first_roll=0.0; first_pitch=0.0

    ## Position values initialised ##
    pos_x=0.0; pos_y=0.0; pos_z=0.0
  
    
    while not rospy.is_shutdown(): # start loop
       
        #continue
        if float(time.time())-trate >= earliercmdtime:
                earliercmdtime = time.time()
               
                hz_counter =1;
		if connect_APM:      # enable these lines if you want to obtain the data from the arducopter on-the-fly
	#            msg = master.recv_match(blocking=False)
	#            if not msg:
	#                continue
		    if 0:#msg:
			    if msg.get_type() == "BAD_DATA":
				if mavutil.all_printable(msg.data):
				    sys.stdout.write(msg.data)
				    sys.stdout.flush()
			    else: 
				msg_type = msg.get_type()
				if msg_type == "ATTITUDE" :
				    pub_attitude.publish(msg.roll*180/3.1415, msg.pitch*180/3.1415, msg.yaw*180/3.1415, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
		    	            if first_time_imu:
					first_time_imu=False
					first_yaw = msg.yaw
					first_roll = msg.roll
					first_pitch = msg.pitch
					imu_yaw = msg.yaw
					imu_roll = msg.roll
					imu_pitch = msg.pitch
				    else:
					imu_yaw = msg.yaw
					imu_roll = msg.roll
					imu_pitch = msg.pitch	   

		
                ## Update position values ##
		(old_pos_x,old_pos_y,old_pos_z,pos_x,pos_y,pos_z) = update_pos_values(pos_x,pos_y,pos_z,vicon_x,vicon_y,vicon_z,first_time_pos)	
		(err_x,err_y,err_z) = calc_pos_error(home_x,home_y,home_z,pos_x,pos_y,pos_z)   # home - pos
		(d_x, d_y, d_z) = calc_pos_error(old_pos_x,old_pos_y,old_pos_z,pos_x,pos_y,pos_z) 

		## Toggle for first time through loop ##
		if first_time_pos:
		    first_time_pos=False

		## PID controller  (roll+pitch and thrust, yaw later) ##
		I_ex = I_ex + err_x*trate
		I_ey = I_ey + err_y*trate
		I_ez = I_ez + err_z*trate

		V_x = d_x*trate	# units of m/s
		V_y = d_y*trate	# units of m/s
		V_z = d_z*trate	# units of m/s

		## Cap the integration error as well after certain value ##
		if (I_ex) > I_ex_MAX:
		    I_ex = I_ex_MAX #* I_ex / abs(I_ex)

		if (I_ex) < -I_ex_MAX:
		    I_ex = -I_ex_MAX #* I_ex / abs(I_ex)

		if (I_ey) > I_ey_MAX:
		    I_ey = I_ey_MAX #* I_ey / abs(I_ey)

		if (I_ey) < -I_ey_MAX:
		    I_ey = -I_ey_MAX #* I_ey / abs(I_ey)

		if (I_ez) > I_ez_MAX:
		    I_ez = I_ez_MAX #* I_ez / abs(I_ez)

		if (I_ez) < I_ez_MAX:
		    I_ez = -I_ez_MAX #* I_ez / abs(I_ez)
	     
		p_pitch = pitch_P * err_x
		i_pitch = pitch_I * I_ex
		d_pitch = pitch_D * V_x

		p_roll =  roll_P * err_y
		i_roll =  roll_I * I_ey
		d_roll =  roll_D * V_y

		p_th = thrust_P * err_z
		i_th = thrust_I * (I_ez)
		d_th = thrust_D * V_z

		##  Calculate PID values to send
		sendroll =  (p_roll + i_roll + d_roll) #

		
		sendpitch = (p_pitch + i_pitch + d_pitch) 
		###### neg pitch     === hexacopter will move forward
		###### pos pitch     === hexacopter will move backward
		###### neg roll     === hexacopter will move left
		###### pos roll     === hexacopter will move right

	        sendthrust = p_th + i_th + d_th
                sendyaw = yaw_P*(vicon_yaw  - home_yaw) #0 #-imu_yaw 

		## Cap the roll/pitch/yaw/thrust values ##
		if (sendroll) > roll_MAX:
		    sendroll = roll_MAX #* sendroll / abs(sendroll)

		if (sendroll) < -roll_MAX:
		    sendroll = -roll_MAX #* sendroll / abs(sendroll)
	 

		if (sendpitch) > pitch_MAX:
		    sendpitch = pitch_MAX #* sendpitch / abs(sendpitch)
		   
		if (sendpitch) < -pitch_MAX:
		    sendpitch = -pitch_MAX #* sendpitch / abs(sendpitch)



	  	if sendthrust > thrust_MAX:
		    sendthrust = thrust_MAX
		if sendthrust < 0:
		    sendthrust = 0

               
		## Send values to APM ##
		if connect_APM:
		    master.mav.set_roll_pitch_yaw_thrust_send(master.target_system, master.target_component, 
		                			      sendroll, sendpitch, sendyaw, sendthrust)

                send_odometry.transform.translation.x = sendroll * 180/3.14159
                send_odometry.transform.translation.y = sendpitch * 180/3.14159
                send_odometry.transform.translation.z = sendyaw * 180/3.14159
		## Print some output to display for debugging ##
#		stdscr.addstr(0, 0, "Position controller is running...")
#		if connect_APM:
#		    stdscr.addstr(1, 0, "Connection to APM established...")
#		else:
#		    stdscr.addstr(1, 0, "******Warning: No connection to APM!*************")

#		stdscr.addstr(3, 0, "The gains are:")
#		stdscr.addstr(4, 0, "              P       I       D")
#		stdscr.addstr(5, 0, "Roll          %.3f   %.3f   %.3f" % (roll_P, roll_I, roll_D))
#		stdscr.addstr(6, 0, "Pitch         %.3f   %.3f   %.3f" % (pitch_P, pitch_I, pitch_D))
#		stdscr.addstr(7, 0, "Yaw           %.3f   %.3f   %.3f" % (yaw_P, yaw_I, yaw_D))
#		stdscr.addstr(8, 0, "Thrust        %.3f   %.3f   %.3f" % (thrust_P, thrust_I, thrust_D))
#		stdscr.addstr(10, 0, "Home position is:    (%+.3f,%+.3f,%+.3f) m" % (home_x, home_y, home_z))
#		stdscr.addstr(11, 0, "Current position is: (%+.3f,%+.3f,%+.3f) m" % (pos_x, pos_y, pos_z))
#		stdscr.addstr(12, 0, "Position error is:   (%+.3f,%+.3f,%+.3f) m" % (err_x, err_y, err_z))
	#	if connect_APM:
	#	    stdscr.addstr(14, 0, "The IMU values are  (%+.3f,%+.3f,%+.3f) deg     " % (imu_roll/deg2pi, imu_pitch/deg2pi, imu_yaw/deg2pi))
	#	else:
	#	    stdscr.addstr(14, 0, "The IMU values are not available!")
#		stdscr.addstr(15, 0, "The Vicon values are  (%+.3f,%+.3f,%+.3f) deg      " % (vicon_roll/deg2pi,vicon_pitch/deg2pi,vicon_yaw/deg2pi))
#		stdscr.addstr(17, 0, "Sending control commands to APM:")
#		stdscr.addstr(18, 0, "Roll (deg)    Pitch (deg)    Yaw (deg)    Thrust (PWM)")
#		stdscr.addstr(19, 0, "%+.3f         %+.3f         %+.3f         %+.3f" % (sendroll/deg2pi, sendpitch/deg2pi, sendyaw/deg2pi, sendthrust))
#		stdscr.addstr(20, 0, "      \n")
#		stdscr.refresh()
 #       else:
 #               hz_counter =0; #stdscr.addstr(22, 0, " The rate of command is not correct")
                pub_odom .publish(send_odometry)

#--------------------------------------------------#
# BEGIN PROGRAM                                    #
#--------------------------------------------------#

# 1. Wait for the heartbeat msg to find the system ID #
if connect_APM:
    wait_heartbeat(master)

# 2. Wait for 5 seconds for the system to be ready #
    print("Sleeping for 5 seconds to allow system to be ready...")
    rospy.sleep(5)
    print("Now sending all stream request for rate %u" % rate)

# 3. Read in IMU #
 #   master.mav.request_data_stream_send(master.target_system, master.target_component,
  #                                      mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)

# 4. Execute main loop #
if __name__ == '__main__':
    ## Initialise CURSES ##
  #  stdscr = curses.initscr()
  #  curses.noecho()
  #  curses.cbreak()
    try:
	## Subscribe to VICON/Kinect topic ##
        rospy.init_node('roscopter')  
        rospy.Rate(50);
        rospy.Subscriber("/vicon/hexacopter1/hexacopter1", TransformStamped, get_vicon)
	## Run mainloop ##
           
        mainloop()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    #finally:
	## Close CURSES ##
   #     curses.echo()
   #     curses.nocbreak()
   #     curses.endwin()


