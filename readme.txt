git clone --recursive https://github.com/dut09/roscopter.git
cd roscopter
git checkout demo
rosmake roscopter
cd mavlink
./pymavlink/generator/mavgen.py --output pymavlink/dialects/v10/ardupilotmega.py message_definitions/v1.0/ardupilotmega.xml

Notes:
- You might need to update PYTHONPATH to expose mavlink and roslib. In your ~/.bashrc add the following line:
export PYTHONPATH=$PYTHONPATH:/home/taodu/research/ros_workspace/sandbox/roscopter/mavlink/
export PYTHONPATH=$PYTHONPATH:/opt/ros/lunar/lib/python2.7/dist-packages/

- If you encounter "permission denied: /dev/ttyUSB0/", type the following comand:
sudo usermod -a -G dialout $USER
then log out and log in again.

Usage:
cd node
./position_controller.py --name='JieDji'
