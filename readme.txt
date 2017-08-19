After you install ros, try to follow their tutorial to get a basic understanding
of rosbuild (we did not use catkin for this project). You should have created a
workspace folder, which looks like this:
YOUR_WORKSPACE_FODER:
- setup.bash: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:YOUR_WORKSPACE_FOLDER
- sandbox/:
  - Your ros package 1
  - Your ros package 2
  - ...

You will need to add roscopter as a new rospackage in sandbox. To allow ros to
find it, you also need to add YOUR_WORKSPACE_FOLDER/ to ROS_PACKAGE_PATH. That
is why we want a setup.bash there. You can also add the following line to your
~/.bashrc:
source YOUR_WORKSPACE_FOLDER/setup.bash
So that you do not need to source setup.bash to add YOUR_WORKSPACE_FOLDER each
time.

Now go to sandbox, and clone roscopter from github.com/dut09:
git clone --recursive https://github.com/dut09/roscopter.git
cd roscopter

Next, checkout the demo branch:
git checkout demo

Now you need to build mavlink:
cd mavlink
./pymavlink/generator/mavgen.py --output pymavlink/dialects/v10/ardupilotmega.py message_definitions/v1.0/ardupilotmega.xml

Finally, build roscopter.
cd ../../
rosmake roscopter

Notes:
- You might need to update PYTHONPATH to expose mavlink and roslib. In your ~/.bashrc add the following line:
export PYTHONPATH=$PYTHONPATH:/home/taodu/research/ros_workspace/sandbox/roscopter/mavlink/
export PYTHONPATH=$PYTHONPATH:/opt/ros/lunar/lib/python2.7/dist-packages/

- If you encounter "permission denied: /dev/ttyUSB0/", type the following comand:
sudo usermod -a -G dialout $USER
then log out and log in again.

Usage:
Remember, your copter need to use Stabilize mode.

cd node
./position_controller.py --name='JieDji'

