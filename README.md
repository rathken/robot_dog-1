################################################################################
# robot_dog simulation model
#
# To vizualize URDF file in rviz
################################################################################
ros_setup

roscore&

roslaunch urdf_tutorial display.launch model:=urdf/quadruped.xacro

#---------------------------------------
# optional : convert xacro to urdf file
#---------------------------------------
rosrun xacro xacro urdf/quadruped.xarco > urdf/quadruped.urdf
roslaunch urdf_tutorial display.launch model:=urdf/quadruped.urdf

################################################################################
# PyBullet
################################################################################

cd ~/robot
source sourceme
python git/robot_dog/code/quadruped_moving_forward.py

#type 'q' to exit at any time

It also saves a video named 'myQuad.mp4'
