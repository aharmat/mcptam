#!/bin/bash
set -e  # exit on first error
ROS_DISTRO=indigo  
ROS_CI_DESKTOP="`lsb_release -cs`"  # e.g. [precise|trusty|...]
ROS_BASH="/opt/ros/$ROS_DISTRO/setup.bash"

# To install catkin workspace in default location run script without argument
# To install in current directory, run with argument .
CATKIN_WS=$HOME
if [ $# = 1 ];  then
     CATKIN_WS=$1
fi


ros_base_install()
{ 
	sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $ROS_CI_DESKTOP main\" > /etc/apt/sources.list.d/ros-latest.list"
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
	sudo apt-get update -qq
	sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-desktop
	sudo apt-get install -y ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-libg2o
	sudo apt-get install python-rosinstall
}

rosdep_update()
{
	sudo rosdep init
	rosdep update
}

prepare_workspace()
{
	# setup ros env
	source $ROS_BASH

	# create catkin workspace
	cd $CATKIN_WS
	mkdir src
	cd src
	catkin_init_workspace
	cd ..

	# setup catkin workspace
	catkin_make
	source devel/setup.bash
}

# RUN
ros_base_install
rosdep_update
prepare_workspace

