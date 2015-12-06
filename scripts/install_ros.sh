#!/bin/sh
set -e
ROS_PACKAGES_URL='http://packages.ros.org/ros/ubuntu'
APT_KEYS_URL='hkp://pool.sks-keyservers.net:80'
SOURCES_LIST_TARGET='/etc/apt/sources.list.d/ros-latest.list'

install()
{
	# update sources.list and add apt-keys
	echo "deb $ROS_PACKAGES_URL $(lsb_release -sc) main" > $SOURCES_LIST_TARGET
	apt-key adv --keyserver $APT_KEYS_URL --recv-key 0xB01FA116

	# update apt and install ros
	apt-get update
	apt-get install ros-jade-desktop-full

	# initialize rosdep
	rosdep init
	rosdep update

	# env setup
	echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
	source >> ~/.bashrc

	# getting rosinstall
	apt-get install python-rosinstall
}


install
