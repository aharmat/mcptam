#!/bin/bash
set -e  # exit on first error
ROS_VERSION="indigo"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"
ROS_PACKAGES_URL='http://packages.ros.org/ros/ubuntu'
APT_KEYS_URL='hkp://pool.sks-keyservers.net:80'
APT_TARGETS="$(lsb_release -sc) main"
SOURCES_LIST_TARGET='/etc/apt/sources.list.d/ros-latest.list'


install()
{
	# update sources.list and add apt-keys
	echo "deb $ROS_PACKAGES_URL $APT_TARGETS" > $SOURCES_LIST_TARGET
	apt-key adv --keyserver $APT_KEYS_URL --recv-key "0xB01FA116"

	# update apt and install ros
	apt-get update
	apt-get install -y ros-$ROS_VERSION-desktop-full

	# initialize rosdep
	rosdep init
	rosdep update

	# env setup
	echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $HOME/.bashrc

	# install ros
	apt-get install -y python-rosinstall

	# install ros packages	
	apt-get install -y \
		ros-$ROS_VERSION-pcl-ros \
		ros-$ROS_VERSION-image-transport \
		ros-$ROS_VERSION-image-transport-plugins \
		ros-$ROS_VERSION-libg2o
}


# RUN
install
