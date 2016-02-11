#!/bin/bash
set -e  # exit on first error
ROS_DISTRO=indigo
ROS_CI_DESKTOP="`lsb_release -cs`"  # e.g. [precise|trusty|...]

sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $ROS_CI_DESKTOP main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-desktop
sudo apt-get install -y ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-libg2o
# Prepare rosdep to install dependencies.
sudo rosdep init
rosdep update
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt-get install python-rosinstall
