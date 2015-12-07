#!/bin/bash
set -e  # exit on first error
ROS_PACKAGES_URL='http://packages.ros.org/ros/ubuntu'
APT_KEYS_URL='hkp://pool.sks-keyservers.net:80'
APT_TARGETS="$(lsb_release -sc) main"
SOURCES_LIST_TARGET='/etc/apt/sources.list.d/ros-latest.list'

TOON_VERSION="2.2"
TOON_FORMAT="TooN-$TOON_VERSION"
TOON_URL="http://www.edwardrosten.com/cvd/$TOON_FORMAT.tar.gz"

LIBCVD_VERSION="20150407"
LIBCVD_FORMAT="libcvd-$LIBCVD_VERSION"
LIBCVD_URL="http://www.edwardrosten.com/cvd/$LIBCVD_FORMAT.tar.gz"

GVARS3_VERSION="3.0"
GVARS3_FORMAT="gvars-$GVARS3_VERSION"
GVARS3_URL="http://www.edwardrosten.com/cvd/$GVARS3_FORMAT.tar.gz"

ROS_VERSION="indigo"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"


install_ros()
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

install_prerequisits()
{
	# install prerequisits
	apt-get install -y \
		gcc \
		g++ \
		make \
		build-essential \
		libsuitesparse-dev \
		freeglut3 \
		freeglut3-dev \
		freeglut3-dbg
}


install_toon()
{
	# download and extract
	wget $TOON_URL
	tar xvf $TOON_FORMAT.tar.gz

	# configure, build and install
	cd $TOON_FORMAT
	./configure && make && make install
	cd ..
}

install_libcvd()
{
	# download and extract
	wget $LIBCVD_URL
	tar xvf $LIBCVD_FORMAT.tar.gz

	# configure, build and install
	cd $LIBCVD_FORMAT
	export CXXFLAGS=-D_REENTRANT
	./configure --without-ffmpeg 
	make 
	make install
	cd ..
}

install_gvars3()
{
	# download and extract
	wget $GVARS3_URL
	tar xvf $GVARS3_FORMAT.tar.gz

	# configure, build and install
	cd $GVARS3_FORMAT
	./configure --disable-widgets
	make 
	make install
	cd ..
}

install_dependencies()
{
	install_ros
	install_prerequisits

	mkdir build_deps
	cd build_deps
	install_toon
	install_libcvd
	install_gvars3
	rm -rf build_deps
	cd ..
}

build_mcptam()
{
	# setup ros env
	source $ROS_BASH

	# create catkin workspace
	mkdir -p $HOME/catkin_ws/src
	cd $HOME/catkin_ws/src
	catkin_init_workspace
	cd -

	# setup catkin workspace
	cd $HOME/catkin_ws/
	catkin_make
	source devel/setup.bash
	
	# copy mcptam to catkin workspace
	cd ..
	cp -R mcptam $HOME/catkin_ws/src
	cd $HOME/catkin_ws/
	catkin_make	
}


# RUN
install_dependencies
build_mcptam
