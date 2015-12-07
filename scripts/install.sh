#!/bin/sh
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


install_ros()
{
	# update sources.list and add apt-keys
	echo "deb $ROS_PACKAGES_URL $APT_TARGETS" > $SOURCES_LIST_TARGET
	sh -c "apt-key adv --keyserver $APT_KEYS_URL --recv-key 0xB01FA116"

	# # update apt and install ros
	# apt-get update
	# apt-get install -y ros-jade-desktop-full

	# # initialize rosdep
	# rosdep init
	# rosdep update

	# # env setup
	# echo "source /opt/ros/jade/setup.bash" >> $HOME/.bashrc

	# # install ros
	# apt-get install -y python-rosinstall

	# # install ros packages	
	# apt-get install -y \
	# 	ros-jade-image-transport \
	# 	ros-jade-image-plugins \
	# 	ros-jade-pcl-ros \
	# 	ros-jade-libg2o \
	# 	ros-jade-uvc-camera
}

install_prerequisits()
{
	# install prerequisits
	apt-get install -y \
		gcc \
		g++ \
		make \
		build-essential
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


# RUN
install_dependencies
