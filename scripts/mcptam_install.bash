#!/bin/bash
# Normal Use: ./mcptam_install.bash /path/to/catkin_ws 
# CI Use: ./mcptam_install.bash /path/to/catkin_ws CI

set -e  # exit on first error

CATKIN_WS= $1
CI=false
if [ $2 = "CI" ]; then
	CI=true
else
fi

TOON_VERSION="2.2"
TOON_FORMAT="TooN-$TOON_VERSION"
TOON_URL="http://www.edwardrosten.com/cvd/$TOON_FORMAT.tar.gz"

LIBCVD_VERSION="20150407"
LIBCVD_FORMAT="libcvd-$LIBCVD_VERSION"
LIBCVD_URL="http://www.edwardrosten.com/cvd/$LIBCVD_FORMAT.tar.gz"

GVARS3_VERSION="3.0"
GVARS3_FORMAT="gvars-$GVARS3_VERSION"
GVARS3_URL="http://www.edwardrosten.com/cvd/$GVARS3_FORMAT.tar.gz"

install_prerequisites()
{
	# install prerequisites
	sudo apt-get install -y \
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
	./configure && make && sudo make install
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
	sudo make install
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
	sudo make install
	cd ..
}

install_dependencies()
{
	install_prerequisites

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
	# confirm setup of ros env
	source $ROS_BASH

	cd 
	if [ $CI ]; then
		# grab pull request version to be tested
		cd ..
	    cp -R mcptam $CATKIN_WS/src
	    cd -
	else
		# clone mcptam to catkin workspace
		cd $CATKIN_WS/src
		git clone https://github.com/wavelab/mcptam mcptam
		cd -
	fi
	cd $CATKIN_WS
	catkin_make
	cd -
}


# RUN
install_dependencies
build_mcptam
