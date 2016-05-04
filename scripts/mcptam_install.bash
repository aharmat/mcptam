#!/bin/bash
# Normal Use: ./mcptam_install.bash /path/to/catkin_ws 
# CI Use: ./mcptam_install.bash /path/to/catkin_ws CI
# Note: You must set up a catkin workspace in advance and either launch this script
# from the workspace or pass the path to the workspace. Uses catkin_make to build 
# mcptam.

set -e  # exit on first error

# Determine workspace location
CATKIN_WS=$1
CI=false
if [ $# -eq 2 ] && [ $2 == "CI" ];	then
	CI=true
	MCPTAM_DIR=pwd
fi
echo "**********************"
echo "MCPTAM_DIR: $MCPTAM_DIR"
echo "**********************"


# Define dependency versions
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
	cd ..
	rm -rf build_deps

}

build_mcptam()
{

	if [ "$CI" = true ]; then
		# grab pull request version to be tested, build done by travis
		cd ..
	    cp -R mcptam $CATKIN_WS/src
	    cd -
	else
		# clone mcptam to catkin workspace and build for user
		cd $CATKIN_WS/src
		git clone https://github.com/wavelab/mcptam
		cd -
		cd $CATKIN_WS
		catkin_make
		cd -
	fi
}


# RUN
echo "##### Starting installation of mcptam dependencies..."
install_dependencies
echo "##### Starting build of mcptam..."
build_mcptam
echo "##### mcptam_install.bash script complete."	
