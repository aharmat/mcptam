#!/bin/sh
set -e
TOON_VERSION="2.2"
TOON_FORMAT="TooN-$TOON_VERSION"
TOON_URL="http://www.edwardrosten.com/cvd/$TOON_FORMAT.tar.gz"
LIBCVD_VERSION="20150407"
LIBCVD_FORMAT="libcvd-$LIBCVD_VERSION"
LIBCVD_URL="http://www.edwardrosten.com/cvd/$LIBCVD_FORMAT.tar.gz"
GVARS3_VERSION="3.0"
GVARS3_FORMAT="gvars-$GVARS3_VERSION"
GVARS3_URL="http://www.edwardrosten.com/cvd/$GVARS3_FORMAT.tar.gz"

install_prerequisits()
{
	apt-get install \
		gcc \
		g++ \
		make \
		build-essential
}

install_ros()
{
	apt-get install \
		ros-hydro-image-transport \
		ros-hydro-image-plugins \
		ros-hydro-pcl-ros \
		ros-hydro-libg2o \
		ros-hydro-uvc-camera
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
	install_prerequisits
	install_ros
	install_toon
	install_libcvd
	install_gvars3
}

install_dependencies
