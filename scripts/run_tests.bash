#!/bin/bash
set -e  # exit on first error
ROS_VERSION="indigo"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"


run_tests()
{
	# setup ros env
	source $HOME/catkin_ws/devel/setup.bash
	 
	#build the workspace
	cd $HOME/catkin_ws
	catkin_make

	#run the tests
	catkin_make run_tests_mcptam
		

}


# RUN
run_tests
