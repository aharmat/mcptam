#!/bin/bash
set -e  # exit on first error
ROS_VERSION="indigo"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"


run_tests()
{
	# setup ros env
	#source $HOME/indigo/catkin_ws/devel/setup.bash
	 
	#build the workspace
	#cd $HOME/indigo/catkin_ws
	#catkin_make

	#build the tests
	#catkin_make tests

	#run the tests
	#local test_result = catkin_make run_tests_mcptam
	#echo "$test_result"

	catkin_make tests
	catkin_make run_tests_mcptam
	
		

}


# RUN
run_tests
#test_status=$(run_tests)
#echo "$test_status"
#exit test_status
