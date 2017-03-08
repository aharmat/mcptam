#!/bin/bash
set -e  # exit on first error
ROS_VERSION="indigo"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"


run_tests()
{
	catkin_make tests
	catkin_make run_tests_mcptam
}


# RUN
run_tests
#test_status=$(run_tests)
#echo "$test_status"
#exit test_status
