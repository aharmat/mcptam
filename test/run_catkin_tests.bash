#!/bin/bash

catkin_make test 	#builds the tests
catkin_make run_tests #run the tests
catkin_test_results 	# Provide the results to Travis CI. This will return non-zero if a test fails.
