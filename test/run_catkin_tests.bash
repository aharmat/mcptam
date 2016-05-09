#!/bin/bash

catkin_make run_tests 	#builds and runs the tests

catkin_test_results 	# Provide the results to Travis CI. This will return non-zero if a test fails.
