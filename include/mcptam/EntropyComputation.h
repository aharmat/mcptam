/****************************************************************************************
 *
 * \file EntropyComputation.h
 * \brief Functions for entropy computations
 *
 * Copyright 2015   Arun Das, University of Waterloo (adas@uwaterloo.ca)
 * 
 *
 ****************************************************************************************/

#include <ros/ros.h>

#ifndef __ENTROPY_COMPUTATIONS_H
#define __ENTROPY_COMPUTATIONS_H


inline double compute_point_entropy_scalar(double point_covariance)
{
	if(point_covariance>0)
		return 0.5*log(2*M_PI*2.71828*point_covariance);
	else
		return -1.0;
}
 

 #endif