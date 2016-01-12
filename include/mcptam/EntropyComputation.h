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
#include <mcptam/Types.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>  // needed for LEVELS define
#include <vector>
#include <boost/intrusive_ptr.hpp>
#include <mcptam/TrackerData.h>
#include <mcptam/MapPoint.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <mcptam/TrackerState.h>
#include <mcptam/Tracker.h> 

#ifndef __ENTROPY_COMPUTATION_H
#define __ENTROPY_COMPUTATION_H


inline double compute_point_entropy_scalar(double point_covariance)
{
	if(point_covariance>0)
		return 0.5*log(2*M_PI*2.71828*point_covariance);
	else
		return -1.0;
}


/// evaluates a map point and computes the expected entropy reduction based on the current keyframe observation in the tracker.
  double EvaluatePoint(Tracker* tracker, MapPoint& point, KeyFrame& tracker_kf,double priorPointCovariance, int level, double& prevEntropy);
  
  /// evaluates the tracker covariance and returns the entropy for the x y and z directions in a vector
  TooN::Vector<3> EvaluateTracker(Tracker* tracker);
 

 #endif