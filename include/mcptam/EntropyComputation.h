/*************************************************************************
 *
 *
 *  Copyright 2014  Adam Harmat (McGill University)
 *                      [adam.harmat@mail.mcgill.ca]
 *                  Michael Tribou (University of Waterloo)
 *                      [mjtribou@uwaterloo.ca]
 *
 *  Multi-Camera Parallel Tracking and Mapping (MCPTAM) is free software:
 *  you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation,
 *  either version 3 of the License, or (at your option) any later
 *  version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  MCPTAM is based on the Parallel Tracking and Mapping (PTAM) software.
 *  Copyright 2008 Isis Innovation Limited
 *
 *
 ************************************************************************/


/****************************************************************************************
 *
 * \file EntropyComputation.h
 * \brief Functions for entropy computations
 *
 * Copyright 2015   Arun Das, University of Waterloo (adas@uwaterloo.ca)
 *
 *
 * This class contains functions for calculating the entropy (uncertainty) of state
 * parameters in tracker to decide when a keyframe should be selected. Once the individual 
 * entropy of a state rises above a user-defined threshold, a multi-keyframe that maximizes
 * the reduction in expected entropy is found and inserted in the map.
 *
 * For more details visit: A. Das and S. L. Waslander, “Entropy based keyframe selection for
 * multi-camera visual slam,” in Ieee/rsj international conference on intelligent robots and
 * systems (iros), Hamburg, Germany, 2015, pp. 3676-3681.
 ****************************************************************************************/

#include <ros/ros.h>
#include <mcptam/Types.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>  // needed for LEVELS define
#include <boost/intrusive_ptr.hpp>
#include <mcptam/TrackerData.h>
#include <mcptam/MapPoint.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <mcptam/TrackerState.h>
#include <mcptam/Tracker.h>

#ifndef __ENTROPYCOMPUTATION_H
#define __ENTROPYCOMPUTATION_H


inline double compute_point_entropy_scalar(double pointCovariance)
{
	if(pointCovariance>0)
		return 0.5*log(2*M_PI*M_E*pointCovariance);
	else
		return -1.0;
}

/// evaluates a map point and computes the expected entropy reduction based on the current keyframe observation in the tracker.
double EvaluatePointEntropyReduction(Tracker* tracker, MapPoint& point, KeyFrame& trackerKF,double priorPointCovariance, int level, double& prevEntropy);

/// evaluates the tracker covariance and returns the entropy for the x y and z directions in a vector
TooN::Vector<3> EvaluateTrackerEntropy(Tracker* tracker);

void ComputePointMotionInAnchorKF(TooN::Vector<3> pointInCamera, KeyFrame& anchorKF, double deltaLength, TooN::SO3<>& Rp);

void PerturbPoint(TooN::Vector<3> pointInCamera, TooN::SO3<> rp, double deltaLength, TooN::Matrix<3>& jacobian);

double ComputeUpdatedCovariance(int pointLevel, TooN::Vector<2> imageJacobian, double priorPointCovariance);


#endif // __ENTROPYCOMPUTATION_H
