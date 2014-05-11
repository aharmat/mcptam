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
 * \file LoadStaticParamsServer.h
 * \brief Loads variables from the ROS parameter server for the Server
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Loads variables from the ROS parameter server into the static members of 
 * various classes.
 *
 * This file deals with server-specific variables.
 *
 ****************************************************************************************/

#ifndef __LOAD_STATIC_PARAMS_SERVER_H
#define __LOAD_STATIC_PARAMS_SERVER_H

#include <mcptam/MapMakerServerBase.h>
#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/ChainBundle.h>
#include <mcptam/SystemBase.h>
#include <ros/ros.h>

/// Load static parameters from the ROS parameter server for the server module
void LoadStaticParamsServer()
{
  ros::NodeHandle nh_priv("~");
  
  nh_priv.getParam("adjuster_recent_min_size", BundleAdjusterBase::snRecentMinSize);
  nh_priv.getParam("adjuster_recent_num", BundleAdjusterBase::snRecentNum);
  nh_priv.getParam("adjuster_min_map_points", BundleAdjusterBase::snMinMapPoints);
  
  nh_priv.getParam("bundle_max_iterations", ChainBundle::snMaxIterations);
  nh_priv.getParam("bundle_max_trials_after_failure", ChainBundle::snMaxTrialsAfterFailure);
  nh_priv.getParam("bundle_percent_limit", ChainBundle::sdUpdatePercentConvergenceLimit);
  nh_priv.getParam("bundle_update_limit", ChainBundle::sdUpdateRMSConvergenceLimit);
  
  nh_priv.getParam("mm_min_map_points", MapMakerServerBase::snMinMapPoints);
  nh_priv.getParam("mm_max_consecutive_failed_ba", MapMakerServerBase::snMaxConsecutiveFailedBA);
  nh_priv.getParam("mm_max_triangulation_kfs", MapMakerServerBase::snMaxTriangulationKFs);
  nh_priv.getParam("mm_init_depth", MapMakerServerBase::sdInitDepth);
  nh_priv.getParam("mm_init_point_max_num", MapMakerServerBase::snMaxInitPointsLevelZero);
  nh_priv.getParam("mm_init_point_mode", MapMakerServerBase::ssInitPointMode);  // options: "stereo", "idp", "both"
  nh_priv.getParam("mm_init_cov_thresh", MapMakerServerBase::sdInitCovThresh);
  nh_priv.getParam("mm_large_point_test", MapMakerServerBase::sbLargePointTest);
  
  nh_priv.getParam("level_zero_points", SystemBase::sbLevelZeroPoints);
}


#endif

