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
 * \file LoadStaticParamsClient.h
 * \brief Loads variables from the ROS parameter server for the Client
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * Loads variables from the ROS parameter server into the static members of
 * various classes.
 *
 * This file deals with client-specific variables.
 *
 ****************************************************************************************/

#ifndef MCPTAM_LOADSTATICPARAMSCLIENT_H
#define MCPTAM_LOADSTATICPARAMSCLIENT_H

#include <mcptam/MapMakerClientBase.h>
#include <mcptam/Tracker.h>
#include <ros/ros.h>

/// Load static parameters from the ROS parameter server for the client items
void LoadStaticParamsClient()
{
  ros::NodeHandle nh_priv("~");

  nh_priv.getParam("mm_min_outliers", MapMakerClientBase::snMinOutliers);
  nh_priv.getParam("mm_outlier_multiplier", MapMakerClientBase::sdOutlierMultiplier);
  nh_priv.getParam("mm_max_scaled_mkf_dist", MapMakerClientBase::sdMaxScaledMKFDist);
  nh_priv.getParam("mm_max_scaled_kf_dist", MapMakerClientBase::sdMaxScaledKFDist);

  nh_priv.getParam("tracker_rot_est_blur", Tracker::sdRotationEstimatorBlur);
  nh_priv.getParam("tracker_use_rot_est", Tracker::sbUseRotationEstimator);
  nh_priv.getParam("tracker_draw_corners", Tracker::sbDrawFASTCorners);
  nh_priv.getParam("tracker_max_patches", Tracker::snMaxPatchesPerFrame);
  nh_priv.getParam("tracker_min_patches", Tracker::snMinPatchesPerFrame);
  nh_priv.getParam("tracker_coarse_min", Tracker::snCoarseMin);
  nh_priv.getParam("tracker_coarse_max", Tracker::snCoarseMax);
  nh_priv.getParam("tracker_coarse_range", Tracker::snCoarseRange);
  nh_priv.getParam("tracker_coarse_sub_pix_its", Tracker::snCoarseSubPixIts);
  nh_priv.getParam("tracker_disable_coarse", Tracker::sbDisableCoarse);
  nh_priv.getParam("tracker_coarse_min_vel", Tracker::sdCoarseMinVelocity);
  nh_priv.getParam("tracker_mestimator", Tracker::sMEstimatorName);
  nh_priv.getParam("tracker_quality_good", Tracker::sdTrackingQualityGood);
  nh_priv.getParam("tracker_quality_bad", Tracker::sdTrackingQualityBad);
  nh_priv.getParam("tracker_collect_all_points", Tracker::sbCollectAllPoints);

  nh_priv.getParam("reloc_max_score", Relocaliser::sdRecoveryMaxScore);
}

#endif  // MCPTAM_LOADSTATICPARAMSCLIENT_H
