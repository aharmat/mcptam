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
 * \file LoadStaticParamsGeneral.h
 * \brief Loads variables from the ROS parameter server for both client and server
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Loads variables from the ROS parameter server into the static members of 
 * various classes.
 *
 * This file deals with variables that apply to both client and server.
 *
 ****************************************************************************************/

#ifndef __LOAD_STATIC_PARAMS_GENERAL_H
#define __LOAD_STATIC_PARAMS_GENERAL_H

#include <mcptam/KeyFrame.h>
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/PatchFinder.h>
#include <mcptam/CameraGroupSubscriber.h>
#include <ros/ros.h>

/// Load static parameters from the ROS parameter server for the client and server modules
void LoadStaticParamsGeneral()
{
  ros::NodeHandle nh_priv("~");
  
  nh_priv.getParam("kf_candidate_thresh", KeyFrame::sdCandidateThresh); 
  nh_priv.getParam("kf_distance_mean_diff_fraction", KeyFrame::sdDistanceMeanDiffFraction);
  nh_priv.getParam("kf_adaptive_thresh", KeyFrame::sbAdaptiveThresh);
  
  nh_priv.getParam("sbi_width", SmallBlurryImage::sirSize.x);
  nh_priv.getParam("sbi_height", SmallBlurryImage::sirSize.y);
  
  nh_priv.getParam("finder_max_ssd_per_pixel", PatchFinder::snMaxSSDPerPixel);
  
  nh_priv.getParam("image_topic", CameraGroupSubscriber::sImageTopic);
  nh_priv.getParam("info_topic", CameraGroupSubscriber::sInfoTopic);
  nh_priv.getParam("pose_topic", CameraGroupSubscriber::sPoseTopic);
  nh_priv.getParam("camera_prefix", CameraGroupSubscriber::sCameraPrefix);
  nh_priv.getParam("dynamic_sync", CameraGroupSubscriber::sbDynamicSync);
  nh_priv.getParam("max_sync_window", CameraGroupSubscriber::sdMaxSyncWindow);
}


#endif

