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
 * \file SystemClient.h
 * \brief Declaration of SystemClient class
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * Defines the rest of the objects necessary to run mcptam_client. Provides the blocking
 * Run function which loops until exiting, acquiring images, calling the tracker, 
 * and drawing to the window.
 *
 ****************************************************************************************/

#ifndef __SYSTEM_CLIENT_H
#define __SYSTEM_CLIENT_H

#include <mcptam/SystemBase.h>
#include <mcptam/Types.h>
#include <mcptam/Reset.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

class MapMakerClient;
class Tracker;

/** @brief Implements the rest of the objects necssary to run MCTAM_Client */
class SystemClient : public SystemBase
{
public:
  /// Creates objects, sets up GUI
  SystemClient();
  
  /// Destructor
  ~SystemClient();
  
  /** @brief Blocking function that loops indefinitiely
   * 
   *  Acquires images, calls the tracker, and draws to the window. MapMakerClient runs
   *  in its own thread in the background, so we don't have to call it here */
  void Run();
  
protected:

  /** @brief Deals with user interface commands
   *  @param command The saved command
   *  @param params The saved command parameters */
  void GUICommandHandler(std::string command, std::string params);
  
  /// Callback called when an initialization request is received
  bool InitSystemCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  
  /// Callback called when a reset request is received
  bool ResetSystemCallback(mcptam::Reset::Request &request, mcptam::Reset::Response &response);
  
  /// Publish the current tracker state
  void PublishState();
  
  /// Publish the current tracker pose
  void PublishPose();
  
  /// Publish the small preview image
  void PublishSmallImage();
  
  /// Check the flag for whether to draw the window
  bool IsHeadless();
  
  ImageBWMap mmFramesBW;   ///< %Map of greyscale CVD::Images
  
  MapMakerClient *mpMapMakerClient;   ///< Pointer to the MapMakerClient
  Tracker *mpTracker;      ///< Pointer to the Tracker
  
  bool mbDone;              ///< Should I quit run loop?
  
  int mnSmallImageLevel; ///< Pyramid level to get small images from
  ImageRefMap mmSmallImageOffsets;   ///< Offsets for drawing small images into message
  CVD::ImageRef mirSmallImageSize;   ///< Full size of compound small image that will be sent to server
  
  ros::ServiceServer mInitSystemServer;    ///< Offers service to initialize system
  ros::ServiceServer mResetSystemServer;   ///< Offers service to reset system
  
  ros::Publisher mSystemInfoPub;   ///< Publishes diagnostic messages about framerate and computation durations
  ros::Publisher mTrackerStatePub;   ///< Publisher for tracker state information
  ros::Publisher mTrackerPosePub;   ///< Publisher for tracker pose (position, orientation, covariances)
  ros::Publisher mTrackerPoseWithCovPub; ///< Publisher for tracker pose with covariance
  
  image_transport::Publisher mTrackerSmallImagePub;   ///< Publisher for tracker state information
  image_transport::ImageTransport mImageTransport;    ///< Image transport for small image
  
  ros::Duration mPosePublishDur;   ///< How often to publish pose
  ros::Time mLastPosePublishTime;  ///< Last time pose was published

};

#endif

