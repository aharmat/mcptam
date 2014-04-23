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
 * \file SystemFrontendBase.h
 * \brief Declaration of SystemFrontendBase interface
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * Base class for System and SystemClient, contains functions common to the frontend
 * such as publishing state and pose, and providing services for initializing and
 * resetting the tracker. The tracker object lives here.
 *
 ****************************************************************************************/

#ifndef __SYSTEM_FRONTEND_BASE_H
#define __SYSTEM_FRONTEND_BASE_H

#define MAX_STATS_QUEUE_SIZE 10

#include <mcptam/SystemBase.h>
#include <mcptam/Types.h>
#include <mcptam/Reset.h>
#include <queue>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

class Tracker;

/** @brief Base class for other System(...) classes, sets up commonly used objects and provides
 *         some useful functions.
 * 
 * The initialized objects are the OpenGL window, the video source, and the map,
 * along with some other data structures. Implements a callback function
 * that can be used with a GVars-driven menu system to catch inputs. */
class SystemFrontendBase : public SystemBase
{
public:

  /** @param windowName The name to be displayed in the OpenGL window's titlebar */
  SystemFrontendBase(std::string windowName, bool bFullSize);
  
  /// Frees dynamically allocated objects
  ~SystemFrontendBase();
  
protected:
  
  /** @brief Calculates the average of the durations contained in the argument
   *  @param queue The queue holding the durations
   *  @return The average duration */
  double AverageDuration(std::deque<ros::Duration>& queue);
  
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
  
  void PublishSystemInfo(std::stringstream& captionStream);
  
  /// Check the flag for whether to draw the window
  bool IsHeadless();
  
  
  Tracker *mpTracker;      ///< Pointer to the Tracker
  
  int mnSmallImageLevel; ///< Pyramid level to get small images from
  ImageRefMap mmSmallImageOffsets;   ///< Offsets for drawing small images into message
  CVD::ImageRef mirSmallImageSize;   ///< Full size of compound small image that will be sent to server
  
   // This data will be displayed on GUI and published by mSystemInfoPub
  std::queue<ros::Time> mqLoopTimes;
  std::deque<ros::Duration> mqTotalDurations;
  std::deque<ros::Duration> mqFrameGrabDurations;
  std::deque<ros::Duration> mqFrameDelayDurations;
  
  long int mnGrabAttempts;
  long int mnGrabSuccesses;
    
  image_transport::Publisher mSmallImagePub;   ///< Publisher for tracker state information
  image_transport::ImageTransport mImageTransport;    ///< Image transport for small image
  
  ros::ServiceServer mInitSystemServer;    ///< Offers service to initialize system
  ros::ServiceServer mResetSystemServer;   ///< Offers service to reset system
  
  ros::Publisher mSmallImagePointsPub;  ///< Publishes measurements the can be displayed in the small tracker image
  ros::Publisher mSystemInfoPub;   ///< Publishes diagnostic messages about framerate and computation durations
  ros::Publisher mStatePub;   ///< Publisher for tracker state information
  ros::Publisher mPoseArrayPub;   ///< Publisher for tracker pose (array of cameras)
  ros::Publisher mPoseWithCovPub; ///< Publisher for tracker pose with covariance
  
};

#endif

