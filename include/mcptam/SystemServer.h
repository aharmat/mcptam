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
 * \file SystemServer.h
 * \brief Declaration of SystemServer class
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * Defines the rest of the objects necessary to run mcptam_Server. Provides the
 *blocking
 * Run function which loops until exiting, only drawing to the window because
 *MapMakerServer
 * runs in its own thread in the background.
 *
 ****************************************************************************************/

#ifndef MCPTAM_SYSTEMSERVER_H
#define MCPTAM_SYSTEMSERVER_H

#include <string>
#include <mcptam/SystemBase.h>
#include <mcptam/Types.h>
#include <mcptam/Tracker.h>  // only for TrackingQuality
#include <mcptam/TrackerState.h>
#include <mcptam/SystemInfo.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class MapMakerServer;
class BundleAdjusterMulti;
class KeyFrameViewer;

/** @brief Implements the rest of the objects necssary to run MCTAM_Server */
class SystemServer : public SystemBase
{
public:
  /// Creates objects, sets up GUI
  SystemServer();

  /// Destructor
  ~SystemServer();

  /** @brief Blocking function that loops indefinitiely
   *
   *  Draws to the window. MapMakerServer runs
   *  in its own thread in the background, so we don't have to call it here */
  void Run();

protected:
  /** @brief Deals with user interface commands
   *  @param command The saved command
   *  @param params The saved command parameters */
  void GUICommandHandler(std::string command, std::string params);

  /// Callback called when a new tracker state message received
  void TrackerStateCallback(const mcptam::TrackerStateConstPtr &stateMsg);

  /// Callback called when a new small preview image is received
  void TrackerSmallImageCallback(const sensor_msgs::ImageConstPtr &imageMsg);

  /// Callback called when a new small preview image point set is received
  void TrackerSmallImagePointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pointMsg);

  /// Callback called when a new system info message received
  void SystemInfoCallback(const mcptam::SystemInfoConstPtr &infoMsg);

  MapMakerServer *mpMapMakerServer;       ///< Pointer to the MapMakerServer
  BundleAdjusterMulti *mpBundleAdjuster;  ///< Pointer to the bundle adjuster
  /// that uses MultiBundle
  KeyFrameViewer *mpKeyFrameViewer;  ///< Pointer to the KeyFrameViewer

  bool mbDone;  ///< Should I quit run loop?

  // Data received from the client side
  TooN::SE3<> mse3TrackerPose;                              ///< Pose from the Tracker
  Tracker::TrackingQuality mTrackingQuality;                ///< Quality of the pose tracking
  bool mbTrackerLost;                                       ///< Is the tracker lost?
  CVD::Image<CVD::Rgb<CVD::byte>> mTrackerSmallImage;       ///< Small composite image sent from the tracker
  pcl::PointCloud<pcl::PointXYZ> mTrackerSmallImagePoints;  ///< Measurement
  /// points to be
  /// overlaid on small
  /// image

  double mdFrameGrabDuration;   ///< Time to grab the camera frames
  double mdFrameDelayDuration;  ///< Delay between image capture and receiving at
  /// client
  double mdTrackingDuration;  ///< Time to track the pose
  double mdFPS;               ///< Number of frames per second
  double mdGrabSuccessRatio;  ///< Frame grab success rate

  ros::ServiceClient mInitSystemClient;   ///< Used to call initialization service on client
  ros::ServiceClient mResetSystemClient;  ///< Used to call reset on client
  ros::Subscriber mTrackerStateSub;       ///< Subscribe to tracker state messages
  ros::Subscriber mSystemInfoSub;         ///< Subscribe to system info messages

  image_transport::ImageTransport mImageTransport;  ///< Image transport for the image messages
  image_transport::Subscriber mSmallImageSub;
  ros::Subscriber mSmallImagePointsSub;
};

#endif  // MCPTAM_SYSTEMSERVER_H
