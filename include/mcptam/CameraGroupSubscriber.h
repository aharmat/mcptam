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
 * \file CameraGroupSubscriber.h
 * \brief Declaration of CameraGroupSubscriber class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * CameraGroupSubsriber subscribes to data being published by all cameras in one
 * camera group. It subscribes to image, calibration, and pose data, and offers
 * functions for retrieving this data.
 *
 ****************************************************************************************/

#ifndef __CAMERA_GROUP_SUBSCRIBER_H
#define __CAMERA_GROUP_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <ros/callback_queue.h>

typedef std::map<std::string, cv_bridge::CvImageConstPtr> ImagePtrMap;
typedef std::map<std::string, sensor_msgs::CameraInfo> InfoMap;
typedef std::map<std::string, geometry_msgs::Pose> PoseMap;

/** @brief Subscribes to data being published by all cameras in one camera group 
 * 
 * Image, calibration, and pose data is received, with the option to receive pose data on a 
 * separate topic. Functions are available for retrieving this data. 
 * The images need to be published with approximately the same time stamp, i.e., they should have been
 * triggered together. An approximate time policy is used for receiving multiple image messages
 * so if the messages have slightly different timestamps the code will still work. Look at the 
 * ApproximateTime policy in ROS's message_filters class. */
class CameraGroupSubscriber
{
public:
  /** @param cameras The names of the cameras making up this group. A maximum of 8 cameras are supported at this time.
   *  @param getPose Should I get the poses of the cameras? Some camera types don't publish a pose so we don't want to wait indefinitely. */
  CameraGroupSubscriber(std::vector<std::string> vCameraNames, bool bGetPoseSeparately = false);
  ~CameraGroupSubscriber();
  
  /** @brief Grab a new set of images from all the cameras in the group
   *  @param active Pointer to a boolean which can terminate acquisition by being set to false
   *  @return A map of camera names => CvImageConstPtr, from which images can be extracted in OpenCV format */
  ImagePtrMap GetNewImage(bool* bActive);
  
  /** @brief Returns the average timestamp of the last acquisition */
  ros::Time GetLastTimestamp(){ return mLastTimestamp; }
  
  /** @brief Get the internal calibration from all the cameras
   * 
   *  This function returns a saved CameraInfo message it got during initialization, so if the camera starts sending 
   *  a different calibration after the constructor has been called, it won't be reflected in the output of this function.
   *  @return A map of camera names => CameraInfo, from which camera parameters can be extracted.  */
  InfoMap GetInfo(){ return mmSavedInfos; }
  
  /** @brief Get the pose from all the cameras
   * 
   *  This function returns a saved Pose message it got during initialization, so if the camera starts sending 
   *  a different pose after the constructor has been called, it won't be reflected in the output of this function.
   *  @return A map of camera names => Pose, from which camera poses can be extracted.  */
  PoseMap GetPose(){ return mmSavedPoses; }
  
  bool ContainsCamera(std::string camName)
  { 
    return std::find(mvCameraNames.begin(), mvCameraNames.end(), camName) != mvCameraNames.end();
  }
  
protected:

  /** @brief This will be called by the ROS subscriber whenever images from all the cameras are received
   * 
   *  If fewer than 8 cameras are being subscribed to, the unused message slots are simply duplicates of the first message, but that's ok because they're just pointers
   *  so there's not much of a performance hit. The function puts the messages into CvImageConstPtr objects and saves them in lastImage.
   *  @param msg1 Image message from camera 1
   *  @param msg2 Image message from camera 2
   *  @param msg3 Image message from camera 3
   *  @param msg4 Image message from camera 4
   *  @param msg5 Image message from camera 5
   *  @param msg6 Image message from camera 6
   *  @param msg7 Image message from camera 7
   *  @param msg8 Image message from camera 8 */
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::ImageConstPtr& msg3, const sensor_msgs::ImageConstPtr& msg4,
                     const sensor_msgs::ImageConstPtr& msg5, const sensor_msgs::ImageConstPtr& msg6, const sensor_msgs::ImageConstPtr& msg7, const sensor_msgs::ImageConstPtr& msg8);
  
  /** @brief This will be called by the ROS subscriber whenever a CameraInfo message arrives from any of the cameras
   * 
   *  Saves the message into the savedInfo map.
   *  @param infoMsg The actual CameraInfo message
   *  @param cameraName The camera that this message came from */
  void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr& infoMsg, std::string cameraName);
  
  /** @brief This will be called by the ROS subscriber whenever a Pose message arrives from any of the cameras
   * 
   *  Saves the message into the savedPose map.
   *  @param poseMsg The actual Pose message
   *  @param cameraName The camera that this message came from */
  void PoseCallback(const geometry_msgs::Pose::ConstPtr& poseMsg, std::string cameraName);
  
  /// We're using the ApproximateTime synchronization policy supporting up to 8 images
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproxTimePolicy;  

  ros::NodeHandle mNodeHandle;  ///< ROS node handle with global namespace
  ros::NodeHandle mNodeHandlePriv;  ///< ROS node handle with private namespace
  image_transport::ImageTransport* mpImageTransport;      ///< The image transport, used to enable image compression
  ros::CallbackQueue mCallbackQueue;         ///< Custom callback queue so we can spin just for our own callbacks instead of a node-wide spin
  
  unsigned mNumCams;                       ///< The number of cameras in this group
  std::vector<std::string> mvCameraNames;   ///< Vector of camera names in this group
  bool mbDynamicSync;                      ///< Dynamically set the inter message lower bound of the synchronizer based on observed framerate?
  
  message_filters::Synchronizer<ApproxTimePolicy>* mpSync;   ///< The message synchronizer, using the ApproxTimePolicy we defined 
  std::vector< image_transport::SubscriberFilter* > mvpImageSubs;   ///< Vector of image subscribers (one per camera) as SubscriberFilter pointers so they can be chained with the synchronizer
  std::vector< ros::Subscriber > mvInfoSubs;    ///< Vector of regular subscribers (one per camera) for getting camera info, these don't need to be synchronized
  std::vector< ros::Subscriber > mvPoseSubs;    ///< Vector of regular subscribers (one per camera) for getting camera pose, these don't need to be synchronized
  
  ros::Time mLastTimestamp;    ///< Last (average) timestamp of the acquired images
  ImagePtrMap mmLastImages;    ///< A map of camera name => CvImageConstPtr from the last acquisition
  InfoMap mmSavedInfos;        ///< A map of camera name => CameraInfo that we saved from the initialization
  PoseMap mmSavedPoses;        ///< A map of camera name => Pose that we saved from the initialization
    
};

#endif
