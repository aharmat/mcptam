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
 * \file VideoSourceSingle.h
 * \brief Declaration of VideoSourceSingle class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * VideoSourceSingle is the source of images for CameraCalibrator. It can
 *receive images
 * from just one camera. It also gets the camera calibration
 * parameters and calibrated pose, which are available through getter methods.
 *
 * Virtually nothing in this class is from PTAM's VideoSource, except for the
 *name of
 * the GetAndFillFrameBW and GetAndFillFrameBWandRGB functions.
 *
 ****************************************************************************************/

#ifndef MCPTAM_VIDEOSOURCESINGLE_H
#define MCPTAM_VIDEOSOURCESINGLE_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <string>

#include <TooN/TooN.h>
#include <TooN/se3.h>

#include <opencv2/core/core.hpp>

class VideoSourceSingle
{
public:
  /** @brief Sets up message subscribers, reads come camera parameters
   *
   *  The following ROS params are available to set: \n
   *  cam_name: The name of the camera node \n
   *  image_topic: The image topic name \n
   *  info_topic: The CameraInfo topic name \n
   *  pose_topic: The fixed pose topic name
   *  @param bGetPose Should get the calibrated pose of the cameras? */
  explicit VideoSourceSingle(bool bGetPoseSeparately = false);

  /** @breif Destructor */
  ~VideoSourceSingle();

  // Acquisition functions
  /** @brief Acquires a new image from the camera, outputs it in greyscale and
   * color RGB formats
   *  @param timeout How long to wait for acquisition before returning
   *  @param [out] imBW The acquired greyscale image
   *  @param [out] imRGB The acquired coor RGB image
   *  @return Did the acquisition succeed? */
  bool GetAndFillFrameBWandRGB(ros::WallDuration timeout, CVD::Image<CVD::byte> &imBW,
                               CVD::Image<CVD::Rgb<CVD::byte>> &imRGB);

  /** @brief Acquires a new image from the camera, outputs it in greyscale
   * format
   *  @param timeout How long to wait for acquisition before returning
   *  @param [out] imBW The acquired greyscale image
   *  @return Did the acquisition succeed? */
  bool GetAndFillFrameBW(ros::WallDuration timeout, CVD::Image<CVD::byte> &imBW);
  bool GetAndFillFrameBW(ros::WallDuration timeout, CVD::Image<CVD::byte> &imBW, ros::Time &timestamp);

  /** @brief Get the size of the current camera image
   *  @return The width and height of the current image */
  CVD::ImageRef GetSize()
  {
    return mirSize;
  }

  /** @brief Get the size of the current camera image scaled by binning
   *  @return The width and height of the current image scaled by binning */
  CVD::ImageRef GetFullScaleSize()
  {
    return mirFullScaleSize;
  }

  /** @brief Get the size of the image the camera was calibrated with
   *  @return The width and height of the calibration image */
  CVD::ImageRef GetCalibSize()
  {
    return mirCalibSize;
  }

  /** @brief Get the calibrated parameters
   *  @return The parameter vector */
  TooN::Vector<9> GetParams()
  {
    return mv9Params;
  }

  /** @brief Get the fixed relative pose of the camera
   *  @return The pose */
  TooN::SE3<> GetPose()
  {
    return mse3Pose;
  }

  /** @brief Get the name of the camera node
   *  @return The name */
  std::string GetCamName()
  {
    return mCamName;
  }

  /** @brief Save the intrinsic camera parameters
   *  @param v9Params Vector of the Taylor model camera parameters
   *  @return Successfully saved the camera parameters */
  bool SaveParams(const TooN::Vector<9> &v9Params);

private:
  /** @brief Acquire a new frame
   *  @param timeout How long to wait before bailing
   *  @return Did the acquisition succeed? */
  bool NewAcquisition(ros::WallDuration timeout);

  // Callback functions
  /** @brief Called from the image subscriber, stores the image for later
   * conversion
   *  @param imageMsg The image message */
  void ImageCallback(const sensor_msgs::ImageConstPtr &imageMsg);

  /** @brief Called from the info subscriber, fills out various member variables
   *  @param infoMsg The CameraInfo message */
  void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &infoMsg);

  /** @brief Called from the pose subscriber, saves recieved pose
   *  @param poseMsg The pose message */
  void PoseCallback(const geometry_msgs::Pose::ConstPtr &poseMsg);

  CVD::ImageRef mirSize;           ///< The current image size
  CVD::ImageRef mirFullScaleSize;  ///< The current image size scaled by any
  /// binning being applied
  CVD::ImageRef mirCalibSize;  ///< The image sized used for calibration
  CVD::ImageRef mirBinning;    ///< The current bigging value

  cv_bridge::CvImageConstPtr mCVPtr;  ///< Used to get an OpenCV Mat from a ROS image
  bool mbAcquiredImage;               ///< Set to true when a new image has been received
  bool mbAcquiredInfo;                ///< Set to true when a new info message has been received
  bool mbAcquiredPose;                ///< Set to true when a new pose message has been received

  TooN::Vector<9> mv9Params;  ///< The TaylorCamera parameters
  TooN::SE3<> mse3Pose;       ///< The fixed camera pose

  std::string mCamName;       ///< The name of the camera node
  std::string mSetInfoTopic;  ///< Name of the ROS SetInfo topic for the cameras

  ros::NodeHandle mNodeHandle;      ///< ROS global node handle
  ros::NodeHandle mNodeHandlePriv;  ///< ROS private node handle

  image_transport::ImageTransport *mpImageTransport;  ///< Enables use of compression over image topic
  image_transport::Subscriber mImageSub;              ///< The image subsriber
  ros::Subscriber mInfoSub;                           ///< The CameraInfo subscriber
  ros::Subscriber mPoseSub;                           ///< The pose subscriber

  ros::CallbackQueue mCallbackQueue;  ///< Installed callback queue for ROS

  bool mbGetPoseSeparately;  ///< Whether the extrinsic pose of the cameras are
  /// sent as a separate pose message or in camera_info
};

#endif  // MCPTAM_VIDEOSOURCESINGLE_H
