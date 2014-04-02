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
 * \file VideoSourceMulti.h
 * \brief Declaration of VideoSourceMulti class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * VideoSourceMulti is the source of images in mcptam. It can receive images from multiple
 * cameras, optionally in synchronized groups. It also gets the camera calibration
 * parameters and calibrated poses, which are available through getter methods.
 *
 * Virtually nothing in this class is from PTAM's VideoSource, except for the name of 
 * the GetAndFillFrameBW and GetAndFillFrameBWandRGB functions.
 *
 ****************************************************************************************/

#ifndef __MULTI_VIDEO_SOURCE_H
#define __MULTI_VIDEO_SOURCE_H

#include <mcptam/Types.h>

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <mcptam/CameraGroupSubscriber.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <atomic>

/** @brief Receives images, camera parameters and calibrated poses from multiple cameras
 * 
 *  The cameras can be grouped by synchronization, meaning that the images should have been
 *  triggered at the same time and so their timestamps should be nearly identical. Images
 *  from a camera group all need to arrive, otherwise the acquisition doesn't succeed and
 *  the system waits for the next set of synchronized images. */
class VideoSourceMulti
{
public:
  /** @brief Sets up list of camera groups from the "cam_group_list" ROS parameter, see CamGroupsParser class
   *  @param bGetPose Should get the calibrated pose of the cameras on a separate topic? */
  VideoSourceMulti(bool bGetPoseSeparately = false);
 
  /** @brief Destructor */
  ~VideoSourceMulti();
 
  // Acquisition functions
  /** @brief Acquires a new set of images from the camera group whose data arrives first, outputs it in greyscale and color RGB formats
   *  @param timeout How long to wait for acquisition before returning
   *  @param [out] mImBW The map that will contain the acquired greyscale images
   *  @param [out] mImRGB The map that will contain the acquired color RGB images
   *  @return Did the acquisition succeed? */
  bool GetAndFillFrameBWandRGB(ros::WallDuration timeout, ImageBWMap &mImBW, ImageRGBMap &mImRGB);
  
  /** @brief Acquires a new image from the specified camera, outputs it in greyscale and color RGB formats
   *  @param camName The name of the camera to get images from
   *  @param timeout How long to wait for acquisition before returning
   *  @param [out] imBW The acquired greyscale image
   *  @param [out] imRGB The acquired color RGB image
   *  @return Did the acquisition succeed? */
  bool GetAndFillFrameBWandRGB(std::string camName, ros::WallDuration timeout, CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
  
  /** @brief Acquires a new set of images from the camera group whose data arrives first, outputs it in greyscale format
   *  @param timeout How long to wait for acquisition before returning
   *  @param [out] mImBW The map that will contain the acquired greyscale images
   *  @return Did the acquisition succeed? */
  bool GetAndFillFrameBW(ros::WallDuration timeout, ImageBWMap &mImBW, ros::Time& timestamp);
  
  /** @brief Acquires a new image from the specified camera, outputs it in greyscale format
   *  @param camName The name of the camera to get images from
   *  @param timeout How long to wait for acquisition before returning
   *  @param [out] imBW The acquired greyscale image
   *  @return Did the acquisition succeed? */
  bool GetAndFillFrameBW(std::string camName, ros::WallDuration timeout, CVD::Image<CVD::byte> &imBW);
  
  // Info getter functions
  /** @brief Get the total size of all camera images, tiled side by side
   * 
   *  Makes the assumption that the tiling should be 2 columns. Could be changed if necessary.
   *  @param bFullSize If true, uses the full scale size of the camera images, otherwise uses the current size
   *  @return The width and height of the area taken up by the tiled images */
  CVD::ImageRef GetTotalSize(bool bFullSize);
  
  /** @brief Get the individual sizes of the current camera images
   *  @return A map of camera names => image sizes */
  ImageRefMap GetSizes(){ return mmSizes; }
  
  /** @brief Get the individual sizes of the current camera images times any binning being applied
   *  @return A map of camera names => image sizes */
  ImageRefMap GetFullScaleSizes(){ return mmFullScaleSizes; }
  
  /** @brief Get the individual sizes of the camera images that were used to calibrate the cameras
   *  @return A map of camera names => image sizes */
  ImageRefMap GetCalibSizes(){ return mmCalibSizes; }
  
  /** @brief Get the coordinates of the top left of each image necessary to recreate the tiling layout used in GetTotalSize
   *  @return A map of camera names => coordinates */
  ImageRefMap GetDrawOffsets(){ return mmDrawOffsets; }
  
  /** @brief Get the calibrated parameters of each camera
   *  @return A map of camera names => parameter vectors */
  ParamMap GetParams(){ return mmParams; }
  
  /** @brief Get the calibrated pose of each camera
   *  @return A map of camera names => poses */
  SE3Map GetPoses(){ return mmPoses; }
  
  /** @brief Call the service to set the camera poses in the respective camera_infos 
   *  @param mPoses A map of the extrinsic poses for the cameras
   *  @return Successfully saved all poses */
  bool SavePoses(const SE3Map& mPoses);
  
  // Utility functions
  /** @brief Get the number of camera groups
   *  @return Number of groups */
  int GetNumGroups(){ return mvpCamGroups.size(); }
  /** @brief Get the number of cameras overall
   *  @return Number of cameras */
  int GetNumCams(){ return mvCamNames.size(); }
  /** @brief Get the names of all the cameras
   *  @return Vector of camera names */
  std::vector<std::string> GetCamNames(){ return mvCamNames; }
  
protected:
  
  // Acquisition worker functions
  /** @brief This is the function that actually does the image acquisition
   * 
   *  Launches a waiting thread for each camera group, continuously checks mmpLastImages to 
   *  see if any of them have successfully gotten their images.
   *  @param timeout How long to wait before bailing out
   *  @param camName The name of a specific camera to get images from, leave empty string to 
   *                 get images from earliest group to complete 
   *  @return Did the acquisition succeed? */
  bool NewAcquistion(ros::WallDuration timeout, std::string camName = "");
  
  /** @brief Wait for a specific group to finish acquiring images
   *  
   *  Once images have been acquired, checks to see any other group
   *  has beaten it before recording its images. Launched in a seperate
   *  thread because it calls the group's GetNewImage function, which 
   *  is blocking.
   *  @param nGroupIdx The index of the group being worked on
   *  @param pThreadActive Setting this to false will terminate the acquisition */
  void WaitForGroup(int nGroupIdx, bool* pThreadActive);
    
  // Misc functions
  /// Calculate the draw offsets that are returned by GetDrawOffsets
  void CalcDrawOffsets();
  
  /** @brief Extract data from the CameraInfo message and fill out member variables
   *  @param infoMsg The calibration message from the camera
   *  @param cameraName The name of the camera that sent the message */
  void RecordInfo(sensor_msgs::CameraInfo infoMsg, std::string cameraName, bool bGetPoseSeparately);
  
  std::vector<std::string> mvCamNames;                ///< The names of all the cameras we're getting images from
  std::vector<CameraGroupSubscriber*> mvpCamGroups;   ///< Vector of camera groups
  ImageRefMap mmSizes;               ///< %Map of current camera image sizes
  ImageRefMap mmFullScaleSizes;      ///< %Map of current camera image sizes times any binning being applied
  ImageRefMap mmCalibSizes;          ///< %Map of image sizes that were used for calibration
  ImageRefMap mmBinnings;            ///< %Map of binning values
  ImageRefMap mmDrawOffsets;         ///< %Map of drawing offsets 
  ImagePtrMap mmpLastImages;         ///< %Map of images most recently received
  ParamMap mmParams;                 ///< %Map of camera parameters
  SE3Map mmPoses;                    ///< %Map of camera poses
  
  ImageBWMap mmWorkspaceBW;          ///< %Map of greyscale images that are overwritten and not deleted, used as a scratchpad for image acquisition
  ImageRGBMap mmWorkspaceRGB;        ///< %Map of color RGB images that are overwritten and not deleted, used as a scratchpad for image acquisition
  
  std::atomic<unsigned int> mnGroupsFinished;   ///< Allows threads to determine which group acquired first
  boost::mutex mImageMutex;           ///< Guards mmpLastImages from thread access problems
  bool mbWaitActive;          ///< Setting to false will terminate acquisition threads
  
  // Threadpool stuff  
  /** @brief Threadpool component
   * Implemented using boost ASIO according to http://think-async.com/Asio/Recipes */
  boost::asio::io_service mIOService; 
  
  /** @brief Threadpool component
   * Implemented using boost ASIO according to http://think-async.com/Asio/Recipes */
  boost::asio::io_service::work mWork;
  
  /** @brief Threadpool component
   * Implemented using boost ASIO according to http://think-async.com/Asio/Recipes */
  boost::thread_group mThreads;
  
  ros::NodeHandle mNodeHandle;  ///< ROS node handle with global namespace
  ros::NodeHandle mNodeHandlePriv;  ///< ROS node handle with private namespace
  
  std::string mSetInfoTopic;    ///< Name of the SetInfo ROS topic
};

#endif

