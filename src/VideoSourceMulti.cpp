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

//=========================================================================================
//
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/VideoSourceMulti.h>
#include <mcptam/Utility.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Empty.h>

using namespace TooN;

VideoSourceMulti::VideoSourceMulti(bool bGetPoseSeparately) : mWork(mIOService), mNodeHandlePriv("~")
{
  ROS_INFO("VideoSourceMulti: Initializing");

  mNodeHandlePriv.param<std::string>("set_info_topic", mSetInfoTopic, "set_camera_info");

  XmlRpc::XmlRpcValue xmlCamGroupList;
  if (!mNodeHandlePriv.getParam("cam_group_list", xmlCamGroupList))
  {
    ROS_ERROR("VideoSourceMulti: Need a 'cam_group_list' parameter to be set before continuing!");
    return;
  }

  std::vector<std::vector<std::string>> vCamGroupStrings;
  bool bParsed = util::Parse(xmlCamGroupList, vCamGroupStrings);

  if (!bParsed)
  {
    ROS_ERROR("VideoSourceMulti: Error parsing camera groups");
    ros::shutdown();
    return;
  }

  for (unsigned i = 0; i < vCamGroupStrings.size(); ++i)
  {
    for (unsigned j = 0; j < vCamGroupStrings[i].size(); ++j)
      ROS_INFO_STREAM("VideoSourceMulti: Creating camera group from " << vCamGroupStrings[i][j]);

    mvpCamGroups.push_back(new CameraGroupSubscriber(
                             vCamGroupStrings[i], bGetPoseSeparately));  // new subscriber group (with pose subscription)
    mvCamNames.insert(mvCamNames.end(), vCamGroupStrings[i].begin(), vCamGroupStrings[i].end());
  }

  ROS_INFO("VideoSourceMulti: Got all CamGroups");

  for (unsigned i = 0; i < mvpCamGroups.size(); ++i)
  {
    InfoMap mInfo = mvpCamGroups[i]->GetInfo();
    for (InfoMap::iterator it = mInfo.begin(); it != mInfo.end(); it++)
    {
      RecordInfo(it->second, it->first, bGetPoseSeparately);
    }

    if (bGetPoseSeparately)
    {
      PoseMap mPose = mvpCamGroups[i]->GetPose();
      for (PoseMap::iterator it = mPose.begin(); it != mPose.end(); it++)
      {
        mmPoses[it->first] = util::PoseMsgToSE3(it->second);
      }
    }
  }

  ROS_INFO_STREAM("VideoSourceMulti: Got all parameters " << (bGetPoseSeparately ? " and separate poses" : ""));

  mbWaitActive = true;
  for (unsigned i = 0; i < mvpCamGroups.size(); ++i)
  {
    ImagePtrMap mpImages =
      mvpCamGroups[i]->GetNewImage(&mbWaitActive);  // get a set of images so we can extract image size info
    for (ImagePtrMap::iterator it = mpImages.begin(); it != mpImages.end(); it++)
    {
      std::string camName = it->first;
      mmSizes[camName] = CVD::ImageRef(it->second->image.cols, it->second->image.rows);
      CVD::ImageRef irBinning = mmBinnings[camName];
      mmFullScaleSizes[camName] =
        CVD::ImageRef(it->second->image.cols * irBinning.x, it->second->image.rows * irBinning.y);

      // Prepare workspace variables
      mmWorkspaceBW[camName] = CVD::Image<CVD::byte>();
      mmWorkspaceRGB[camName] = CVD::Image<CVD::Rgb<CVD::byte>>();
    }
  }

  ROS_INFO("VideoSourceMulti: Got first set of images");

  CalcDrawOffsets();

  mbWaitActive = false;
  for (unsigned i = 0; i < mvpCamGroups.size(); ++i)
  {
    mThreads.create_thread(boost::bind(&boost::asio::io_service::run, &mIOService));
  }
};

VideoSourceMulti::~VideoSourceMulti()
{
  mbWaitActive = false;        // force stop acquisitions
  ros::Duration(1.0).sleep();  // allow any camera groups to finish

  // Kill threadpool
  mIOService.stop();
  mThreads.join_all();

  for (unsigned i = 0; i < mvpCamGroups.size(); ++i)
    delete mvpCamGroups[i];

  mvpCamGroups.clear();
}

// Extract data from the CameraInfo message and fill out member variables
void VideoSourceMulti::RecordInfo(sensor_msgs::CameraInfo infoMsg, std::string cameraName, bool bGetPoseSeparately)
{
  if (infoMsg.distortion_model != "taylor")
  {
    ROS_ERROR_STREAM(cameraName << " not calibrated with Taylor distortion model!");
    ros::shutdown();
  }

  if (infoMsg.binning_x == 0)
    infoMsg.binning_x = 1;

  if (infoMsg.binning_y == 0)
    infoMsg.binning_y = 1;

  // The parameters are:
  // 0 - a0 coefficient
  // 1 - a2 coefficient
  // 2 - a3 coefficient
  // 3 - a4 coefficient
  // 4 - image center xc
  // 5 - image center yc
  // 6 - affine transform param c
  // 7 - affine transform param d
  // 8 - affine transform param e

  double c = infoMsg.K[0];
  double d = infoMsg.K[1];
  double e = infoMsg.K[3];

  double xc = infoMsg.K[2];
  double yc = infoMsg.K[5];

  double a0 = infoMsg.D[0];
  double a2 = infoMsg.D[1];
  double a3 = infoMsg.D[2];
  double a4 = infoMsg.D[3];

  mmParams[cameraName] = makeVector(a0, a2, a3, a4, xc, yc, c, d, e);
  mmBinnings[cameraName] = CVD::ImageRef(infoMsg.binning_x, infoMsg.binning_y);
  mmCalibSizes[cameraName] = CVD::ImageRef(infoMsg.width, infoMsg.height);

  if (!bGetPoseSeparately)  // pose is located in the info message, so extract it
  {
    TooN::Matrix<3> m3Rot;
    for (int j = 0; j < 3; ++j)
    {
      for (int i = 0; i < 3; ++i)
      {
        m3Rot(j, i) = infoMsg.R[j * 3 + i];
      }
    }
    TooN::Vector<3> v3Trans = TooN::makeVector(infoMsg.P[3], infoMsg.P[7], infoMsg.P[11]);

    // Check the rotation matrix to see if it is a proper rotation
    TooN::Matrix<3> m3Result = m3Rot * m3Rot.T();  // this should be close to identity
    TooN::Vector<3> v3Ones = makeVector(1, 1, 1);
    TooN::Vector<3> v3Diff = v3Ones - m3Result * v3Ones;  // should be close to zero

    if (v3Diff * v3Diff > 1e-4)
    {
      m3Rot = Identity;
      v3Trans = Zeros;
      ROS_WARN_STREAM("VideoSourceMulti: The rotation matrix inside the CameraInfo message of " << cameraName
                      << " is invalid!");
      ROS_WARN_STREAM("VideoSourceMulti: Perhaps you meant to get the camera pose separately but forgot to set the "
                      "get_pose_separately parameter to true?");
      ROS_WARN_STREAM("VideoSourceMulti: Defaulting to IDENTITY transformation.");
      ROS_WARN_STREAM("VideoSourceMulti: If you are calibrating the camera, continue without worry. If you are not, "
                      "things will be messed up.");
    }

    TooN::SE3<> se3Pose(TooN::SO3<>(m3Rot), v3Trans);

    // Need to invert the pose before storing since it is stored in the "usual" format in the
    // info message (ie pose of the camera IN another reference frame, such that x_r = P_c * x_c
    // where x_r is point in reference frame, P_c is camera's pose, x_c is point in camera frame)
    // whereas the KeyFrames where these poses end up store the poses in the inverse manner
    // (ie x_c = P_c * x_r)
    mmPoses[cameraName] = se3Pose.inverse();
  }
}

// Get the total size of all camera images, tiled side by side
CVD::ImageRef VideoSourceMulti::GetTotalSize(bool bFullSize)
{
  CVD::ImageRef irSize;
  int nCounter = 0;

  ImageRefMap &mSizesToUse = bFullSize ? mmFullScaleSizes : mmSizes;

  for (ImageRefMap::iterator it = mSizesToUse.begin(); it != mSizesToUse.end(); it++, nCounter++)
  {
    if (nCounter == 0)
      irSize = it->second;
    else if (nCounter < 2)
      irSize.x += it->second.x;
    else if (nCounter % 2 == 0)
      irSize.y += it->second.y;
  }

  return irSize;
}

// Calculate the draw offsets that are returned by GetDrawOffsets
void VideoSourceMulti::CalcDrawOffsets()
{
  mmDrawOffsets.clear();
  int nCounter = 0;

  for (unsigned i = 0; i < mvCamNames.size(); ++i, ++nCounter)
  {
    std::string camName = mvCamNames[i];
    std::string camNameLast = (i > 0) ? mvCamNames[i - 1] : camName;

    if (nCounter == 0)
      mmDrawOffsets[camName] = CVD::ImageRef(0, 0);
    else if (nCounter > 0)
    {
      mmDrawOffsets[camName] = mmDrawOffsets[camNameLast];

      if (nCounter % 4 == 1)
      {
        mmDrawOffsets[camName].x += mmSizes[camNameLast].x;
      }
      else if (nCounter % 4 == 2)
      {
        mmDrawOffsets[camName].y += mmSizes[camNameLast].y;
      }
      else if (nCounter % 4 == 3)
      {
        mmDrawOffsets[camName].x = 0;
      }
      else
      {
        mmDrawOffsets[camName].x = 0;
        mmDrawOffsets[camName].y += mmSizes[camNameLast].y;
      }
    }
  }
}

// Acquires a new image from the specified camera, outputs it in greyscale and color RGB formats
bool VideoSourceMulti::GetAndFillFrameBWandRGB(std::string camName, ros::WallDuration timeout,
    CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte>> &imRGB)
{
  bool bSuccess = NewAcquistion(timeout, camName);

  if (!bSuccess)
    return false;

  util::ConversionRGB(mmpLastImages[camName]->image, mmWorkspaceRGB[camName]);
  util::ConversionBW(mmpLastImages[camName]->image, mmWorkspaceBW[camName]);

  imRGB = mmWorkspaceRGB[camName];
  imBW = mmWorkspaceBW[camName];

  return true;
}

// Acquires a new set of images from the camera group whose data arrives first, outputs it in greyscale and color RGB
// formats
bool VideoSourceMulti::GetAndFillFrameBWandRGB(ros::WallDuration timeout, ImageBWMap &imBW, ImageRGBMap &imRGB)
{
  imBW.clear();
  imRGB.clear();

  bool bsuccess = NewAcquistion(timeout);

  if (!bsuccess)
    return false;

  for (ImagePtrMap::iterator it = mmpLastImages.begin(); it != mmpLastImages.end(); it++)
  {
    util::ConversionRGB(it->second->image, mmWorkspaceRGB[it->first]);
    util::ConversionBW(it->second->image, mmWorkspaceBW[it->first]);

    imRGB[it->first] = mmWorkspaceRGB[it->first];
    imBW[it->first] = mmWorkspaceBW[it->first];
  }

  return true;
}

// Acquires a new image from the specified camera, outputs it in greyscale format
bool VideoSourceMulti::GetAndFillFrameBW(std::string camName, ros::WallDuration timeout, CVD::Image<CVD::byte> &imBW)
{
  bool success = NewAcquistion(timeout, camName);

  if (!success)
    return false;

  util::ConversionBW(mmpLastImages[camName]->image, mmWorkspaceBW[camName]);
  imBW = mmWorkspaceBW[camName];

  return true;
}

// Acquires a new set of images from the camera group whose data arrives first, outputs it in greyscale format
bool VideoSourceMulti::GetAndFillFrameBW(ros::WallDuration timeout, ImageBWMap &imBW, ros::Time &timestamp)
{
  imBW.clear();
  bool success = NewAcquistion(timeout);

  if (!success)
    return false;

  timestamp = mLastTimestamp;

  for (ImagePtrMap::iterator it = mmpLastImages.begin(); it != mmpLastImages.end(); it++)
  {
    util::ConversionBW(it->second->image, mmWorkspaceBW[it->first]);
    imBW[it->first] = mmWorkspaceBW[it->first];
  }

  return true;
}

// Wait for a specific group to finish acquiring images
void VideoSourceMulti::WaitForGroup(int nGroupIdx, bool *pThreadActive)
{
  ImagePtrMap mpTempImages = mvpCamGroups[nGroupIdx]->GetNewImage(pThreadActive);

  if (!mpTempImages.empty())
  {
    // We got our images, need to check if some other group has beaten us to the punch
    if (++mnGroupsFinished == 1)  // we are the first to finish
    {
      boost::mutex::scoped_lock lock(mImageMutex);
      mmpLastImages = mpTempImages;
      mLastTimestamp = mvpCamGroups[nGroupIdx]->GetLastTimestamp();
    }
  }
}

// This is the function that actually does the image acquisition
bool VideoSourceMulti::NewAcquistion(ros::WallDuration timeout, std::string camName)
{
  mmpLastImages.clear();

  // Load tasks into threadpool queue
  // By creating the tasks in this loop there is a slight preference given
  // to the earlier camera groups, since their thread will start
  // executing earlier. Hopefully this time isn't too great....

  mnGroupsFinished = 0;
  mbWaitActive = true;
  for (unsigned i = 0; i < mvpCamGroups.size(); ++i)
  {
    mIOService.post(boost::bind(&VideoSourceMulti::WaitForGroup, this, i, &mbWaitActive));
  }

  ros::WallTime start = ros::WallTime::now();
  while (ros::ok())
  {
    boost::mutex::scoped_lock lock(mImageMutex);

    if (camName != "")  // looking for a particular image
    {
      if (mmpLastImages.count(camName))  // got the image we wanted
        break;
    }
    else
    {
      if (mmpLastImages.size() > 0)  // got some images!
        break;
    }

    if (timeout != ros::WallDuration(0) && (ros::WallTime::now() - start) > timeout)  // timed out
      break;
  }

  ros::WallDuration grabDur = ros::WallTime::now() - start;
  ROS_DEBUG_STREAM(">>>>>>>>>>>> Grab dur: " << grabDur);

  mbWaitActive = false;  // this will terminate all remaining WaitForGroup threads

  if (camName != "")
    return mmpLastImages.count(camName);
  else
    return !mmpLastImages.empty();
}

// Call the service to set the camera poses in the respective camera_infos
bool VideoSourceMulti::SavePoses(const SE3Map &mPoses)
{
  // Get the CameraInfo messages from all the cameras first. This way
  // we don't mess with the intrinsic calibration parameters. Then we'll
  // fill in the new poses, and call the appropriate service.

  InfoMap mAllInfo;
  for (unsigned i = 0; i < mvpCamGroups.size(); ++i)
  {
    InfoMap mInfo = mvpCamGroups[i]->GetInfo();
    mAllInfo.insert(mInfo.begin(), mInfo.end());
  }

  bool bAllSuccess = true;

  for (SE3Map::const_iterator se3_it = mPoses.begin(); se3_it != mPoses.end(); ++se3_it)
  {
    std::string cameraName = se3_it->first;
    ROS_ASSERT(mAllInfo.count(cameraName));

    sensor_msgs::SetCameraInfo srv;
    srv.request.camera_info = mAllInfo[cameraName];

    // We need to store the pose in the inverse sense of how it is kept in mcptam
    // since the ROS camera convention specifies it this way
    TooN::SE3<> se3Pose = se3_it->second.inverse();
    TooN::Matrix<3> m3Rot = se3Pose.get_rotation().get_matrix();
    TooN::Vector<3> v3Trans = se3Pose.get_translation();

    for (int j = 0; j < 3; ++j)
    {
      for (int i = 0; i < 3; ++i)
      {
        srv.request.camera_info.R[j * 3 + i] = m3Rot(j, i);
      }
    }

    srv.request.camera_info.P[3] = v3Trans[0];
    srv.request.camera_info.P[7] = v3Trans[1];
    srv.request.camera_info.P[11] = v3Trans[2];

    ros::ServiceClient client =
      mNodeHandle.serviceClient<sensor_msgs::SetCameraInfo>(std::string(cameraName + "/" + mSetInfoTopic));

    if (client.call(srv) && srv.response.success)
    {
      ROS_INFO_STREAM("VideoSourceMulti: Saved pose of " << cameraName);
      ROS_INFO_STREAM(se3Pose);
    }
    else
    {
      ROS_ERROR_STREAM("VideoSourceMulti: Failed to save pose of " << cameraName);
      bAllSuccess = false;
    }
  }

  return bAllSuccess;
}
