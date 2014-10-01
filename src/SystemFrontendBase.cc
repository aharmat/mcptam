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
// Parts of this code are from the original PTAM, which are
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/SystemFrontendBase.h>
#include <mcptam/Tracker.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/SystemInfo.h>
#include <mcptam/TrackerState.h>
#include <mcptam/Utility.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/TrackerCovInfo.h>
#include <mcptam/Map.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cvd/utility.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>

using namespace TooN;

SystemFrontendBase::SystemFrontendBase(std::string windowName, bool bFullSize)
: SystemBase(windowName, bFullSize, !IsHeadless())
, mpTracker(NULL)  // derived class will need to allocate object because it needs pointer to appropriate map maker
, mImageTransport(mNodeHandlePriv)
{
  int nGroups = mpVideoSourceMulti->GetNumGroups();
  int nCams = mpVideoSourceMulti->GetNumCams();
  
  if(nCams == 0)
  {
    ROS_FATAL("System: No cameras detected, shutting down.");
    ros::shutdown();
    return;
  }
  
  if(nGroups == nCams && nCams > 1)  // no group has more than one camera and more than 1 group
  {
    ROS_FATAL("System: More than one camera group and none have more than one camera, won't be able to initialize");
    ros::shutdown();
    return;
  }
  
  mInitSystemServer = mNodeHandlePriv.advertiseService("init", &SystemFrontendBase::InitSystemCallback, this);
  mResetSystemServer = mNodeHandlePriv.advertiseService("reset", &SystemFrontendBase::ResetSystemCallback, this);
  
  mSystemInfoPub = mNodeHandlePriv.advertise<mcptam::SystemInfo>("system_info", 1);
  mStatePub = mNodeHandlePriv.advertise<mcptam::TrackerState>("tracker_state", 1);
  mPoseWithCovPub = mNodeHandlePriv.advertise<geometry_msgs::PoseWithCovarianceStamped>("tracker_pose_cov", 1);
  mPoseArrayPub = mNodeHandlePriv.advertise<geometry_msgs::PoseArray>("tracker_pose_array", 1);
  mSmallImagePointsPub = mNodeHandlePriv.advertise<pcl::PointCloud<pcl::PointXYZ> >("tracker_small_image_points", 1);
  mSmallImagePub = mImageTransport.advertise("tracker_small_image", 1);
  
  mPoseCovNormPub = mNodeHandlePriv.advertise<std_msgs::Float64>("tracker_pose_cov_norm", 1);
  mPoseCovExpNormPub = mNodeHandlePriv.advertise<mcptam::TrackerCovInfo>("tracker_pose_cov_exp_norm", 1);
  
  mNodeHandlePriv.param<int>("small_image_level", mnSmallImageLevel, 2);
  if(mnSmallImageLevel < 0)
    mnSmallImageLevel = 0;
  if(mnSmallImageLevel >= LEVELS)
  {
    ROS_WARN_STREAM("small_image_level set to "<<mnSmallImageLevel<<" is too large, setting to maximum allowed value of "<<LEVELS-1);
    mnSmallImageLevel = LEVELS-1;
  }
  
  // Compute small image offsets from draw offsets based on pyramid level
  for(ImageRefMap::iterator offset_it = mmDrawOffsets.begin(); offset_it != mmDrawOffsets.end(); ++offset_it)
  {
    std::string camName = offset_it->first;
    mmSmallImageOffsets[camName] = CVD::ir(LevelNPos(CVD::vec(offset_it->second), mnSmallImageLevel));
  }
  
  // Full size of compound small image
  mirSmallImageSize = CVD::ir_rounded(LevelNPos(CVD::vec(mpVideoSourceMulti->GetTotalSize(false)), mnSmallImageLevel));
  
  
}

SystemFrontendBase::~SystemFrontendBase()
{
  // don't delete mpTracker because we didn't allocate it, let derived class do that
}

//  Calculates the average of the durations contained in the argument
double SystemFrontendBase::AverageDuration(std::deque<ros::Duration>& queue)
{
  ros::Duration durSum(0);
  for(unsigned i=0; i < queue.size(); ++i)
  {
    durSum += queue[i];
  }
  
  double dAvg = durSum.toSec() / queue.size();
  return dAvg;
}

// Check the flag for whether to draw the window
bool SystemFrontendBase::IsHeadless()
{
  ros::NodeHandle nh_priv("~");
  bool bHeadless;
  nh_priv.param<bool>("headless", bHeadless , false);
  
  return bHeadless;
}

// Publish the current tracker state
void SystemFrontendBase::PublishState()
{
  ROS_ASSERT(mpTracker);
  
  mcptam::TrackerState state_msg;
  
  state_msg.header.stamp = mpTracker->GetCurrentTimestamp();
  
  std::stringstream ss;
  ss<<mpTracker->GetCurrentPose();
  state_msg.se3TrackerPose = ss.str();
  state_msg.bLost = mpTracker->IsLost();
  state_msg.mTrackingQuality = mpTracker->GetTrackingQuality();
  state_msg.dCovNorm = mpTracker->GetCurrentCovarianceNorm();
  
  mStatePub.publish(state_msg);
}

// Publish the current tracker pose
void SystemFrontendBase::PublishPose()
{
  ROS_ASSERT(mpTracker);
  
  //if(mpTracker->GetTrackingQuality() == Tracker::NONE || mpTracker->IsLost())
  //  return;
  
  geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
  pose_cov_msg.header.stamp = mpTracker->GetCurrentTimestamp();
  pose_cov_msg.header.frame_id = "vision_world";
  
  SE3<> pose = mpTracker->GetCurrentPose().inverse();
  Matrix<3> rot = pose.get_rotation().get_matrix();
  TooN::Matrix<6> cov = mpTracker->GetCurrentCovariance(0);
  
  // debug:
  std_msgs::Float64 cov_norm_msg;
  cov_norm_msg.data = TooN::norm_fro(cov);
  mPoseCovNormPub.publish(cov_norm_msg);
  
  //---------- Experimental cov --------------
  TooN::Matrix<6> cov_exp = mpTracker->GetCurrentCovariance(1);
  TooN::Matrix<6> cov_old = mpTracker->GetCurrentCovariance(-1);
  ros::Duration cov_dur = mpTracker->GetCovarianceDuration();
  int nMapSize = mpMap->mlpPoints.size();
  int nMeasNum = mpTracker->GetMeasNum();
  
  mcptam::TrackerCovInfo cov_info_msg;
  cov_info_msg.cov_norm = TooN::norm_fro(cov_exp);
  cov_info_msg.cov_norm_approx = cov_norm_msg.data;
  cov_info_msg.cov_norm_old = TooN::norm_fro(cov_old);
  cov_info_msg.calc_dur = cov_dur.toSec();
  cov_info_msg.map_size = nMapSize;
  cov_info_msg.meas_num = nMeasNum;
  
  mPoseCovExpNormPub.publish(cov_info_msg);
  //------------------------------------------
  
  // clear cross correlation
  cov.slice<0,3,3,3>() = TooN::Zeros;
  cov.slice<3,0,3,3>() = TooN::Zeros;
  
  // Change covariance matrix frame from camera to world
  cov.slice<0,0,3,3>() = rot * cov.slice<0,0,3,3>() * rot.T();
  cov.slice<3,3,3,3>() = rot * cov.slice<3,3,3,3>() * rot.T();
  
  // Some hacky heuristics here
  if(mpTracker->GetTrackingQuality() == Tracker::GOOD)
  {
    cov = cov *1e2;
  }
  else if(mpTracker->GetTrackingQuality() == Tracker::DODGY)
  {
    cov = cov *1e5;
  }
  else if(mpTracker->GetTrackingQuality() == Tracker::BAD)
  {
    cov = cov *1e8;
  }
  
  pose_cov_msg.pose.pose = util::SE3ToPoseMsg(pose);
  
  int numElems = 6;
  for(int i=0; i < numElems; ++i)
  {
    for(int j=0; j < numElems; ++j)
    {
      pose_cov_msg.pose.covariance[i*numElems + j] = cov(i,j);
    }
  }
  
  mPoseWithCovPub.publish(pose_cov_msg);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::poseMsgToTF(pose_cov_msg.pose.pose, transform);
  br.sendTransform(tf::StampedTransform(transform, mpTracker->GetCurrentTimestamp(), "vision_world", "vision_pose"));
  
  SE3Map mPoses = mpTracker->GetCurrentCameraPoses();
  
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.stamp = pose_cov_msg.header.stamp;
  pose_array_msg.header.frame_id = pose_cov_msg.header.frame_id;
  pose_array_msg.poses.resize(1 + mPoses.size());
  pose_array_msg.poses[0] = pose_cov_msg.pose.pose;
  
  int i=1;
  for(SE3Map::iterator it = mPoses.begin(); it != mPoses.end(); ++it, ++i)
  {
    pose_array_msg.poses[i] = util::SE3ToPoseMsg(it->second.inverse());
    tf::poseMsgToTF(pose_array_msg.poses[i], transform);
    br.sendTransform(tf::StampedTransform(transform, mpTracker->GetCurrentTimestamp(), "vision_world", it->first));
  }
  
  mPoseArrayPub.publish(pose_array_msg);
}

void SystemFrontendBase::PublishSystemInfo(std::stringstream& captionStream)
{
  ROS_ASSERT(mpTracker);
  
  mcptam::SystemInfo info_msg;
  info_msg.header.stamp = mpTracker->GetCurrentTimestamp();
  
  if(mqFrameGrabDurations.size() == MAX_STATS_QUEUE_SIZE)
  {
    double dAvg = AverageDuration(mqFrameGrabDurations);
    captionStream << std::endl <<"Average Frame Grab Duration: "<< dAvg;
    info_msg.dFrameGrabDuration = dAvg;
  } 
  
  if(mqFrameDelayDurations.size() == MAX_STATS_QUEUE_SIZE)
  {
    double dAvg = AverageDuration(mqFrameDelayDurations);
    captionStream << std::endl <<"Average Frame Delay Duration: "<< dAvg;
    info_msg.dFrameDelayDuration = dAvg;
  } 
  
  if(mqTotalDurations.size() == MAX_STATS_QUEUE_SIZE)
  {
    double dAvg = AverageDuration(mqTotalDurations);
    captionStream << std::endl << "Average Tracking Duration: "<< dAvg;
    info_msg.dTrackingDuration = dAvg;
  }  
    
  if(mqLoopTimes.size() == MAX_STATS_QUEUE_SIZE)
  {
    ros::Duration dur(mqLoopTimes.back() - mqLoopTimes.front());
    double dFPS = (mqLoopTimes.size())/dur.toSec();
    captionStream<<std::endl<<"FPS: "<<dFPS;
    info_msg.dFPS = dFPS;
  }
  
  info_msg.dGrabSuccessRatio = (double)mnGrabSuccesses/mnGrabAttempts;
  info_msg.message = captionStream.str();
    
  mSystemInfoPub.publish(info_msg);
}


// Publish the small preview image
void SystemFrontendBase::PublishSmallImage()
{
  ROS_ASSERT(mpTracker);
  
  // Don't even bother doing image conversion if nobody's listening
  if(mSmallImagePub.getNumSubscribers() == 0)
    return;
    
  // We're going to create a small tiled image, based on mmSmallImageOffsets, and copy
  // individual downsampled images into specific locations in the image
  CVD::Image<CVD::byte> imSmall(mirSmallImageSize, 0);
  
  // Pointcloud to hold measurements. Z axis corresponds to pyramid level
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointMsg (new pcl::PointCloud<pcl::PointXYZ>());
    
  for(ImageRefMap::iterator offset_it = mmSmallImageOffsets.begin(); offset_it != mmSmallImageOffsets.end(); ++offset_it)
  {
    std::string camName = offset_it->first;
    CVD::ImageRef irOffset = offset_it->second;
    
    CVD::Image<CVD::byte> imTracker = mpTracker->GetKeyFrameImage(camName, mnSmallImageLevel);
    
    // When tracker just added an MKF to the map, it will have gotten rid of the MKF it is holding, so
    // imTracker will be an empty image. CVD::copy doesn't like this too much.
    if(imTracker.totalsize() > 0)  
      CVD::copy(imTracker, imSmall, imTracker.size(), CVD::ImageRef(), irOffset);
      
    for(unsigned l=0; l < LEVELS; ++l)
    {
      std::vector<TooN::Vector<2> > vMeas = mpTracker->GetKeyFrameSimpleMeas(camName, l);
      
      for(unsigned i=0; i < vMeas.size(); ++i)
      {
        TooN::Vector<2> v2MeasInSmallImage = CVD::vec(irOffset) + LevelNPos(vMeas[i], mnSmallImageLevel);
        pcl::PointXYZ pclPoint;
        
        pclPoint.x = v2MeasInSmallImage[0];
        pclPoint.y = v2MeasInSmallImage[1];
        pclPoint.z = l;
        
        pointMsg->points.push_back(pclPoint);
      }
    }
  }
  
  ROS_DEBUG_STREAM("Collected "<<pointMsg->points.size()<<" simple measurements and put them in a point cloud");
  
  ros::Time timestamp = mpTracker->GetCurrentTimestamp();
  
  pointMsg->header.frame_id = "tracker_small_image";
  pointMsg->width = pointMsg->points.size();
  pointMsg->height = 1;
  pointMsg->is_dense = false;
  
#if ROS_VERSION_MINIMUM(1, 9, 54)   // Hydro or above, uses new PCL library
  pointMsg->header.stamp = timestamp.toNSec();
#else
  pointMsg->header.stamp = timestamp;
#endif
  
  sensor_msgs::Image imageMsg;
  imageMsg.header.stamp = timestamp;
  util::ImageToMsg(imSmall, imageMsg);
  
  mSmallImagePointsPub.publish(pointMsg);
  mSmallImagePub.publish(imageMsg);    
}

// Callback called when an initialization request is received
bool SystemFrontendBase::InitSystemCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  mpTracker->RequestInit(true);
  
  return true;
}

// Callback called when a reset request is received
bool SystemFrontendBase::ResetSystemCallback(mcptam::Reset::Request &request, mcptam::Reset::Response &response)
{
  mpTracker->Reset(request.bSavePose, true);
  
  if(request.bReInit)
    mpTracker->RequestInit(false);
  
  return true;
}

