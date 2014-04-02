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

#include <mcptam/SystemClient.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/OpenGL.h>
#include <mcptam/MapMakerClient.h>
#include <mcptam/Tracker.h>
#include <mcptam/SystemInfo.h>
#include <mcptam/TrackerState.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <mcptam/LevelHelpers.h>
#include <sensor_msgs/Image.h>
#include <gvars3/instances.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cvd/utility.h>

using namespace GVars3;
using namespace TooN;

SystemClient::SystemClient()
: SystemBase("mcptam_client", false, !IsHeadless())
, mImageTransport(mNodeHandlePriv)
{
  int nGroups = mpVideoSourceMulti->GetNumGroups();
  int nCams = mpVideoSourceMulti->GetNumCams();
  
  if(nCams == 0)
  {
    ROS_FATAL("SystemClient: No cameras detected, shutting down.");
    ros::shutdown();
  }
  
  if(nGroups == nCams)  // no group has more than one camera
  {
    ROS_FATAL("SystemClient: Number of cameras is the same as number of groups, this won't work!");
    ros::shutdown();
    return;
  }
  
  if(mpGLWindow)
  {
    // Register some commands with GVars, these commands will be triggered
    // by button presses and will result in GUICommandCallBack being called
    GUI.RegisterCommand("exit", GUICommandCallBack, this);
    GUI.RegisterCommand("quit", GUICommandCallBack, this);
    GUI.RegisterCommand("ShowNextKeyFrame", GUICommandCallBack, this);
    GUI.RegisterCommand("ShowPrevKeyFrame", GUICommandCallBack, this);
    GUI.RegisterCommand("Reset", GUICommandCallBack, this);
    GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
    GUI.RegisterCommand("InitTracker", GUICommandCallBack, this);
    
    // Add menu groupings and buttons to the GL window
    /* Menu
    Root
      |
      +-> Reset
      +-> Init
      +-> Images
      | |
      | +-> Back
      | +-> Level0 Points
      | +-> Glare Masks
      | +-> Draw Masks
      +-> Level
      +-> Level Points
    */
    
    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    
    GUI.ParseLine("DrawMasks=0");
    GUI.ParseLine("DrawOnlyLevel=0");
    GUI.ParseLine("DrawLevel=0");
    GUI.ParseLine("GlareMasking=0");
    GUI.ParseLine("LevelZeroPoints=0");
    
    // Main Menu
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Init InitTracker Root");
    GUI.ParseLine("Menu.AddMenuButton Root \"Images...\" \"\" Images");
    GUI.ParseLine("Menu.AddMenuSlider Root \"Level\" DrawLevel 0 3 Root");
    GUI.ParseLine("Menu.AddMenuToggle Root \"Level Pts\" DrawOnlyLevel Root");

    // Images Menu
    GUI.ParseLine("Menu.AddMenuButton Images \"< Back\" \"\" Root");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Level0 Pts\" LevelZeroPoints Images");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Draw Masks\" DrawMasks Images");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Glare Mask\" GlareMasking Images");
  }
  
  mInitSystemServer = mNodeHandlePriv.advertiseService("init", &SystemClient::InitSystemCallback, this);
  mResetSystemServer = mNodeHandlePriv.advertiseService("reset", &SystemClient::ResetSystemCallback, this);
  
  mSystemInfoPub = mNodeHandlePriv.advertise<mcptam::SystemInfo>("system_info", 1);
  mTrackerStatePub = mNodeHandlePriv.advertise<mcptam::TrackerState>("tracker_state", 1);
  mTrackerPoseWithCovPub = mNodeHandlePriv.advertise<geometry_msgs::PoseWithCovarianceStamped>("tracker_pose_cov", 1);
  mTrackerPosePub = mNodeHandlePriv.advertise<geometry_msgs::PoseArray>("tracker_pose_array", 1);
  mTrackerSmallImagePub = mImageTransport.advertise("tracker_small_image", 1);
  
  mpMapMakerClient = new MapMakerClient(*mpMap);
  mpTracker = new Tracker(*mpMap, *mpMapMakerClient, mmCameraModels, mmPoses, mmDrawOffsets, mpGLWindow);
  
  ImageBWMap masksMap = LoadMasks(); 
  mpTracker->SetMasks(masksMap);
  
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
  mirSmallImageSize = CVD::ir(LevelNPos(CVD::vec(mpVideoSourceMulti->GetTotalSize(false)), mnSmallImageLevel));
  
  double rate;
  mNodeHandlePriv.param<double>("pose_publish_rate", rate, 0);
  
  if(rate == 0)
    mPosePublishDur = ros::Duration(0);
  else
    mPosePublishDur = ros::Duration(1.0/rate);
    
  mLastPosePublishTime = ros::Time::now();

  mbDone = false;
}

SystemClient::~SystemClient()
{
  delete mpTracker;
  delete mpMapMakerClient;
}

void SystemClient::Run()
{
  // This data will be displayed on GUI and published by mDiagnosticPub
  std::queue<ros::Time> qLoopTimes;
  std::deque<ros::Duration> qTotalDurations;
  std::deque<ros::Duration> qFrameGrabDurations;
  
  unsigned int nMaxQueueSize = 10;
  ros::Time grabStartTime;
  ros::Time trackStartTime;
  bool bLastGrabSuccess = true;
  long int nGrabAttempts = 0;
  long int nGrabSuccesses = 0;
  
  while(!mbDone && ros::ok())
  {
    if(mpGLWindow)
    {
       // Required before drawing
      mpGLWindow->SetupViewport();  
      mpGLWindow->SetupVideoOrtho();
      mpGLWindow->SetupVideoRasterPosAndZoom();
    }
    
    // Grab new video frame...
    ++nGrabAttempts;
    if(bLastGrabSuccess)
      grabStartTime = ros::Time::now();
    ros::Time timestamp;
    bool bLastGrabSuccess = mpVideoSourceMulti->GetAndFillFrameBW(ros::WallDuration(0.2), mmFramesBW,timestamp);
    
    if(bLastGrabSuccess)
    {
      ++nGrabSuccesses;
      
      if(mpGLWindow)
      {
        // Clear screen with black
        glClearColor(0,0,0,1);
        glClear(GL_COLOR_BUFFER_BIT);
      }
      
      qFrameGrabDurations.push_back(ros::Time::now() - grabStartTime);
      if(qFrameGrabDurations.size() > nMaxQueueSize)
        qFrameGrabDurations.pop_front();
      
      trackStartTime = ros::Time::now();
      // Perform the actual tracking, only draw if mpGLWindow is set!
      mpTracker->TrackFrame(mmFramesBW, timestamp, mpGLWindow);
    
      qTotalDurations.push_back(ros::Time::now() - trackStartTime);
      if(qTotalDurations.size() > nMaxQueueSize)
        qTotalDurations.pop_front();
      
      qLoopTimes.push(ros::Time::now());
      if(qLoopTimes.size() > nMaxQueueSize)
        qLoopTimes.pop();
        
      PublishState();
      
      if(mPosePublishDur == ros::Duration(0) || ros::Time::now() - mLastPosePublishTime > mPosePublishDur)
      {
        PublishPose();
        mLastPosePublishTime = ros::Time::now();
      }
    }
    else
    {
      // "System: didn't get new frame"
    }
   
    std::stringstream captionStream;
    captionStream << mpTracker->GetMessageForUser();
      
    mcptam::SystemInfo info_msg;
    info_msg.header.stamp = ros::Time::now();
    
    if(qFrameGrabDurations.size() == nMaxQueueSize)
    {
      double dAvg = AverageDuration(qFrameGrabDurations);
      captionStream << std::endl <<"Average Frame Grab Duration: "<< dAvg;
      info_msg.dFrameGrabDuration = dAvg;
    }  
    
    if(qTotalDurations.size() == nMaxQueueSize)
    {
      double dAvg = AverageDuration(qTotalDurations);
      captionStream << std::endl << "Average Tracking Duration: "<< dAvg;
      info_msg.dTrackingDuration = dAvg;
    }  
      
    if(qLoopTimes.size() == nMaxQueueSize)
    {
      ros::Duration dur(qLoopTimes.back() - qLoopTimes.front());
      double dFPS = (nMaxQueueSize-1)/dur.toSec();
      captionStream<<std::endl<<"FPS: "<<dFPS;
      info_msg.dFPS = dFPS;
    }
    
    info_msg.dGrabSuccessRatio = (double)nGrabSuccesses/nGrabAttempts;
    info_msg.message = captionStream.str();
      
    mSystemInfoPub.publish(info_msg);
      
    if(mpGLWindow)
    {
      mpGLWindow->DrawCaption(captionStream.str());
      mpGLWindow->DrawMenus();
      mpGLWindow->swap_buffers();
      mpGLWindow->HandlePendingEvents();
    }
    
    // GUI interface
    while(!mqCommands.empty())
    {
      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }
    
    PublishSmallImage();
    mCallbackQueueROS.callAvailable();
  }
}

// Callback called when an initialization request is received
bool SystemClient::InitSystemCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  if(!mpMap->mbGood)
    mpTracker->RequestInit(true);
  
  return true;
}

// Callback called when a reset request is received
bool SystemClient::ResetSystemCallback(mcptam::Reset::Request &request, mcptam::Reset::Response &response)
{
  mpTracker->Reset(request.bSavePose, true);
  
  if(request.bReInit)
    mpTracker->RequestInit(false);
  
  return true;
}

// Deals with user interface commands
void SystemClient::GUICommandHandler(std::string command, std::string params)  
{
  if(command=="quit" || command == "exit")
  {
    mbDone = true;
    return;
  }
  
  if(command=="Reset")
  {
    mpTracker->Reset(false, true);
    return;
  }

  // KeyPress commands are issued by GLWindow
  if(command=="KeyPress")
  {
    if(params == "Space")
    {
      mpTracker->RequestInit(true);
    }
    else if(params == "r")
    {
      mpTracker->Reset(false, true);
    }
    else if(params == "q" || params == "Escape")
    {
      mbDone = true;
    }
    return;
  }
  
  if(command=="InitTracker")
  {
    mpTracker->RequestInit(true);
    return;
  }
  
  ROS_FATAL_STREAM("SystemClient: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}

// Check the flag for whether to draw the window
bool SystemClient::IsHeadless()
{
  ros::NodeHandle nh_priv("~");
  bool bHeadless;
  nh_priv.param<bool>("headless", bHeadless , false);
  
  return bHeadless;
}

// Publish the current tracker state
void SystemClient::PublishState()
{
  mcptam::TrackerState state_msg;
  
  state_msg.header.stamp = ros::Time::now();
  
  std::stringstream ss;
  ss<<mpTracker->GetCurrentPose();
  state_msg.se3TrackerPose = ss.str();
  state_msg.bLost = mpTracker->IsLost();
  state_msg.mTrackingQuality = mpTracker->GetTrackingQuality();
  state_msg.dCovNorm = mpTracker->GetCurrentCovarianceNorm();
  
  mTrackerStatePub.publish(state_msg);
}

// Publish the current tracker pose
void SystemClient::PublishPose()
{
  if(mpTracker->GetTrackingQuality() == Tracker::NONE || mpTracker->IsLost())
    return;
  
  geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
  pose_cov_msg.header.stamp = mpTracker->GetCurrentTimestamp();
  pose_cov_msg.header.frame_id = "vision_world";
  
  SE3<> pose = mpTracker->GetCurrentPose().inverse();
  Matrix<3> rot = pose.get_rotation().get_matrix();
  TooN::Matrix<6> cov = mpTracker->GetCurrentCovariance();
  
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
    cov = cov *1e4;
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
  
  mTrackerPoseWithCovPub.publish(pose_cov_msg);
  
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
  
  mTrackerPosePub.publish(pose_array_msg);
}

// Publish the small preview image
void SystemClient::PublishSmallImage()
{
  // Don't even bother doing image conversion if nobody's listening
  if(mTrackerSmallImagePub.getNumSubscribers() == 0)
    return;
    
  if(mmFramesBW.size() == 0)
    return;
    
  // We're going to create a small tiled image, based on mmSmallImageOffsets, and copy
  // individual downsampled images into specific locations in the image
    
  CVD::Image<CVD::byte> imSmall(mirSmallImageSize, 0);
    
  for(ImageRefMap::iterator offset_it = mmSmallImageOffsets.begin(); offset_it != mmSmallImageOffsets.end(); ++offset_it)
  {
    std::string camName = offset_it->first;
    CVD::ImageRef irOffset = offset_it->second;
    
    CVD::Image<CVD::byte> imTracker = mpTracker->GetKeyFrameImage(camName, mnSmallImageLevel);
    
    // When tracker just added an MKF to the map, it will have gotten rid of the MKF it is holding, so
    // imTracker will be an empty image. CVD::copy doesn't like this too much.
    if(imTracker.totalsize() > 0)  
      CVD::copy(imTracker, imSmall, imTracker.size(), CVD::ImageRef(), irOffset); 	
  }
    
  sensor_msgs::Image image_msg;
  image_msg.header.stamp = ros::Time::now();
  
  util::ImageToMsg(imSmall, image_msg);
  
  mTrackerSmallImagePub.publish(image_msg);    
}

