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
 * \file System.cc
 * \brief Implementation of the System class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the original PTAM, which are
 * Copyright 2008 Isis Innovation Limited
 *
 * Description of the System.cc file.
 *
 ****************************************************************************************/

#include <mcptam/System.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/KeyFrameViewer.h>
#include <mcptam/OpenGL.h>
#include <mcptam/MapMaker.h>
#include <mcptam/Map.h>
#include <mcptam/Tracker.h>
#include <mcptam/SystemInfo.h>
#include <mcptam/BundleAdjusterMulti.h>

#include <mcptam/TrackerState.h>
#include <gvars3/instances.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <cvd/image_interpolate.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <mcptam/Utility.h>

using namespace GVars3;
using namespace TooN;

System::System()
: SystemBase("mcptam",true, true)
{
  int nGroups = mpVideoSourceMulti->GetNumGroups();
  int nCams = mpVideoSourceMulti->GetNumCams();
  
  if(nCams == 0)
  {
    ROS_FATAL("System: No cameras detected, shutting down.");
    ros::shutdown();
  }
  
  if(nGroups == nCams && nCams > 1)  // no group has more than one camera and more than 1 group
  {
    ROS_FATAL("System: More than one camera group and none have more than one camera, won't be able to initialize");
    ros::shutdown();
  }
  
  // Register some commands with GVars, these commands will be triggered
  // by button presses and will result in GUICommandCallBack being called
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("ShowNextKeyFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("ShowPrevKeyFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("InitTracker", GUICommandCallBack, this);
  
  // Debug commands for menu
  GUI.RegisterCommand("ScaleMapUp", GUICommandCallBack, this);
  GUI.RegisterCommand("ScaleMapDown", GUICommandCallBack, this);
  GUI.RegisterCommand("ExportMapToFile", GUICommandCallBack, this);
  GUI.RegisterCommand("ManualAddMKF", GUICommandCallBack, this);
  
  // Add menu groupings and buttons to the GL window
  /* Menu
  Root
    |
    +-> Reset
    +-> Init
    +-> Debug
    | |
    | +-> Back
    | +-> CrossCam
    | +-> Export Map
    | +-> Scale Up
    | +-> Scale Down
    | +-> Add MKF
    +-> Images
    | |
    | +-> Back
    | +-> Level0 Points
    | +-> Glare Masks
    | +-> Draw Masks
    +-> Keyframes
    | |
    | +-> Back
    | +-> Add MKFs
    | +-> View Keyframes
    |   |
    |   +-> Back
    |   +-> Candidates (on/off)
    |   +-> Show Next
    |   +-> Show Prev
    |   +-> Level
    +-> Level
    +-> Level Points
  */

  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  
  GUI.ParseLine("DrawMasks=0");
  GUI.ParseLine("DrawCandidates=0");
  GUI.ParseLine("DrawOnlyLevel=0");
  GUI.ParseLine("DrawLevel=0");
  GUI.ParseLine("GlareMasking=0");
  GUI.ParseLine("LevelZeroPoints=0");
  GUI.ParseLine("AddingMKFs=1");
  GUI.ParseLine("CrossCamera=1");
  
  // Main Menu
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Init InitTracker Root");
  GUI.ParseLine("Menu.AddMenuButton Root \"Debug...\" \"\" Debug");
  GUI.ParseLine("Menu.AddMenuButton Root \"Images...\" \"\" Images");
  GUI.ParseLine("Menu.AddMenuButton Root \"Keyframes...\" \"\" Keyframes");
  GUI.ParseLine("Menu.AddMenuSlider Root \"Level\" DrawLevel 0 3 Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Level Pts\" DrawOnlyLevel Root");

  // Debug Menu
  GUI.ParseLine("Menu.AddMenuButton Debug \"< Back\" \"\" Root");
  GUI.ParseLine("Menu.AddMenuToggle Debug \"CrossCam\" CrossCamera Debug");
  GUI.ParseLine("Menu.AddMenuButton Debug \"Export Map\" ExportMapToFile Debug");
  GUI.ParseLine("Menu.AddMenuButton Debug \"Scale Down\" ScaleMapDown Debug");
  GUI.ParseLine("Menu.AddMenuButton Debug \"Scale Up\" ScaleMapUp Debug");
  GUI.ParseLine("Menu.AddMenuButton Debug \"Add MKF\" ManualAddMKF Debug");

  // Images Menu
  GUI.ParseLine("Menu.AddMenuButton Images \"< Back\" \"\" Root");
  GUI.ParseLine("Menu.AddMenuToggle Images \"Level0 Pts\" LevelZeroPoints Images");
  GUI.ParseLine("Menu.AddMenuToggle Images \"Draw Masks\" DrawMasks Images");
  GUI.ParseLine("Menu.AddMenuToggle Images \"Glare Mask\" GlareMasking Images");

  // Keyframes Menu
  GUI.ParseLine("Menu.AddMenuButton Keyframes \"< Back\" \"\" Root");
  GUI.ParseLine("Menu.AddMenuToggle Keyframes \"Add MKFs\" AddingMKFs Keyframes");
  GUI.ParseLine("Menu.AddMenuButton Keyframes \"View...\" \"\" View");

  // View Keyframes
  GUI.ParseLine("Menu.AddMenuButton View \"< Back\" \"\" Keyframes");
  GUI.ParseLine("Menu.AddMenuToggle View \"Candidates\" DrawCandidates View");
  GUI.ParseLine("Menu.AddMenuButton View \"Show Next\" ShowNextKeyFrame View");
  GUI.ParseLine("Menu.AddMenuButton View \"Show Prev\" ShowPrevKeyFrame View");
  GUI.ParseLine("Menu.AddMenuSlider View \"Level\" DrawLevel 0 3 View");
  
  
  // Advertise the topics
  mSystemInfoPub = mNodeHandlePriv.advertise<mcptam::SystemInfo>("system_info", 1);
  mTrackerStatePub = mNodeHandlePriv.advertise<mcptam::TrackerState>("tracker_state", 1);
  mTrackerPoseWithCovPub = mNodeHandlePriv.advertise<geometry_msgs::PoseWithCovarianceStamped>("tracker_pose_cov", 1);
  mTrackerPosePub = mNodeHandlePriv.advertise<geometry_msgs::PoseArray>("tracker_pose_array", 1);
  // Advertise reset before creating mapmaker
  mResetSystemServer = mNodeHandlePriv.advertiseService("reset", &System::ResetSystemCallback, this);
  
  // Create the BundleAdjuster, MapMaker, and Tracker objects
  mpBundleAdjuster = new BundleAdjusterMulti(*mpMap, mmCameraModels, true, false);
  mpMapMaker = new MapMaker(*mpMap, mmCameraModels, *mpBundleAdjuster);
  mpTracker = new Tracker(*mpMap, *mpMapMaker, mmCameraModels, mmPoses, mmDrawOffsets, mpGLWindow);
  mpKeyFrameViewer = new KeyFrameViewer(*mpMap, *mpGLWindow, mmDrawOffsets, mpVideoSourceMulti->GetSizes());
  
  ImageBWMap masksMap = LoadMasks(); 
  mpTracker->SetMasks(masksMap);
  
  // Initialize the variables for calculating the tracking rate
  double rate;
  mNodeHandlePriv.param<double>("pose_publish_rate", rate, 0);
  
  if(rate == 0)
    mPosePublishDur = ros::Duration(0);
  else
    mPosePublishDur = ros::Duration(1.0/rate);
    
  mLastPosePublishTime = ros::Time::now();
  
  mbDone = false;
}

System::~System()
{
  delete mpKeyFrameViewer;
  delete mpTracker;
  delete mpMapMaker;
  delete mpBundleAdjuster;
}

void System::Run()
{
  // This data will be displayed on GUI
  std::queue<ros::Time> qLoopTimes;
  std::deque<ros::Duration> qTotalDurations;
  std::deque<ros::Duration> qFrameGrabDurations;
  std::deque<ros::Duration> qFrameDelayDurations;
  
  unsigned int nMaxQueueSize = 10;
  ros::Time grabStartTime;
  ros::Time grabEndTime;
  ros::Time trackStartTime;
  bool bLastGrabSuccess = true;
  
  // Loop until instructed to stop
  while(!mbDone && ros::ok())
  {
    // Required before drawing
    mpGLWindow->SetupViewport();  
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();
  
    // Grab new video frame...
    if(bLastGrabSuccess)
      grabStartTime = ros::Time::now();
    ros::Time timestamp;
    bLastGrabSuccess = mpVideoSourceMulti->GetAndFillFrameBW(ros::WallDuration(0.2), mmFramesBW, timestamp);
    grabEndTime = ros::Time::now();
    
    static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN|SILENT);
    bool bDrawKeyFrames = *gvsCurrentSubMenu == "View";
    
    if(bLastGrabSuccess)
    {
      // Clear screen with black
      glClearColor(0,0,0,1);
      glClear(GL_COLOR_BUFFER_BIT);
  
      qFrameGrabDurations.push_back(grabEndTime - grabStartTime);
      if(qFrameGrabDurations.size() > nMaxQueueSize)
        qFrameGrabDurations.pop_front();
        
      qFrameDelayDurations.push_back(grabEndTime - timestamp);
      if(qFrameDelayDurations.size() > nMaxQueueSize)
        qFrameDelayDurations.pop_front();
      
      trackStartTime = ros::Time::now();
      // Perform the actual tracking
      mpTracker->TrackFrame(mmFramesBW, timestamp, !bDrawKeyFrames);
    
      // Calculate timings for the loop and tracking steps
      qTotalDurations.push_back(ros::Time::now() - trackStartTime);
      if(qTotalDurations.size() > nMaxQueueSize)
        qTotalDurations.pop_front();
      
      qLoopTimes.push(ros::Time::now());
      if(qLoopTimes.size() > nMaxQueueSize)
        qLoopTimes.pop();
        
      // Send out the tracker state and pose messages from the tracking result
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
      
    if(bDrawKeyFrames)
    {
      // Clear screen with black
      glClearColor(0,0,0,0);
      glClear(GL_COLOR_BUFFER_BIT);
        
      mpKeyFrameViewer->Draw();
    }
    else
    {
      // Show the main menu
    }
   
    // Update the GUI with the caption info
    std::stringstream captionStream;
    if(bDrawKeyFrames)
      captionStream << mpKeyFrameViewer->GetMessageForUser();
    else
      captionStream << mpTracker->GetMessageForUser();
      
    mcptam::SystemInfo info_msg;
    info_msg.header.stamp = ros::Time::now();
    
    if(qFrameGrabDurations.size() == nMaxQueueSize)
    {
      double dAvg = AverageDuration(qFrameGrabDurations);
      captionStream << std::endl <<"Average Frame Grab Duration: "<< dAvg;
      info_msg.dFrameGrabDuration = dAvg;
    }  
    
    if(qFrameDelayDurations.size() == nMaxQueueSize)
    {
      double dAvg = AverageDuration(qFrameDelayDurations);
      captionStream << std::endl <<"Average Frame Delay: "<< dAvg;
      //info_msg.dFrameGrabDuration = dAvg;
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
      
    mSystemInfoPub.publish(info_msg);
      
    mpGLWindow->DrawCaption(captionStream.str());
    mpGLWindow->DrawMenus();
    mpGLWindow->swap_buffers();
    mpGLWindow->HandlePendingEvents();
    
    // Handle any remaining commands from the GUI interface
    while(!mqCommands.empty())
    {
      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }
    
    mCallbackQueueROS.callAvailable();
  }
}

void System::GUICommandHandler(std::string command, std::string params)  
{
  if(command=="quit" || command == "exit")
  {
    mbDone = true;
    return;
  }
  
  if(command=="ShowNextKeyFrame")
  {
    mpKeyFrameViewer->Next();
    return;
  }
  
  if(command=="ShowPrevKeyFrame")
  {
    mpKeyFrameViewer->Prev();
    return;
  }
  
  if(command=="ScaleMapUp")
  {
    mpMapMaker->RequestRescaling(2.0);
    return;
  }

  if(command=="ScaleMapDown")
  {
    mpMapMaker->RequestRescaling(0.5);
    return;
  }

  if(command=="ExportMapToFile")
  {
    std::string mapFileName = "map.dat";
    std::string cameraFileName = "cameras.dat";

    ROS_INFO_STREAM("> Requesting file dump to "<<mapFileName<<" and "<<cameraFileName);

    mpMapMaker->RequestFileDump(mapFileName);
    DumpCamerasToFile(cameraFileName);
    return;
  }

  if(command=="ManualAddMKF")
  {
    if(mpMapMaker->Initializing())
    {
      ROS_INFO_STREAM("> Forcing end of initialization");
      mpMapMaker->RequestStopInit();
    }
    else
    {
      ROS_INFO_STREAM("> Forcing adding of next MKF");
      mpTracker->AddNext();
    }
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
    else if(params == "o")
    {
      mpBundleAdjuster->SetNotConverged();
    }
    else if(params == "a")
    {
      mpTracker->AddNext();
    }
    
    return;
  }
  
  if(command=="InitTracker")
  {
    mpTracker->RequestInit(true);
    return;
  }
  
  ROS_FATAL_STREAM("System: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}

// Callback for Tracker reset command
bool System::ResetSystemCallback(mcptam::Reset::Request &request, mcptam::Reset::Response &response)
{
  mpTracker->Reset(request.bSavePose, true);
  
  if(request.bReInit)
    mpTracker->RequestInit(false);
  
  return true;
}

// Publish the system state
void System::PublishState()
{
  mcptam::TrackerState state_msg;
  
  state_msg.header.stamp = mpTracker->GetCurrentTimestamp();
  
  std::stringstream ss;
  ss<<mpTracker->GetCurrentPose();
  state_msg.se3TrackerPose = ss.str();
  state_msg.bLost = mpTracker->IsLost();
  state_msg.mTrackingQuality = mpTracker->GetTrackingQuality();
  state_msg.dCovNorm = mpTracker->GetCurrentCovarianceNorm();
  
  if(state_msg.mTrackingQuality != Tracker::NONE)
    mTrackerStatePub.publish(state_msg);
}

// Publish the current pose
void System::PublishPose()
{
  // If the tracker is lost, exit early without publishing
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
  
  // Some hacky heuristics here, may want to add these values to a config file
  if(mpTracker->GetTrackingQuality() == Tracker::GOOD)
  {
    cov = cov * 1e2;
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
  
  mTrackerPoseWithCovPub.publish(pose_cov_msg);
  
  // Publish the transforms and poses for each of the cluster cameras at the current time
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
