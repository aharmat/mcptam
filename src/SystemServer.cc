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

#include <mcptam/SystemServer.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/KeyFrameVisualizer.h>
#include <mcptam/OpenGL.h>
#include <mcptam/MapMakerServer.h>
#include <mcptam/BundleAdjusterMulti.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <mcptam/Reset.h>
#include <mcptam/LevelHelpers.h>
#include <gvars3/instances.h>
#include <std_srvs/Empty.h>

using namespace GVars3;
using namespace TooN;

SystemServer::SystemServer()
: SystemBase("mcptam_server", true, true)
, mImageTransport(mNodeHandle)
{ 
  // Register some commands with GVars, these commands will be triggered
  // by button presses and will result in GUICommandCallBack being called
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("ShowNextKeyFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("ShowPrevKeyFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("InitTracker", GUICommandCallBack, this);
  
  // Add menu groupings and buttons to the GL window
  /* Menu
  Root
    |
    +-> Reset
    +-> Init
    +-> Keyframes
    | |
    | +-> Back
    | +-> Candidates (on/off)
    | +-> Show Next
    | +-> Show Prev
    | +-> Level
  */
  
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  
  GUI.ParseLine("DrawTrackerMeas=1");
  GUI.ParseLine("DrawCandidates=0");
  GUI.ParseLine("DrawLevel=0");
  GUI.ParseLine("LevelZeroPoints=0");
  
  static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN|SILENT);
  *gvnLevelZeroPoints = SystemBase::sbLevelZeroPoints;
  
  // Main Menu
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Init InitTracker Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"LiveMeas\" DrawTrackerMeas Root");
  GUI.ParseLine("Menu.AddMenuButton Root \"Keyframes...\" \"\" View");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Level0 Pts\" LevelZeroPoints Root");

  // View Keyframes
  GUI.ParseLine("Menu.AddMenuButton View \"< Back\" \"\" Root");
  GUI.ParseLine("Menu.AddMenuToggle View \"Candidates\" DrawCandidates View");
  GUI.ParseLine("Menu.AddMenuButton View \"Show Next\" ShowNextKeyFrame View");
  GUI.ParseLine("Menu.AddMenuButton View \"Show Prev\" ShowPrevKeyFrame View");
  GUI.ParseLine("Menu.AddMenuSlider View \"Level\" DrawLevel 0 3 View");
  
  // All of these will need to be remapped so advertise it in global namespace for ease of remapping
  mInitSystemClient = mNodeHandle.serviceClient<std_srvs::Empty>("init");
  while(!mInitSystemClient.waitForExistence(ros::Duration(5)) && ros::ok())
		ROS_WARN_STREAM("SystemServer: Waiting for init service to be advertised by SystemClient...");
    
  mResetSystemClient = mNodeHandle.serviceClient<mcptam::Reset>("reset");
  while(!mResetSystemClient.waitForExistence(ros::Duration(5)) && ros::ok())
		ROS_WARN_STREAM("SystemServer: Waiting for reset service to be advertised by SystemClient...");
  
  mSystemInfoSub = mNodeHandle.subscribe("system_info", 1, &SystemServer::SystemInfoCallback, this);
  mTrackerStateSub = mNodeHandle.subscribe("tracker_state", 1, &SystemServer::TrackerStateCallback, this);
  //mTrackerSmallImageSub = mImageTransport.subscribe("tracker_small_image", 1, &SystemServer::TrackerSmallImageCallback, this);
  
  mSmallImageSub = mImageTransport.subscribe("tracker_small_image", 1, &SystemServer::TrackerSmallImageCallback, this); 
  mSmallImagePointsSub = mNodeHandle.subscribe("tracker_small_image_points", 1, &SystemServer::TrackerSmallImagePointsCallback, this);
  
  mpBundleAdjuster = new BundleAdjusterMulti(*mpMap, mmCameraModels);
  mpMapMakerServer = new MapMakerServer(*mpMap, mmCameraModels, *mpBundleAdjuster);
  mpKeyFrameVisualizer = new KeyFrameVisualizer(*mpMap, *mpGLWindow, mmDrawOffsets, mpVideoSourceMulti->GetSizes());
  
  // VideoSourceMulti was only needed to build mmCameraModels, delete it so that
  // subscribers shut down
  delete mpVideoSourceMulti;
  mpVideoSourceMulti = NULL;

  mbDone = false;
  
  // Data that we'll receive from client side
  mTrackingQuality = Tracker::NONE;
  mbTrackerLost = false;
  mdFrameGrabDuration = 0;
  mdTrackingDuration = 0;
  mdFPS = 0;
  mdGrabSuccessRatio = 0;
}

SystemServer::~SystemServer()
{
  delete mpKeyFrameVisualizer;
  delete mpMapMakerServer;
  delete mpBundleAdjuster;
}

void SystemServer::Run()
{
  while(!mbDone && ros::ok())
  {
    mCallbackQueueROS.callAvailable();
    
    mpGLWindow->SetupViewport();
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();
    
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT);
    
    static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN|SILENT);
    bool bDrawKeyFrames = *gvsCurrentSubMenu == "View";
    
    static gvar3<int> gvnDrawTrackerMeas("DrawTrackerMeas", 1, HIDDEN|SILENT);
    
    std::stringstream captionStream;
    
    if(!bDrawKeyFrames)
    {
      CVD::ImageRef irWindowSize = mpGLWindow->GetWindowSize();
      CVD::ImageRef irOffset = (irWindowSize - mTrackerSmallImage.size()) * 0.5;
      CVD::ImageRef irZero(0,0);
      if(irOffset < irZero)
        irOffset = irZero;
        
      glRasterPos(irOffset);  // Top left corner
      glDrawPixels(mTrackerSmallImage);
      
      if(*gvnDrawTrackerMeas)
      {
        glPointSize(4);
        glEnable(GL_BLEND);
        glEnable(GL_POINT_SMOOTH);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_POINTS);
        
        for(unsigned i=0; i < mTrackerSmallImagePoints.points.size(); ++i)
        {
          pcl::PointXYZ& point = mTrackerSmallImagePoints.points[i];
          CVD::glColor(gavLevelColors[(int)point.z]);
          CVD::glVertex(CVD::vec(irOffset) + TooN::makeVector(point.x, point.y));
        }
        
        glEnd();
      }
      
      double dMaxPointCov = mpMapMakerServer->GetMaxCov();
      
      if(dMaxPointCov > 0)
      {
        double dRedFrac = dMaxPointCov/10;
        if(dRedFrac > 1)
          dRedFrac = 1.0;
        double dGreenFac = 1-dRedFrac;
        std::stringstream sscov;
        sscov<<"Max cov: "<<dMaxPointCov;
        std::string covstring = sscov.str();
      
        glColor3f(dRedFrac,dGreenFac,0);
        mpGLWindow->PrintString(CVD::ImageRef(10,80), covstring, 15);
      }
      
    }
    else
    {
      mpKeyFrameVisualizer->Draw();
      captionStream << mpKeyFrameVisualizer->GetMessageForUser();
      captionStream << std::endl;
    }
    
    if(mTrackingQuality != Tracker::NONE)
    {
      if(!mbTrackerLost)
      {
        captionStream << "Tracking Map, quality ";
        if(mTrackingQuality == Tracker::GOOD)  captionStream << "good.";
        if(mTrackingQuality == Tracker::DODGY) captionStream << "poor.";
        if(mTrackingQuality == Tracker::BAD)   captionStream << "bad.";
        
        captionStream << std::endl;
      }
      else
      {
        captionStream << "** Attempting recovery **" << std::endl;
      }
    }
    else
    {
      captionStream << "Waiting to initialize tracker"<<std::endl;
    }
    
    captionStream<<"Frame Grab Success Ratio: "<<mdGrabSuccessRatio<<std::endl;
    captionStream<<"Frame Grab Dur: "<<mdFrameGrabDuration<<std::endl;
    captionStream<<"Frame Delay Dur: "<<mdFrameDelayDuration<<std::endl;
    captionStream<<"Tracking Dur: "<<mdTrackingDuration<<std::endl;
    captionStream<<"FPS: "<<mdFPS<<std::endl;
    
    // Add current time to have some indication of being alive
    captionStream << "Time: "<<ros::Time::now();

    mpGLWindow->DrawCaption(captionStream.str());
    mpGLWindow->DrawMenus();
    mpGLWindow->swap_buffers();
    mpGLWindow->HandlePendingEvents();
    
    // GUI interface
    while(!mqCommands.empty())
    {
      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }
  }
}

// Deals with user interface commands
void SystemServer::GUICommandHandler(std::string command, std::string params)
{
  if(command=="quit" || command == "exit")
  {
    mbDone = true;
    return;
  }
  
  if(command=="Reset")
  {
    mcptam::Reset reset_srv;
    reset_srv.request.bReInit = false;
    reset_srv.request.bSavePose = false;
    while(!mResetSystemClient.call(reset_srv) && ros::ok())
    {
      ROS_INFO("SystemServer: Trying to reset system...");
      ros::Duration(0.5).sleep();
    }

    return;
  }
  
  if(command=="ShowNextKeyFrame")
  {
    mpKeyFrameVisualizer->Next();
    return;
  }
  
  if(command=="ShowPrevKeyFrame")
  {
    mpKeyFrameVisualizer->Prev();
    return;
  }
  
  // KeyPress commands are issued by GLWindow
  if(command=="KeyPress")
  {
    if(params == "Space")  // send start signal to tracker
    {
      std_srvs::Empty srv;
      while(!mInitSystemClient.call(srv) && ros::ok())
      {
        ROS_INFO("SystemServer: Trying to initialize system...");
        ros::Duration(0.5).sleep();
      }
    }
    else if(params == "r")  // reset
    {
      mcptam::Reset reset_srv;
      reset_srv.request.bReInit = false;
      reset_srv.request.bSavePose = false;
      while(!mResetSystemClient.call(reset_srv) && ros::ok())
      {
        ROS_INFO("SystemServer: Trying to reset system...");
        ros::Duration(0.5).sleep();
      }
    }
    else if(params == "q" || params == "Escape")
    {
      mbDone = true;
    }
    return;
  }
  
  if(command=="InitTracker")  // send start signal to tracker
  {
    std_srvs::Empty srv;
    while(!mInitSystemClient.call(srv) && ros::ok())
    {
      ROS_INFO("SystemServer: Trying to initialize system...");
      ros::Duration(0.5).sleep();
    }
    
    return;
  }
  
  ROS_FATAL_STREAM("SystemServer: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}

// Callback called when a new system info message received
void SystemServer::SystemInfoCallback(const mcptam::SystemInfoConstPtr& infoMsg)
{
  mdFrameGrabDuration = infoMsg->dFrameGrabDuration;
  mdFrameDelayDuration = infoMsg->dFrameDelayDuration;
  mdTrackingDuration = infoMsg->dTrackingDuration;
  mdFPS = infoMsg->dFPS;
  mdGrabSuccessRatio = infoMsg->dGrabSuccessRatio;
  //std::cout<<infoMsg->message<<std::endl;
}

// Callback called when a new tracker state message received
void SystemServer::TrackerStateCallback(const mcptam::TrackerStateConstPtr& stateMsg)
{
  std::stringstream ss;
  ss << stateMsg->se3TrackerPose;
  ss >> mse3TrackerPose;
  mTrackingQuality = Tracker::TrackingQuality(stateMsg->mTrackingQuality);
  mbTrackerLost = stateMsg->bLost;
}

// Callback called when a new small preview image is received
void SystemServer::TrackerSmallImageCallback(const sensor_msgs::ImageConstPtr& imageMsg)
{
  ROS_DEBUG("In TrackerSmallImageCallback!");
  
  cv_bridge::CvImageConstPtr cvPtr;
  
  try
  {
    // Try a conversion
    cvPtr = cv_bridge::toCvShare(imageMsg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("TrackerSmallImageCallback: cv_bridge exception: %s", e.what());
    return;
  }
  
  util::ConversionRGB(cvPtr->image, mTrackerSmallImage);  
}

// Callback called when a new small preview image point set is received
void SystemServer::TrackerSmallImagePointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointMsg)
{
  mTrackerSmallImagePoints = *pointMsg;
}
