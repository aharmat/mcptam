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
#include <mcptam/KeyFrameViewer.h>
#include <mcptam/OpenGL.h>
#include <mcptam/MapMaker.h>
#include <mcptam/Tracker.h>
#include <mcptam/BundleAdjusterMulti.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/Types.h>
#include <gvars3/instances.h>

using namespace GVars3;
using namespace TooN;

System::System()
: SystemFrontendBase("mcptam", true)
{
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
    
    static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN|SILENT);
    *gvnLevelZeroPoints = SystemBase::sbLevelZeroPoints;
    
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
  }
  
  // Create the BundleAdjuster, MapMaker, and Tracker objects
  mpBundleAdjuster = new BundleAdjusterMulti(*mpMap, mmCameraModels, true, false, false);
  mpMapMaker = new MapMaker(*mpMap, mmCameraModels, *mpBundleAdjuster);
  mpTracker = new Tracker(*mpMap, *mpMapMaker, mmCameraModels, mmPoses, mmDrawOffsets, mpGLWindow);
  mpKeyFrameViewer = new KeyFrameViewer(*mpMap, *mpGLWindow, mmDrawOffsets, mpVideoSourceMulti->GetSizes());
  
  ImageBWMap masksMap = LoadMasks(); 
  mpTracker->SetMasks(masksMap);
  
  mBookmarkClient = mNodeHandlePriv.serviceClient<std_srvs::Empty>("/rosbag/bookmark");
  mRewindClient = mNodeHandlePriv.serviceClient<std_srvs::Empty>("/rosbag/rewind");
    
  mbDone = false;
  mbDoingTrials = false;
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
  ImageBWMap mFramesBW; 
  
  ros::Time grabStartTime;
  ros::Time grabEndTime;
  ros::Time trackStartTime;
  bool bLastGrabSuccess = true;
  
  mnGrabAttempts = 0;
  mnGrabSuccesses = 0;
  
  // Loop until instructed to stop
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
    ++mnGrabAttempts;
    if(bLastGrabSuccess)
      grabStartTime = ros::Time::now();
    ros::Time timestamp;
    bLastGrabSuccess = mpVideoSourceMulti->GetAndFillFrameBW(ros::WallDuration(0.2), mFramesBW, timestamp);
    grabEndTime = ros::Time::now();
    
    static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN|SILENT);
    bool bDrawKeyFrames = *gvsCurrentSubMenu == "View";
    
    if(bLastGrabSuccess)
    {
      ++mnGrabSuccesses;
      
      if(mpGLWindow)
      {
        // Clear screen with black
        glClearColor(0,0,0,1);
        glClear(GL_COLOR_BUFFER_BIT);
      }
  
      mqFrameGrabDurations.push_back(grabEndTime - grabStartTime);
      if(mqFrameGrabDurations.size() > MAX_STATS_QUEUE_SIZE)
        mqFrameGrabDurations.pop_front();
        
      mqFrameDelayDurations.push_back(grabEndTime - timestamp);
      if(mqFrameDelayDurations.size() > MAX_STATS_QUEUE_SIZE)
        mqFrameDelayDurations.pop_front();
      
      trackStartTime = ros::Time::now();
      // Perform the actual tracking, only draw if we're not drawing keyframes and mpGLWindow is set!
      mpTracker->TrackFrame(mFramesBW, timestamp, !bDrawKeyFrames && mpGLWindow);
    
      // Calculate timings for the loop and tracking steps
      mqTotalDurations.push_back(ros::Time::now() - trackStartTime);
      if(mqTotalDurations.size() > MAX_STATS_QUEUE_SIZE)
        mqTotalDurations.pop_front();
      
      mqLoopTimes.push(ros::Time::now());
      if(mqLoopTimes.size() > MAX_STATS_QUEUE_SIZE)
        mqLoopTimes.pop();
        
      // Send out the tracker state and pose messages from the tracking result
      PublishState();
      PublishPose();
      PublishSmallImage();
      
      if(mpTracker->IsLost() && mbDoingTrials)
      {
        mpMapMaker->RequestRestoreMap();
        
        std_srvs::Empty srv;
        if(!mRewindClient.call(srv))
        {
          ROS_FATAL("Could not call rewind service!");
          ros::shutdown();
        }
        
        mbDoingTrials = mpTracker->NextTrial();
      }
      
      // DEBUG
      //if(mpTracker->GetTrackingQuality() != Tracker::NONE)
      //  mbDone = true;
    }
    else
    {
      // "System: didn't get new frame"
    }
      
    if(bDrawKeyFrames && mpGLWindow)
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
      
    PublishSystemInfo(captionStream);
      
    if(mpGLWindow)
    {
      mpGLWindow->DrawCaption(captionStream.str());
      mpGLWindow->DrawMenus();
      mpGLWindow->swap_buffers();
      mpGLWindow->HandlePendingEvents();
    }
    
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
    else if(params == ".")
    {
      std_srvs::Empty srv;
      bool bSuccess = mBookmarkClient.call(srv);
      if(bSuccess)
      {
        mpMapMaker->RequestBackupMap();
        mpTracker->StartTrials(3);
        
        mbDoingTrials = true;
      }
      else
      {
        ROS_ERROR_STREAM("Could not call bookmark service!");
      }
    }
    
    std::cout<<"Keypress params: "<<params<<std::endl;
    
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
