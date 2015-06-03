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
#include <mcptam/KeyFrameVisualizer.h>
#include <mcptam/OpenGL.h>
#include <mcptam/MapMaker.h>
#include <mcptam/Tracker.h>
#include <mcptam/BundleAdjusterMulti.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/Types.h>
#include <mcptam/Map.h>
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
    GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
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
      +-> Map
      | |
      | +->Back
      | +-> Add MKFs (on/off)
      | +-> Update Points (on/off)
      +-> Keyframes
      | |
      | +-> Back
      | +-> Candidates (on/off)
      | +-> Show Next
      | +-> Show Prev
      | +-> Level
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
    GUI.ParseLine("LevelZeroPoints=1");
    GUI.ParseLine("AddingMKFs=1");
    GUI.ParseLine("UpdatingPoints=1");
    GUI.ParseLine("CrossCamera=1");
    GUI.ParseLine("DrawReloc=0");
    GUI.ParseLine("RelocFabMap=0");
    
    bool bLevelZeroPoints;
    mNodeHandlePriv.param<bool>("level_zero_points", bLevelZeroPoints, true);
    
    static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN|SILENT);
    *gvnLevelZeroPoints = bLevelZeroPoints;
    
    static gvar3<int> gvnAddingMKFs("AddingMKFs", 1, HIDDEN|SILENT);
    static gvar3<int> gvnUpdatingPoints("UpdatingPoints", 1, HIDDEN|SILENT);
    
    // If we loaded a map, disable adding MKFs and updating points by default
    if(mpMap->mbGood)
    {
      std::cout<<"Map is GOOD, disabling adding and updating"<<std::endl;
      *gvnAddingMKFs = 0;
      *gvnUpdatingPoints = 0;
    }
    
    // Main Menu
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Init InitTracker Root");
    GUI.ParseLine("Menu.AddMenuButton Root \"Debug...\" \"\" Debug");
    GUI.ParseLine("Menu.AddMenuButton Root \"Images...\" \"\" Images");
    GUI.ParseLine("Menu.AddMenuButton Root \"Map...\" \"\" Map");
    GUI.ParseLine("Menu.AddMenuButton Root \"Keyframes...\" \"\" Keyframes");
    GUI.ParseLine("Menu.AddMenuSlider Root \"Level\" DrawLevel 0 3 Root");
    GUI.ParseLine("Menu.AddMenuToggle Root \"Level Pts\" DrawOnlyLevel Root");

    // Debug Menu
    GUI.ParseLine("Menu.AddMenuButton Debug \"< Back\" \"\" Root");
    GUI.ParseLine("Menu.AddMenuToggle Debug \"CrossCam\" CrossCamera Debug");
    GUI.ParseLine("Menu.AddMenuToggle Debug \"Draw Reloc\" DrawReloc Debug");
    GUI.ParseLine("Menu.AddMenuToggle Debug \"RelocFabMap\" RelocFabMap Debug");
    GUI.ParseLine("Menu.AddMenuButton Debug \"Save Map\" SaveMap Debug");
    GUI.ParseLine("Menu.AddMenuButton Debug \"Scale Down\" ScaleMapDown Debug");
    GUI.ParseLine("Menu.AddMenuButton Debug \"Scale Up\" ScaleMapUp Debug");
    GUI.ParseLine("Menu.AddMenuButton Debug \"Add MKF\" ManualAddMKF Debug");

    // Images Menu
    GUI.ParseLine("Menu.AddMenuButton Images \"< Back\" \"\" Root");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Level0 Pts\" LevelZeroPoints Images");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Draw Masks\" DrawMasks Images");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Glare Mask\" GlareMasking Images");

    // Map Menu
    GUI.ParseLine("Menu.AddMenuButton Map \"< Back\" \"\" Root");
    GUI.ParseLine("Menu.AddMenuToggle Map \"Adding MKFs\" AddingMKFs Map");
    GUI.ParseLine("Menu.AddMenuToggle Map \"Update Pts\" UpdatingPoints Map");

    // Keyframes Menu
    GUI.ParseLine("Menu.AddMenuButton Keyframes \"< Back\" \"\" Root");
    GUI.ParseLine("Menu.AddMenuToggle Keyframes \"Candidates\" DrawCandidates Keyframes");
    GUI.ParseLine("Menu.AddMenuButton Keyframes \"Show Next\" ShowNextKeyFrame Keyframes");
    GUI.ParseLine("Menu.AddMenuButton Keyframes \"Show Prev\" ShowPrevKeyFrame Keyframes");
    GUI.ParseLine("Menu.AddMenuSlider Keyframes \"Level\" DrawLevel 0 3 Keyframes");

  }
  
  // Create the BundleAdjuster, MapMaker, and Tracker objects
  std::cout<<"Creating Bundle adjuster"<<std::endl;
  mpBundleAdjuster = new BundleAdjusterMulti(*mpMap, mmCameraModels, true, false);
  std::cout<<"Creating MapMaker"<<std::endl;
  mpMapMaker = new MapMaker(*mpMap, *mpRelocFabMap, mmCameraModels, *mpBundleAdjuster);
  std::cout<<"Creating Tracker"<<std::endl;
  mpTracker = new Tracker(*mpMap, *mpRelocFabMap, *mpMapMaker, mmCameraModels, mmPosesLive, mmDrawOffsets, mpGLWindow);
   std::cout<<"Creating keyframe viewer"<<std::endl;
  mpKeyFrameVisualizer = new KeyFrameVisualizer(*mpMap, *mpGLWindow, mmDrawOffsets, mpVideoSourceMulti->GetSizes());
  
  LoadLiveMasks(); 
  mpTracker->SetMasks(mmMasksLive);
    
  mbDone = false;
}

System::~System()
{
  delete mpKeyFrameVisualizer;
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
    bool bDrawKeyFrames = *gvsCurrentSubMenu == "Keyframes";
    
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
        
      mpKeyFrameVisualizer->Draw();
    }
    else
    {
      // Show the main menu
    }
   
    // Update the GUI with the caption info
    std::stringstream captionStream;
    if(bDrawKeyFrames)
      captionStream << mpKeyFrameVisualizer->GetMessageForUser();
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
    mpKeyFrameVisualizer->Next();
    return;
  }
  
  if(command=="ShowPrevKeyFrame")
  {
    mpKeyFrameVisualizer->Prev();
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

  if(command=="SaveMap")
  {
    ROS_ASSERT(!mSaveFolder.empty());
  
    mpMapMaker->RequestMapSave(mSaveFolder);
    SaveCamerasToFolder(mSaveFolder, "saved");
    
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
    }
    else if(params == "f")
    {
      mpTracker->ForceRecovery();
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
