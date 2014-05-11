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
#include <mcptam/OpenGL.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/MapMakerClient.h>
#include <mcptam/Tracker.h>
#include <mcptam/Types.h>
#include <gvars3/instances.h>

using namespace GVars3;
using namespace TooN;

SystemClient::SystemClient()
: SystemFrontendBase("mcptam_client", false)
{
  if(mpGLWindow)
  {
    // Register some commands with GVars, these commands will be triggered
    // by button presses and will result in GUICommandCallBack being called
    GUI.RegisterCommand("exit", GUICommandCallBack, this);
    GUI.RegisterCommand("quit", GUICommandCallBack, this);
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
    
    // Main Menu
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Init InitTracker Root");
    GUI.ParseLine("Menu.AddMenuButton Root \"Images...\" \"\" Images");
    GUI.ParseLine("Menu.AddMenuSlider Root \"Level\" DrawLevel 0 3 Root");
    GUI.ParseLine("Menu.AddMenuToggle Root \"Level Pts\" DrawOnlyLevel Root");

    // Images Menu
    GUI.ParseLine("Menu.AddMenuButton Images \"< Back\" \"\" Root");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Draw Masks\" DrawMasks Images");
    GUI.ParseLine("Menu.AddMenuToggle Images \"Glare Mask\" GlareMasking Images");
  }
  
  mpMapMakerClient = new MapMakerClient(*mpMap);
  mpTracker = new Tracker(*mpMap, *mpMapMakerClient, mmCameraModels, mmPoses, mmDrawOffsets, mpGLWindow);
  
  ImageBWMap masksMap = LoadMasks(); 
  mpTracker->SetMasks(masksMap);
  
  mbDone = false;
}

SystemClient::~SystemClient()
{
  delete mpTracker;
  delete mpMapMakerClient;
}

void SystemClient::Run()
{
  ImageBWMap mFramesBW; 
  
  ros::Time grabStartTime;
  ros::Time grabEndTime;
  ros::Time trackStartTime;
  bool bLastGrabSuccess = true;
  
  mnGrabAttempts = 0;
  mnGrabSuccesses = 0;
  
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
    bool bLastGrabSuccess = mpVideoSourceMulti->GetAndFillFrameBW(ros::WallDuration(0.2), mFramesBW,timestamp);
    grabEndTime = ros::Time::now();
    
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
      // Perform the actual tracking, only draw if mpGLWindow is set!
      mpTracker->TrackFrame(mFramesBW, timestamp, mpGLWindow);
    
      mqTotalDurations.push_back(ros::Time::now() - trackStartTime);
      if(mqTotalDurations.size() > MAX_STATS_QUEUE_SIZE)
        mqTotalDurations.pop_front();
      
      mqLoopTimes.push(ros::Time::now());
      if(mqLoopTimes.size() > MAX_STATS_QUEUE_SIZE)
        mqLoopTimes.pop();
        
      PublishState();
      PublishPose();
      PublishSmallImage();
    }
    else
    {
      // "System: didn't get new frame"
    }
   
    std::stringstream captionStream;
    captionStream << mpTracker->GetMessageForUser();
    
    PublishSystemInfo(captionStream);
      
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
    
    mCallbackQueueROS.callAvailable();
  }
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

