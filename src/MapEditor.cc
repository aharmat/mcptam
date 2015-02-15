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

#include <mcptam/MapEditor.h>
#include <mcptam/OpenGL.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <cvd/image_io.h>
#include <stdlib.h>
#include <fstream>
#include <gvars3/instances.h>

using namespace GVars3;

MapEditor::MapEditor(std::string windowName)
: mNodeHandlePriv("~")
, mpGLWindow(InitWindow(windowName))
{
  glewInit();
  
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
    
  mNodeHandle.setCallbackQueue(&mCallbackQueueROS);
  mNodeHandlePriv.setCallbackQueue(&mCallbackQueueROS);
  
  mNodeHandlePriv.getParam("save_folder", mSaveFolder);
  ROS_ASSERT(!mSaveFolder.empty());
  
  LoadCamerasFromFolder(mSaveFolder);
  
  mpMap = new Map;  
  mpMap->LoadFromFolder(mSaveFolder, mmPoses, mmCameraModels, false);
  
  mpMapViewer = new MapViewer(*mpMap, *mpGLWindow);
  
  mbDone = false;
}

MapEditor::~MapEditor()
{
  delete mpGLWindow;
  delete mpMap;
  delete mpMapViewer;
}

// This can be used with GUI.RegisterCommand to capture user input
void MapEditor::GUICommandCallBack(void *ptr, std::string command, std::string params)
{
  Command c;
  c.command = command;
  c.params = params;
  static_cast<MapEditor*>(ptr)->mqCommands.push(c);
}

// Creates a new GLWindow2 object
GLWindow2* MapEditor::InitWindow(std::string windowName)
{
  return new GLWindow2(CVD::ImageRef(800,600), windowName);
}

void MapEditor::LoadCamerasFromFolder(std::string folder)
{
  std::string calibrationsFile = folder + "/calibrations.dat";
  std::ifstream file(calibrationsFile);
  if(!file.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<calibrationsFile<<"]");
    ros::shutdown();
    return;
  }
  
  std::string readline;
  std::stringstream readlineSS;
  
  std::string conversion;
  std::stringstream conversionSS;
  
  // First three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of cameras
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numCams = 0;
  readlineSS>>numCams;
  
  std::cout<<"Reading "<<numCams<<" camera models"<<std::endl;
  
  for(int i=0; i < numCams; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // Camera Name, image size (2 vector), projection center (2 vector), polynomial coefficients (5 vector), affine matrix components (3 vector), inverse polynomial coefficents (variable size)
      
      // First is camera name
      std::string camName;
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> camName;
      
      std::cout<<"Read "<<camName<<std::endl;
      
      CVD::ImageRef irImageSize = CVD::ir(util::readVector<2>(readlineSS));
      TooN::Vector<2> v2Center = util::readVector<2>(readlineSS);
      TooN::Vector<5> v5Coeffs = util::readVector<5>(readlineSS);
      TooN::Vector<3> v3Affine = util::readVector<3>(readlineSS);
      
      TooN::Vector<9> v9Params;
      
      // The parameters (by index) are:
      // 0 - a0 coefficient 
      // 1 - a2 coefficient 
      // 2 - a3 coefficient 
      // 3 - a4 coefficient 
      // 4 - center of projection xc 
      // 5 - center of projection yc 
      // 6 - affine transform param c 
      // 7 - affine transform param d
      // 8 - affine transform param e
      
      v9Params[0] = v5Coeffs[0];
      v9Params[1] = v5Coeffs[2];
      v9Params[2] = v5Coeffs[3];
      v9Params[3] = v5Coeffs[4];
      v9Params[4] = v2Center[0];
      v9Params[5] = v2Center[1];
      v9Params[6] = v3Affine[0];
      v9Params[7] = v3Affine[1];
      v9Params[8] = v3Affine[2];
      
      mmCameraModels.insert(std::pair<std::string,TaylorCamera>(camName, TaylorCamera(v9Params, irImageSize, irImageSize, irImageSize)));
    }
    else
    {
      ROS_FATAL_STREAM("Error reading camera calibration file, bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mmCameraModels.size()<<" camera models"<<std::endl;
  
  file.close();
  
  std::string posesFile = folder + "/poses.dat";
  file.open(posesFile);
  if(!file.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<posesFile<<"]");
    ros::shutdown();
    return;
  }
  
  // First three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of poses
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numPoses = 0;
  readlineSS>>numPoses;
  
  std::cout<<"Reading "<<numPoses<<" poses"<<std::endl;
  
  for(int i=0; i < numPoses; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // First is camera name
      std::string camName;
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> camName;
      
      std::cout<<"Got cam name: "<<camName<<std::endl;
      
      // Next is pose
      mmPoses[camName] = (util::readSE3(readlineSS)).inverse();
    }
    else
    {
      ROS_FATAL_STREAM("Error reading camera poses file, bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mmPoses.size()<<" poses"<<std::endl;
}

void MapEditor::Run()
{
  
  // Loop until instructed to stop
  while(!mbDone && ros::ok())
  {
    // Required before drawing
    mpGLWindow->SetupViewport();  
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();
    
    // Clear screen with black
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT);
    
    mpMapViewer->DrawMap();
   
    mpGLWindow->DrawCaption(mpMapViewer->GetMessageForUser());
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

void MapEditor::GUICommandHandler(std::string command, std::string params)  
{
  if(command=="quit" || command == "exit")
  {
    mbDone = true;
    return;
  }
  
  ROS_FATAL_STREAM("System: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}
