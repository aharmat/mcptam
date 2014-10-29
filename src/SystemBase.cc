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

#include <mcptam/SystemBase.h>
#include <mcptam/OpenGL.h>
#include <mcptam/Map.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/Utility.h>
#include <cvd/image_io.h>
#include <stdlib.h>
#include <fstream>

using namespace TooN;

bool SystemBase::sbLevelZeroPoints = false;

SystemBase::SystemBase(std::string windowName, bool bFullSize, bool bDrawWindow)
: mNodeHandlePriv("~")
, mpVideoSourceMulti(InitVideoSource())
, mpGLWindow((bDrawWindow ? InitWindow(windowName,bFullSize) : NULL))
{
  mNodeHandle.setCallbackQueue(&mCallbackQueueROS);
  mNodeHandlePriv.setCallbackQueue(&mCallbackQueueROS);
  
  ImageRefMap mImageSizes = mpVideoSourceMulti->GetSizes();
  ImageRefMap mFullScaleSizes = mpVideoSourceMulti->GetFullScaleSizes();
  ImageRefMap mCalibSizes = mpVideoSourceMulti->GetCalibSizes();
  ParamMap mParams = mpVideoSourceMulti->GetParams();
  
  mmDrawOffsets = mpVideoSourceMulti->GetDrawOffsets();
  mmPosesLive = mpVideoSourceMulti->GetPoses();
  
  std::string cameraPoseFile;
  if(mNodeHandlePriv.getParam("camera_pose_file", cameraPoseFile))
  {
    std::ifstream poseStream(cameraPoseFile.c_str());
    while ( poseStream.good() )
    {
      std::string camName;
      TooN::SE3<> se3Pose;
      
      poseStream>>camName;
      poseStream>>se3Pose;
      
      se3Pose = se3Pose.inverse();
      
      if(!mmPosesLive.count(camName))
        ROS_WARN_STREAM("Got camera name "<<camName<<" in camera pose file but we are not currently using this camera");
      else
      {
        ROS_INFO_STREAM("Replacing "<<camName<<" pose, old pose: "<<std::endl<<mmPosesLive[camName]<<std::endl<<" new pose: "<<std::endl<<se3Pose);
        mmPosesLive[camName] = se3Pose;
      }
    }
  }
  
  // Create camera models here
  for(ImageRefMap::iterator it = mImageSizes.begin(); it != mImageSizes.end(); it++)
  {
    std::string camName = it->first;
    
    CVD::ImageRef& irImageSize = mImageSizes[camName];
    CVD::ImageRef& irFullScaleSize = mFullScaleSizes[camName];
    CVD::ImageRef& irCalibSize = mCalibSizes[camName];
    Vector<9>& v9Params = mParams[camName];
    
    mmCameraModelsLive.insert(std::pair<std::string,TaylorCamera>(camName, TaylorCamera(v9Params, irCalibSize, irFullScaleSize, irImageSize)));
  }
  
  std::cout<<"Creating map"<<std::endl;
  
  mpMap = new Map;  
  
  std::cout<<"Created map"<<std::endl;
  
  mNodeHandlePriv.getParam("save_folder", mSaveFolder);
  
  bool bLoadMap = false;
  mNodeHandlePriv.getParam("load_map", bLoadMap);
  
  if(bLoadMap)
  {
    ROS_ASSERT(!mSaveFolder.empty());
    LoadCamerasFromFolder(mSaveFolder);
    mpMap->LoadFromFolder(mSaveFolder, mmPosesLoaded, mmCameraModelsLoaded);
  }
  
  mmPoses = mmPosesLive;
  mmCameraModels = mmCameraModelsLive;
  
  // Add the loaded poses, checking for duplicates
  for(SE3Map::iterator pose_it = mmPosesLoaded.begin(); pose_it != mmPosesLoaded.end(); ++pose_it)
  {
    std::string camName = pose_it->first;
    TooN::SE3<> se3Pose = pose_it->second;
    
    if(mmPoses.count(camName))  // Make sure that the poses obtained from live and loading are the same!
    {
      TooN::SE3<> se3Diff = mmPoses[camName] * se3Pose.inverse();
      TooN::Vector<6> v6Diff = se3Diff.ln();
      
      double dDiffMagSquared = v6Diff * v6Diff;
      
      if(dDiffMagSquared > 1e-10)
      {
        ROS_FATAL_STREAM("Difference between live and loaded poses for camera "<<camName<<" too great! Diff mag squared: "<<dDiffMagSquared);
        ros::shutdown();
        return;
      }
    }
    else  // insert the pose
    {
      mmPoses[camName] = se3Pose;
    }
  }
  
  // Add the loaded camera models, checking for duplicates
  for(TaylorCameraMap::iterator cam_it = mmCameraModelsLoaded.begin(); cam_it != mmCameraModelsLoaded.end(); ++cam_it)
  {
    std::string camName = cam_it->first;
    TaylorCamera& camera = cam_it->second;
    
    if(mmCameraModels.count(camName))  // Make sure that the model obtained from live and loading are the same!
    {
      TooN::Vector<9> v9Diff = mmCameraModels[camName].GetParams() - camera.GetParams();
      
      double dDiffMagSquared = v9Diff * v9Diff;
      
      if(dDiffMagSquared > 1e-10)
      {
        ROS_FATAL_STREAM("Difference between live and loaded calibration parameters for camera "<<camName<<" too great! Diff mag squared: "<<dDiffMagSquared);
        ros::shutdown();
        return;
      }
    }
    else  // insert the pose
    {
      mmCameraModels[camName] = camera;
    }
  }
}

SystemBase::~SystemBase()
{
  if(mpGLWindow) // window might not have been created if running client in headless mode
    delete mpGLWindow;
    
  if(mpVideoSourceMulti)  // video source might have been shut down already (ie in server)
    delete mpVideoSourceMulti;
    
  delete mpMap;
}

//  Calculates the average of the durations contained in the argument
double SystemBase::AverageDuration(std::deque<ros::Duration>& queue)
{
  ros::Duration durSum(0);
  for(unsigned i=0; i < queue.size(); ++i)
  {
    durSum += queue[i];
  }
  
  double dAvg = durSum.toSec() / queue.size();
  return dAvg;
}

// This can be used with GUI.RegisterCommand to capture user input
void SystemBase::GUICommandCallBack(void *ptr, std::string command, std::string params)
{
  Command c;
  c.command = command;
  c.params = params;
  static_cast<SystemBase*>(ptr)->mqCommands.push(c);
}

// Creates a new GLWindow2 object
GLWindow2* SystemBase::InitWindow(std::string windowName, bool bFullSize)
{
  return new GLWindow2(mpVideoSourceMulti->GetTotalSize(bFullSize), windowName);
}

// Creates a new VideoSourceMulti object
VideoSourceMulti* SystemBase::InitVideoSource()
{ 
  bool bGetPoseSeparately = false;
  mNodeHandlePriv.getParam("get_pose_separately", bGetPoseSeparately);
  
  return new VideoSourceMulti(bGetPoseSeparately);
}

// Load in the masks for the camera images to block certain portions from being searched for point features
ImageBWMap SystemBase::LoadMasks()
{
  ImageBWMap masksMap;
  XmlRpc::XmlRpcValue masks;
  if(mNodeHandlePriv.getParam ("masks", masks))
  {
    if(masks.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("SystemBase: masks parameter should be a dictionary with camera names as keys and mask file names as values");
      ros::shutdown();
    }
    
    std::string masks_dir;
    if(!mNodeHandlePriv.getParam ("masks_dir", masks_dir))
    {
      ROS_FATAL("System: need a masks_dir parameter if using masks specifying directory where mask images are found");
      ros::shutdown();
    }
    
    XmlRpc::XmlRpcValue::ValueStruct::iterator it;
    for (it= masks.begin(); it != masks.end(); ++it)
    {
      std::stringstream ss;
      ss<<masks_dir<<"/"<<std::string(it->second);
      ROS_INFO_STREAM("Loading "<<ss.str());
      masksMap[it->first] = CVD::img_load(ss.str());
    }
  }
  
  return masksMap;
}

// Saves all map information to a file
void SystemBase::SaveCamerasToFolder(std::string folder)
{ 
  std::string calibrationsFile = folder + "/calibrations.dat";
  std::ofstream ofs(calibrationsFile.c_str());
  
  if(!ofs.good())
  {
    ROS_ERROR_STREAM("Couldn't open "<<calibrationsFile<<" to write camera calibrations");
    return;
  }
  
  // First write camera data to file
  ofs<<"% Camera calibration parameters, format:"<<std::endl;
  ofs<<"% Total number of cameras"<<std::endl;
  ofs<<"% Camera Name, image size (2 vector), projection center (2 vector), polynomial coefficients (5 vector), affine matrix components (3 vector)"<<std::endl;
  
  ofs<<mmCameraModels.size()<<std::endl;
  
  for(TaylorCameraMap::iterator cam_it = mmCameraModels.begin(); cam_it != mmCameraModels.end(); ++cam_it)
  {
    std::string camName = cam_it->first;
    TaylorCamera& camera = cam_it->second;
    
    CVD::ImageRef irImageSize = camera.GetImageSize();
    TooN::Vector<9> v9Params = camera.GetParams();
    
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
    
    ofs<<camName<<", "<<irImageSize.x<<", "<<irImageSize.y<<", "<<v9Params[4]<<", "<<v9Params[5];
    ofs<<", "<<v9Params[0]<<", "<<0<<", "<<v9Params[1]<<", "<<v9Params[2]<<", "<<v9Params[3];
    ofs<<", "<<v9Params[6]<<", "<<v9Params[7]<<", "<<v9Params[8];
      
    ofs<<std::endl;
  }
  
  // Done writing
  ofs<<"% The end";
  ofs.close();
  
  std::string posesFile = folder + "/poses.dat";
  ofs.open(posesFile.c_str());
  
  if(!ofs.good())
  {
    ROS_ERROR_STREAM("Couldn't open "<<posesFile<<" to write camera poses");
    return;
  }
  
  ofs<<"% Camera poses in MKF frame, format:"<<std::endl;
  ofs<<"% Total number of cameras"<<std::endl;
  ofs<<"% Camera Name, Position (3 vector), Orientation (quaternion, 4 vector)"<<std::endl;
  
  // Total number of cameras
  ofs<<mmPoses.size()<<std::endl;
  
  for(SE3Map::iterator pose_it = mmPoses.begin(); pose_it != mmPoses.end(); ++pose_it)
  {
    std::string camName = pose_it->first;
    TooN::SE3<> se3Pose = pose_it->second;
    
    // Store the conventional way of defining pose (ie inverse of PTAM)
    geometry_msgs::Pose pose = util::SE3ToPoseMsg(se3Pose.inverse());
    
    ofs<<camName;
    ofs<<", "<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z;
    ofs<<", "<<pose.orientation.x<<", "<<pose.orientation.y<<", "<<pose.orientation.z<<", "<<pose.orientation.w<<std::endl;
  }
  
  // Done writing
  ofs<<"% The end";
  ofs.close();
  
}

void SystemBase::LoadCamerasFromFolder(std::string folder)
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
      
      mmCameraModelsLoaded.insert(std::pair<std::string,TaylorCamera>(camName, TaylorCamera(v9Params, irImageSize, irImageSize, irImageSize)));
    }
    else
    {
      ROS_FATAL_STREAM("Error reading camera calibration file, bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mmCameraModelsLoaded.size()<<" camera models"<<std::endl;
  
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
      mmPosesLoaded[camName] = (util::readSE3(readlineSS)).inverse();
    }
    else
    {
      ROS_FATAL_STREAM("Error reading camera poses file, bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mmPosesLoaded.size()<<" poses"<<std::endl;
  
}

