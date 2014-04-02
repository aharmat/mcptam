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
#include <cvd/image_io.h>
#include <stdlib.h>
#include <fstream>

using namespace TooN;

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
  mmPoses = mpVideoSourceMulti->GetPoses();
  
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
      
      if(!mmPoses.count(camName))
        ROS_WARN_STREAM("Got camera name "<<camName<<" in camera pose file but we are not currently using this camera");
      else
      {
        ROS_INFO_STREAM("Replacing "<<camName<<" pose, old pose: "<<std::endl<<mmPoses[camName]<<std::endl<<" new pose: "<<std::endl<<se3Pose);
        mmPoses[camName] = se3Pose;
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
    
    mmCameraModels.insert(std::pair<std::string,TaylorCamera>(camName, TaylorCamera(v9Params, irCalibSize, irFullScaleSize, irImageSize)));
  }
  
  mpMap = new Map;  
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

// Write the camera parameters to the given file name
void SystemBase::DumpCamerasToFile(std::string filename)
{
  std::ofstream ofs(filename.c_str());
  
  if(!ofs.good())
  {
    ROS_ERROR_STREAM("Couldn't open "<<filename<<" to write cameras");
    return;
  }
  
  // Write camera data to file
  ofs<<"% Camera calibration parameters, format:"<<std::endl;
  ofs<<"% Total number of cameras"<<std::endl;
  ofs<<"% Camera Name, image size (2 vector), projection center (2 vector), polynomial coefficients (5 vector), affine matrix components (3 vector), inverse polynomial coefficents (variable size)"<<std::endl;
  
  ofs<<mmCameraModels.size()<<std::endl;
  
  for(TaylorCameraMap::iterator cam_it = mmCameraModels.begin(); cam_it != mmCameraModels.end(); ++cam_it)
  {
    std::string camName = cam_it->first;
    TaylorCamera& camera = cam_it->second;
    
    CVD::ImageRef irImageSize = camera.GetImageSize();
    TooN::Vector<9> v9Params = camera.GetParams();
    TooN::Vector<TooN::Resizable> vxInvPoly = camera.GetInvPoly();
    
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
    
    for(int i=0; i < vxInvPoly.size(); ++i)
      ofs<<", "<<vxInvPoly[i];
      
    ofs<<std::endl;
  }
  
  ofs<<"% The end";
  ofs.close();
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
