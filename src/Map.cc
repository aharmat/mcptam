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


// Copyright 2008 Isis Innovation Limited

//=========================================================================================
//
// Modifications
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/Utility.h>
#include <mcptam/LevelHelpers.h>
#include <fstream>
#include <cvd/image_io.h>

KeyFrame* translateKF(std::map<MultiKeyFrame*, MultiKeyFrame*>& MKFTranslator, KeyFrame* pKF)
{
  ROS_ASSERT(MKFTranslator.count(pKF->mpParent));
  MultiKeyFrame* pMKFTranslated = MKFTranslator[pKF->mpParent];
  
  ROS_ASSERT(pMKFTranslated->mmpKeyFrames.count(pKF->mCamName));
  return pMKFTranslated->mmpKeyFrames[pKF->mCamName];
}

Map::Map()
{
  mbGoodSnapshot = false;
}

Map::~Map()
{
  Reset();
}

// Clears all lists and deletes their contents, sets mbGood flag to false
void Map::Reset()
{
  std::cout<<"In Map::Reset()"<<std::endl;
  
  EmptyTrash();
  
  boost::mutex::scoped_lock lock(mMutex);
  
  for(MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    while(point.mnUsing != 0)   // This should only be triggered once as points are released fast
    {
      ROS_INFO("Map: Resetting, waiting for point to be given up by tracker...");
      ros::Duration(0.1).sleep();
    }
      
    delete (*point_it);
  }
    
  mlpPoints.clear();
    
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    while(mkf.mnUsing != 0)   // This should only be triggered once as MKFs are released fast
    {
      ROS_INFO("Map: Resetting, waiting for MultiKeyFrame to be given up by tracker by releasing points...");
      ros::Duration(0.1).sleep();
    }
    
    delete (*mkf_it);
  }
  
  mlpMultiKeyFrames.clear();
  
  mbGood = false;
  
  lock.unlock();
  Restore();
}

// Points marked bad are moved to the trash
std::set<MapPoint*> Map::MoveBadPointsToTrash()
{
  std::set<MapPoint*> sBadPoints;
  
  for(MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end(); )
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbBad)
    {
      boost::mutex::scoped_lock lock(mMutex);
      sBadPoints.insert(&point);
      point.EraseAllMeasurements();
      mlpPointsTrash.push_back(&point);
      mlpPoints.erase(point_it++);
    }
    else
      ++point_it;
    
  }
  
  return sBadPoints;
  
}

// MultiKeyFrames marked bad are moved to the trash
std::set<MultiKeyFrame*> Map::MoveBadMultiKeyFramesToTrash()
{
  std::set<MultiKeyFrame*> sBadMKFs;
  
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); )
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
    {
      boost::mutex::scoped_lock lock(mMutex);
      sBadMKFs.insert(&mkf);
      mkf.EraseBackLinksFromPoints();
      mkf.ClearMeasurements();
      mlpMultiKeyFramesTrash.push_back(&mkf);
      mlpMultiKeyFrames.erase(mkf_it++);
    }
    else
      ++mkf_it;
    
  }
  
  return sBadMKFs;
  
}

// Points marked deleted are moved to the trash
void Map::MoveDeletedPointsToTrash()
{
  
  for(MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end(); )
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbDeleted)
    {
      boost::mutex::scoped_lock lock(mMutex);
      point.mbBad = true;   // set bad flag to true, important
      point.EraseAllMeasurements();
      mlpPointsTrash.push_back(&point);
      mlpPoints.erase(point_it++);
    }
    else
      ++point_it;
    
  }
}

// Points marked deleted are moved to the trash
void Map::MoveDeletedMultiKeyFramesToTrash()
{
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); )
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbDeleted)
    {
      boost::mutex::scoped_lock lock(mMutex);
      mkf.mbBad = true;   // set bad flag to true, important
      mkf.EraseBackLinksFromPoints();
      mkf.ClearMeasurements();
      mlpMultiKeyFramesTrash.push_back(&mkf);
      mlpMultiKeyFramesTrash.erase(mkf_it++);
    }
    else
      ++mkf_it;
    
  }
}
    
// Deletes entities that are in the trash
void Map::EmptyTrash()
{
  for(MapPointPtrList::iterator point_it = mlpPointsTrash.begin(); point_it != mlpPointsTrash.end(); )
  {
    MapPoint& point = *(*point_it);
    
    // There should be no KeyFrames measuring this point by now, but just make sure in case
    // my logic was off
    if(point.mnUsing == 0 && point.mMMData.spMeasurementKFs.size() == 0)
    {
      delete (&point);
      mlpPointsTrash.erase(point_it++);
    }
    else
      ++point_it;
  }
  
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFramesTrash.begin(); mkf_it != mlpMultiKeyFramesTrash.end(); )
  {
    MultiKeyFrame& mkf = *(*mkf_it);
  
    // There should be no measurements left in this MultiKeyFrame, but just make sure
    // in case my logic was off
    if(mkf.mnUsing == 0 && mkf.NumMeasurements() == 0)
    {
      delete (&mkf);
      mlpMultiKeyFramesTrash.erase(mkf_it++);
    }
    else
      ++mkf_it;
  }
  
}

// Saves all map information to a file
void Map::SaveToFolder(std::string folder)
{ 
  std::string mapFile = folder + "/map.dat";
  
  std::ofstream ofs(mapFile.c_str());
  
  if(!ofs.good())
  {
    ROS_ERROR_STREAM("Couldn't open "<<mapFile<<" to write map");
    return;
  }
  
  // IMPORTANT! Set precision so that the discrepancy between saved and then reloaded maps is very very small
  ofs.precision(20);
  
  boost::mutex::scoped_lock lock(mMutex);
  
  // First write BaseFromWorld for each MKF in system
  ofs<<"% MKFs in world frame, format:"<<std::endl;
  ofs<<"% Total number of MKFs"<<std::endl;
  ofs<<"% MKF number, Position (3 vector), Orientation (quaternion, 4 vector)"<<std::endl;
  
  // Total number of MKFs
  ofs<<mlpMultiKeyFrames.size()<<std::endl;
  
  int i=0;
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++i, ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    mkf.mnID = i;
    
    // Store the conventional way of defining pose (ie inverse of PTAM)
    geometry_msgs::Pose pose = util::SE3ToPoseMsg(mkf.mse3BaseFromWorld.inverse());
    
    ofs<<i;
    ofs<<", "<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z;
    ofs<<", "<<pose.orientation.x<<", "<<pose.orientation.y<<", "<<pose.orientation.z<<", "<<pose.orientation.w<<std::endl;
  }
  
  // Now write map point positions
  ofs<<"% Points in world frame, format:"<<std::endl;
  ofs<<"% Total number of points"<<std::endl;
  ofs<<"% Point number, Position (3 vector), Parent MKF number, Parent camera name, Fixed, Optimized"<<std::endl;
  
  // Total number of points
  ofs<<mlpPoints.size()<<std::endl;
  
  int nTotalMeas = 0;
  i=0;
  for(MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end(); ++i, ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mnID = i;
    
    ofs<<i;
    ofs<<", "<<point.mv3WorldPos[0]<<", "<<point.mv3WorldPos[1]<<", "<<point.mv3WorldPos[2];
    ofs<<", "<<point.mpPatchSourceKF->mpParent->mnID<<", "<<point.mpPatchSourceKF->mCamName;
    ofs<<", "<<point.mbFixed<<", "<<point.mbOptimized<<std::endl;
    
    nTotalMeas += point.mMMData.spMeasurementKFs.size();
  }
  
  // Now write measurements
  ofs<<"% Measurements of points from KeyFrames, format: "<<std::endl;
  ofs<<"% Total number of measurements"<<std::endl;
  ofs<<"% MKF number, camera name, point number, image position (2 vector) at level 0, level, subpix, source"<<std::endl;
  
  // Total number of measurements
  ofs<<nTotalMeas<<std::endl;
  
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
        Measurement& meas = *(meas_it->second);
        
        ofs<<mkf.mnID<<", "<<kf.mCamName<<", "<<point.mnID<<", ";
        ofs<<meas.v2RootPos[0]<<", "<<meas.v2RootPos[1]<<", "<<meas.nLevel<<", "<<meas.bSubPix<<", "<<meas.eSource<<std::endl;
      }
    }
  }
  
  // Done writing
  ofs<<"% The end";
  ofs.close();
  
  
  // Now write the images
  
  std::map<std::string, CVD::Parameter<> > param;
  param["jpeg.quality"] = CVD::Parameter<int>(90);
  
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++i, ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      
      std::stringstream ss;
      std::ofstream imageStream;
      
      if(kf.maLevels[0].image.totalsize() > 0)
      {
        ss<<folder<<"/mkf"<<mkf.mnID<<"_"<<camName<<"_image.jpg";
        imageStream.open(ss.str().c_str());
        if(!imageStream.is_open())
        {
          ROS_FATAL_STREAM("Couldn't open file ["<<ss.str()<<"]");
          ros::shutdown();
          return;
        }
        
        CVD::img_save<CVD::byte>(kf.maLevels[0].image, imageStream, CVD::ImageType::JPEG, param);
        
        imageStream.close();
        
        if(kf.maLevels[0].mask.totalsize() > 0)
        {
          ss.str(std::string());
          ss.clear();
          ss<<folder<<"/mkf"<<mkf.mnID<<"_"<<camName<<"_mask.jpg";
          imageStream.open(ss.str().c_str());
          
          if(!imageStream.is_open())
          {
            ROS_FATAL_STREAM("Couldn't open file ["<<ss.str()<<"]");
            ros::shutdown();
            return;
          }
          
          CVD::img_save<CVD::byte>(kf.maLevels[0].mask, imageStream, CVD::ImageType::JPEG, param);
        }
      }
    }
  }
  
}

// Load all map information to a file
void Map::LoadFromFolder(std::string folder, SE3Map mPoses, TaylorCameraMap mCameraModels, bool bFix)
{ 
  Reset();
  
  std::string mapFile = folder + "/map.dat";
  
  std::ifstream file(mapFile);
  if(!file.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<mapFile<<"]");
    ros::shutdown();
    return;
  }
  
  std::string readline;
  std::stringstream readlineSS;
  
  std::string conversion;
  std::stringstream conversionSS;
  
  // Three parts to the file: MKF poses in world, 
  // map points in world, measurements between KFs and points
  
  // Translation map for MKF id -> MKF pointer and point id -> point pointer
  std::map<int, MultiKeyFrame*> mID_To_MKF;
  std::map<int, MapPoint*> mID_To_Point;
  
  // Load MKF poses, create MKFs on the fly
  // First three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of MKFs
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numMKFs = 0;
  readlineSS>>numMKFs;
  
  std::cout<<"Reading "<<numMKFs<<" MKFs"<<std::endl;
  
  for(int i=0; i < numMKFs; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // MKF number, Position (3 vector), Orientation (quaternion, 4 vector)
      MultiKeyFrame* pMKF = new MultiKeyFrame;
      pMKF->mbFixed = bFix ? true : (i == 0);
      
      // First is MKF id
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> pMKF->mnID;
      mID_To_MKF[pMKF->mnID] = pMKF;
      
      // Next is pose
      pMKF->mse3BaseFromWorld = (util::readSE3(readlineSS)).inverse();
      
      for(SE3Map::iterator it = mPoses.begin(); it != mPoses.end(); ++it)
      {
        std::string camName = it->first;
        std::cout<<"Assigning "<<camName<<" to KF"<<std::endl;
        KeyFrame* pKF = new KeyFrame(pMKF, camName);
        pKF->mbActive = true;
        pKF->mse3CamFromBase = it->second;
        pKF->mse3CamFromWorld = pKF->mse3CamFromBase * pMKF->mse3BaseFromWorld;
        pMKF->mmpKeyFrames[camName] = pKF;
        
        // Read mask and image
        CVD::Image<CVD::byte> imImage, imMask;
        std::stringstream ss;
        std::ifstream imageFile;
        
        ss<<folder<<"/mkf"<<pMKF->mnID<<"_"<<camName<<"_image.jpg";
        imageFile.open(ss.str().c_str());
        if(!imageFile.is_open())
        {
          ROS_WARN_STREAM("Couldn't open image file ["<<ss.str()<<"], not assigning KF any image! This could be ok if KF was used during IDP init");
          ROS_WARN_STREAM("If any points try to set this KF as a source, we're gonna make a fuss");
        }
        else
        {
		  imImage = CVD::img_load(imageFile);
		  ROS_ASSERT(imImage.totalsize() > 0);
        
		  ss.str(std::string());
		  ss.clear();
		  imageFile.close();
		  ss<<folder<<"/mkf"<<pMKF->mnID<<"_"<<camName<<"_mask.jpg";
		  imageFile.open(ss.str().c_str());
	      if(!imageFile.is_open())
		  {
		    ROS_WARN_STREAM("Couldn't open mask file ["<<ss.str()<<"], assuming no mask exists");
		  }
		  else
		  {
		    imMask = CVD::img_load(imageFile);
		  }
	    }
        
        // Set mask before creating rest of keyframe internals
        if(imMask.totalsize() > 0)
          pKF->SetMask(imMask);
    
        // Don't do deep copy of image since we just created it and nobody else will use it
        // The handling of the mask is not right at the moment, since glare masking is decided 
        // on the tracker side and not saved into the KF, so just don't do it for now....
        if(imImage.totalsize() > 0)
        {
          pKF->MakeKeyFrame_Lite(imImage, false, false);
          pKF->MakeKeyFrame_Rest();
	    }
      }
      
      mlpMultiKeyFrames.push_back(pMKF);
    }
    else
    {
      ROS_FATAL_STREAM("Error reading map file (while reading MKFs), bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mlpMultiKeyFrames.size()<<" MKFs"<<std::endl;
  
  // Now load points, create points on the fly
  // Next three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of Points
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numPoints = 0;
  readlineSS>>numPoints;
  
  std::cout<<"Reading "<<numPoints<<" points"<<std::endl;
  
  for(int i=0; i < numPoints; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // Point number, Position (3 vector), Parent MKF number, Parent camera name
      MapPoint *pPointNew = new MapPoint;
      
      // First is point id
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> pPointNew->mnID;
      mID_To_Point[pPointNew->mnID] = pPointNew;
      
      pPointNew->mv3WorldPos = util::readVector<3>(readlineSS);
      
      int nParentMKF;
      std::string camName;
      int nFixed;
      int nOptimized;
      
      // Parent MKF id
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nParentMKF;
      
      // Parent camera name
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> camName;
      
      // Fixed flag
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nFixed;
      pPointNew->mbFixed = bFix ? true : static_cast<bool>(nFixed);
      
      // Optimized flag
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nOptimized;
      pPointNew->mbOptimized = static_cast<bool>(nOptimized);
      
      pPointNew->mpPatchSourceKF = mID_To_MKF[nParentMKF]->mmpKeyFrames[camName];
      ROS_ASSERT(pPointNew->mpPatchSourceKF->maLevels[0].image.totalsize() > 0);  // make sure we loaded an image if this KF is considered a patch source
      
      mlpPoints.push_back(pPointNew);
    }
    else
    {
      ROS_FATAL_STREAM("Error reading map file (while reading points), bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mlpPoints.size()<<" points"<<std::endl;
  
  // Now load measurements, create them on the fly
  // Next three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of measurements
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numMeas = 0;
  readlineSS>>numMeas;
  
  std::cout<<"Reading "<<numMeas<<" measurements"<<std::endl;
  int measCounter = 0;
  
  for(int i=0; i < numMeas; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // MKF number, camera name, point number, image position (2 vector) at level 0, measurement noise
      Measurement* pMeas = new Measurement;
      
      int nSourceMKF;
      std::string camName;
      int nSourcePoint;
      int nSubPix;
      int nSource;
      
      // First is MKF id
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nSourceMKF;
      
      // Next is camera name
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS>>camName;
      
      // Next is point id
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nSourcePoint;
      
      // Next is image position
      pMeas->v2RootPos = util::readVector<2>(readlineSS);
      
      // Next is level
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> pMeas->nLevel;
      
      // Next is sub pix
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nSubPix;
      pMeas->bSubPix = static_cast<bool>(nSubPix);
      
      // Finally, the measurement source
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> nSource;
      pMeas->eSource = static_cast<Measurement::Src>(nSource);
      
      KeyFrame* pKF = mID_To_MKF[nSourceMKF]->mmpKeyFrames[camName];
      MapPoint* pPoint = mID_To_Point[nSourcePoint];
      
      if(pPoint->mpPatchSourceKF == pKF)
      {
        // Update some point variables
        int nLevelScale = LevelScale(pMeas->nLevel);
        pPoint->mnSourceLevel = pMeas->nLevel;
        pPoint->mirCenter = CVD::ir_rounded(LevelNPos(pMeas->v2RootPos, pMeas->nLevel));
        pPoint->mv3Center_NC = mCameraModels[camName].UnProject(pMeas->v2RootPos);
        pPoint->mv3OneRightFromCenter_NC = mCameraModels[camName].UnProject(pMeas->v2RootPos + CVD::vec(CVD::ImageRef(nLevelScale,0))); 
        pPoint->mv3OneDownFromCenter_NC  = mCameraModels[camName].UnProject(pMeas->v2RootPos + CVD::vec(CVD::ImageRef(0,nLevelScale)));
        
        pPoint->RefreshPixelVectors();
      }
      
      pKF->mmpMeasurements[pPoint] = pMeas;
      pPoint->mMMData.spMeasurementKFs.insert(pKF);
      
      measCounter++;
    }
    else
    {
      ROS_FATAL_STREAM("Error reading map file (while reading measurements), bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<measCounter<<" measurements"<<std::endl;
  
  file.close();
  
  // Refresh the scene depths for the MKFs
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    mkf.RefreshSceneDepthRobust();
  }
  
  mbGood = true;
  MakeSnapshot();
  
}

void Map::MakeSnapshot()
{
  std::cout<<"In Map::MakeSnapshot"<<std::endl;
  
  // Make a copy of all points and MKFs in both live map and trash, put them in the appropriate
  // snapshot variables
  
  // Erase snapshot variables first
  // Don't need to lock mutex yet since nobody is using the snapshots
  
  for(MapPointPtrList::iterator point_it = mlpPointsSnapshot.begin(); point_it != mlpPointsSnapshot.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    ROS_ASSERT(point.mnUsing == 0);  // nobody should be using the snapshots
    delete (*point_it);
  }
    
  mlpPointsSnapshot.clear();
    
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFramesSnapshot.begin(); mkf_it != mlpMultiKeyFramesSnapshot.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    ROS_ASSERT(mkf.mnUsing == 0);  // nobody should be using the snapshots
    delete (*mkf_it);
  }
  
  mlpMultiKeyFramesSnapshot.clear();
  
  // Empty the trash, so we don't have to deal with saving anything from there
  while( !(mlpMultiKeyFramesTrash.empty() && mlpPointsTrash.empty()) )
  {
    std::cout<<"Emptying trash"<<std::endl;
    EmptyTrash();
    ros::Duration(0.1).sleep();
  }
  
  
  // Now we can actually make copies of the live map
  // so lock mutex
  boost::mutex::scoped_lock lock(mMutex);
  
  mbGoodSnapshot = mbGood;
  
  // Sequence of actions:
  // 1. Make copies of MKF data (not the measurements, they require Points)
  // 2. Make copies of Point data (fill in with new MKF-related pointers)
  // 3. Make copies of measurements (fill in with new Point related pointers)
  
  // Need to store live->snapshot translation maps for this to work
  
  std::map<MultiKeyFrame*, MultiKeyFrame*> MKFTranslator;
  std::map<MapPoint*, MapPoint*> PointTranslator;
  
  // Creat the new MKFs
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    MultiKeyFrame* pMKFSnapshot = new MultiKeyFrame;
    
    pMKFSnapshot->mse3BaseFromWorld = mkf.mse3BaseFromWorld;
    pMKFSnapshot->mdTotalDepthMean = mkf.mdTotalDepthMean;
    pMKFSnapshot->mnID = mkf.mnID;
    pMKFSnapshot->mbFixed = mkf.mbFixed;
    pMKFSnapshot->mbBad = mkf.mbBad;
    pMKFSnapshot->mbDeleted = mkf.mbDeleted;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      KeyFrame* pKFSnapshot = new KeyFrame(pMKFSnapshot, camName);
      pMKFSnapshot->mmpKeyFrames[camName] = pKFSnapshot;
      
      pKFSnapshot->mse3CamFromBase = kf.mse3CamFromBase;
      pKFSnapshot->mse3CamFromWorld = kf.mse3CamFromWorld;
      pKFSnapshot->mdSceneDepthMean = kf.mdSceneDepthMean;
      pKFSnapshot->mdSceneDepthSigma = kf.mdSceneDepthSigma;
      pKFSnapshot->mbActive = kf.mbActive;
      
      for(int i=0; i < LEVELS; ++i)
      {
        Level& level = kf.maLevels[i]; 
        Level& levelSnapshot = pKFSnapshot->maLevels[i];
        
        levelSnapshot.lastMask = level.lastMask;
        levelSnapshot.mask = level.mask;
        levelSnapshot.image = level.image;
        levelSnapshot.vCorners = level.vCorners;
        levelSnapshot.vCornerRowLUT = level.vCornerRowLUT;
        levelSnapshot.vCandidates = level.vCandidates;
        levelSnapshot.vScoresAndMaxCorners = level.vScoresAndMaxCorners;
        levelSnapshot.vFastFrequency = level.vFastFrequency;
        levelSnapshot.nFastThresh = level.nFastThresh;
        levelSnapshot.imagePrev = level.imagePrev;
        levelSnapshot.vCornersPrev = level.vCornersPrev; 
      }
      
      // Only do this after the level 0 image has been filled in
      // But only if the original KF had an SBI
      if(kf.mpSBI)
        pKFSnapshot->MakeSBI();      
    }
  
    mlpMultiKeyFramesSnapshot.push_back(pMKFSnapshot);
    MKFTranslator[&mkf] = pMKFSnapshot;
  }
  
  // Create the new points, with correct linking to the new MKFs/KFs
  for(MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    MapPoint* pPointSnapshot = new MapPoint;
    
    pPointSnapshot->mv3WorldPos = point.mv3WorldPos;
    pPointSnapshot->mbBad = point.mbBad;
    pPointSnapshot->mbDeleted = point.mbDeleted;
    pPointSnapshot->mbFixed = point.mbFixed;
    pPointSnapshot->mbOptimized = point.mbOptimized;
    pPointSnapshot->mpPatchSourceKF = translateKF(MKFTranslator, point.mpPatchSourceKF);
    pPointSnapshot->mnSourceLevel = point.mnSourceLevel;
    pPointSnapshot->mirCenter = point.mirCenter;
    pPointSnapshot->mv3Center_NC = point.mv3Center_NC;
    pPointSnapshot->mv3OneDownFromCenter_NC = point.mv3OneDownFromCenter_NC;
    pPointSnapshot->mv3OneRightFromCenter_NC = point.mv3OneRightFromCenter_NC;
    pPointSnapshot->mv3PixelDown_W = point.mv3PixelDown_W;
    pPointSnapshot->mv3PixelRight_W = point.mv3PixelRight_W;

    for(std::set<KeyFrame*>::iterator kf_it = point.mMMData.spMeasurementKFs.begin(); kf_it != point.mMMData.spMeasurementKFs.end(); ++kf_it)
    {
      pPointSnapshot->mMMData.spMeasurementKFs.insert(translateKF(MKFTranslator, *kf_it));
    }
    
    for(std::set<KeyFrame*>::iterator kf_it = point.mMMData.spNeverRetryKFs.begin(); kf_it != point.mMMData.spNeverRetryKFs.end(); ++kf_it)
    {
      pPointSnapshot->mMMData.spNeverRetryKFs.insert(translateKF(MKFTranslator, *kf_it));
    }
    
    pPointSnapshot->mnMEstimatorOutlierCount = point.mnMEstimatorOutlierCount;
    pPointSnapshot->mnMEstimatorInlierCount = point.mnMEstimatorInlierCount;
    pPointSnapshot->mnID = point.mnID;
    
    mlpPointsSnapshot.push_back(pPointSnapshot);
    PointTranslator[&point] = pPointSnapshot;
  }
  
  // Create the new measurements, linking to the correct new points
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    MultiKeyFrame* pMKFSnapshot = MKFTranslator[&mkf];
  
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      KeyFrame* pKFSnapshot = pMKFSnapshot->mmpKeyFrames[camName];
      
      for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
        Measurement& meas = *(meas_it->second);
        
        ROS_ASSERT(PointTranslator.count(&point));
        MapPoint* pPointSnapshot = PointTranslator[&point];
        Measurement* pMeasSnapshot = new Measurement;
        
        pMeasSnapshot->nLevel = meas.nLevel;
        pMeasSnapshot->bSubPix = meas.bSubPix;
        pMeasSnapshot->v2RootPos = meas.v2RootPos;
        pMeasSnapshot->eSource = meas.eSource;
        pMeasSnapshot->bTransferred = meas.bTransferred;
        pMeasSnapshot->nID = meas.nID;
        
        pKFSnapshot->mmpMeasurements[pPointSnapshot] = pMeasSnapshot;
      }
    }
  }
  
}

void Map::Restore()
{
  //ROS_ASSERT(!mlpPointsSnapshot.empty() && !mlpMultiKeyFramesSnapshot.empty());
  
  std::cout<<"In Map::Restore"<<std::endl;
  
  boost::mutex::scoped_lock lock(mMutex);
  
  mbGood = mbGoodSnapshot;
  mlpPoints.swap(mlpPointsSnapshot);
  mlpMultiKeyFrames.swap(mlpMultiKeyFramesSnapshot);   
  
  lock.unlock();  // MakeSnapshot will try to lock mutex
  
  // Need to overwrite the new stuff in the "snapshot" variables with copies
  // of the restored map
  MakeSnapshot();
}

bool Map::Contains(MultiKeyFrame* pMKF)
{
  for(MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    if(pMKF == *mkf_it)
      return true;
  }
  
  return false;
}

