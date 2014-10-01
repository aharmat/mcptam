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

KeyFrame* translateKF(std::map<MultiKeyFrame*, MultiKeyFrame*>& MKFTranslator, KeyFrame* pKF)
{
  ROS_ASSERT(MKFTranslator.count(pKF->mpParent));
  MultiKeyFrame* pMKFTranslated = MKFTranslator[pKF->mpParent];
  
  ROS_ASSERT(pMKFTranslated->mmpKeyFrames.count(pKF->mCamName));
  return pMKFTranslated->mmpKeyFrames[pKF->mCamName];
}


Map::Map()
{
  Reset();
}

Map::~Map()
{
  Reset();
}

// Clears all lists and deletes their contents, sets mbGood flag to false
void Map::Reset()
{
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
      point.EraseAllCrossCov();
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

void Map::MakeSnapshot()
{
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
    EmptyTrash();
    ros::Duration(0.01).sleep();
  }
  
  
  // Now we can actually make copies of the live map
  // so lock mutex
  boost::mutex::scoped_lock lock(mMutex);
  
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
    pPointSnapshot->mm3WorldCov = point.mm3WorldCov;
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
    pPointSnapshot->mv3Normal_NC = point.mv3Normal_NC;
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
  ROS_ASSERT(!mlpPointsSnapshot.empty() && !mlpMultiKeyFramesSnapshot.empty());
  
  boost::mutex::scoped_lock lock(mMutex);
  
  mlpPoints.swap(mlpPointsSnapshot);
  mlpMultiKeyFrames.swap(mlpMultiKeyFramesSnapshot);   
  
  lock.unlock();  // MakeSnapshot will try to lock mutex
  
  // Need to overwrite the new stuff in the "snapshot" variables with copies
  // of the restored map
  MakeSnapshot();
}



