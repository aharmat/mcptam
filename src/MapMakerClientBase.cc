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
// Large parts of this code are from the original PTAM, which are
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/MapMakerClientBase.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/Map.h>
#include <mcptam/Tracker.h>
#include <mcptam/LevelHelpers.h>
#include <gvars3/instances.h>

int MapMakerClientBase::snMinOutliers = 20;
double MapMakerClientBase::sdOutlierMultiplier = 1.0;
double MapMakerClientBase::sdMaxScaledMKFDist = 0.1; //0.3;
double MapMakerClientBase::sdMaxScaledKFDist = 0.1; //0.3;

MapMakerClientBase::MapMakerClientBase(Map& map, TaylorCameraMap &cameras)
  : MapMakerBase(map, cameras, true)  // This will be skipped since inheritance is virtual!
{
  Reset();
};

void MapMakerClientBase::Reset()
{
  // The class deriving from this one should call Reset on MapMakerBase
  
  ROS_DEBUG("MapMakerClientBase: Reset");
  
  boost::mutex::scoped_lock lock(mQueueMutex);
  while(!mqpMultiKeyFramesFromTracker.empty())
  {
    delete mqpMultiKeyFramesFromTracker.front();
    mqpMultiKeyFramesFromTracker.pop_front();
  }
  
  boost::mutex::scoped_lock lock2(mCovMutex);
  mvCovHelpers.clear();
  mvCovHelpers.resize(2);
  mnCurrCov = 0;
}

// Points that are considered outliers by the Tracker are marked with a bad flag
void MapMakerClientBase::MarkOutliersAsBad()
{
  int nMarkedBad = 0;
  // Did the tracker see this point as an outlier more often than as an inlier?
  for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); it++)
  {
    MapPoint &point = *(*it);
    if(point.mbFixed)  // fixed points are exempt from being outliers
      continue;
      
    if(point.mnMEstimatorOutlierCount > snMinOutliers && point.mnMEstimatorOutlierCount > point.mnMEstimatorInlierCount * MapMakerClientBase::sdOutlierMultiplier)
    {
      point.mbBad = true;
      nMarkedBad++;
    }
  }
  
  /*
  if(nMarkedBad > 0)
  {
    ROS_INFO_STREAM("======== Handle outlier based on tracker marked "<<nMarkedBad<<" points as bad");
  }
  */
}

// Points that have fewer than two measurements are marked as outliers. If the point is fixed, one measurement is enough
void MapMakerClientBase::MarkDanglersAsBad()
{
  if(mMap.mlpMultiKeyFrames.size() < 2)  // even during idp init, after the second MKF has been added we shouldn't have danglers left
    return;
    
  for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it)
  {
    MapPoint &point = *(*it);
    if(point.mMMData.GoodMeasCount() < 2 && !point.mbFixed)
      point.mbBad = true;
  }
}

bool MapMakerClientBase::NeedNewMultiKeyFrame(MultiKeyFrame &mkf, TooN::Matrix<6> m6Cov)
{
  if(TrackerQueueSize() > 2)
  {
    ROS_WARN("MapMakerClientBase::NeedNewMultiKeyFrame: Queue size too large, returning false");
    return false;
  }
    
  if(mState == MM_INITIALIZING)  // always need the new MKF when initializing or just finished
  {
    ROS_INFO("NeedNewMultiKeyFrame: initializing, returning true");
    return true;
  }
  
  // See if there's anything close in the queue
  MultiKeyFrame *pClosestMKFInQueue = ClosestMultiKeyFrameInQueue(mkf);
  if(pClosestMKFInQueue)
  {
    double dDistInQueue = mkf.Distance(*pClosestMKFInQueue);
    ROS_DEBUG_STREAM("Closest dist in queue: "<<dDistInQueue);
    dDistInQueue *= (1.0 / mkf.mdTotalDepthMean);  // scale by depth
    
    int nEffectiveSize = mMap.mlpMultiKeyFrames.size();
    if(nEffectiveSize == 2)
      nEffectiveSize = 1;
    
    double dMapSizeFactor = 1.0 - (1.0 / (0.5 + nEffectiveSize));
    double dThresh = MapMakerClientBase::sdMaxScaledMKFDist * dMapSizeFactor;
    
    if(dDistInQueue < dThresh)
      return false;
  }
  
  double dCovMag = TooN::norm_fro(m6Cov);
  ROS_INFO_STREAM("NeedNewMultiKeyFrame: cov mag: "<<dCovMag);
  
  return dCovMag > 1e-5;
}

bool MapMakerClientBase::NeedNewMultiKeyFrame(MultiKeyFrame &mkf, double dMAD)
{
  if(TrackerQueueSize() > 0)
  {
    ROS_WARN("MapMakerClientBase::NeedNewMultiKeyFrame: Queue size too large, returning false");
    return false;
  }
    
  if(mState == MM_INITIALIZING)  // always need the new MKF when initializing or just finished
  {
    ROS_INFO("NeedNewMultiKeyFrame: initializing, returning true");
    return true;
  }
  
  return dMAD > 0.7;
}

// Checks to see if the given MultiKeyFrame is a candidate to be added to the Map
bool MapMakerClientBase::NeedNewMultiKeyFrame(MultiKeyFrame &mkf)
{
  if(TrackerQueueSize() > 2)
  {
    ROS_DEBUG("MapMakerClientBase::NeedNewMultiKeyFrame: Queue size too large, returning false");
    return false;
  }
    
  if(mState == MM_INITIALIZING)  // always need the new MKF when initializing or just finished
  {
    ROS_DEBUG("NeedNewMultiKeyFrame: initializing, returning true");
    return true;
  }
  
  MultiKeyFrame *pClosestMKF = ClosestMultiKeyFrame(mkf);
  double dDist = mkf.Distance(*pClosestMKF);
  
  ROS_DEBUG_STREAM("Closest dist in map: "<<dDist);
  
  // See if there's anything closer in the queue
  MultiKeyFrame *pClosestMKFInQueue = ClosestMultiKeyFrameInQueue(mkf);
  if(pClosestMKFInQueue)
  {
    double dDistInQueue = mkf.Distance(*pClosestMKFInQueue);
    ROS_DEBUG_STREAM("Closest dist in queue: "<<dDistInQueue);
    dDist = std::min(dDist,dDistInQueue);
  }
  
  dDist *= (1.0 / mkf.mdTotalDepthMean);  // scale by depth
  
  int nEffectiveSize = mMap.mlpMultiKeyFrames.size();
  if(nEffectiveSize == 2)
    nEffectiveSize = 1;
    
  double dMapSizeFactor = 1.0 - (1.0 / (0.5 + nEffectiveSize));
  double dThresh = MapMakerClientBase::sdMaxScaledMKFDist * dMapSizeFactor;
  bool bNeed = dDist > dThresh;
  
  //ROS_INFO_STREAM("NeedNewMultiKeyFrame: dist: "<<dDist<<" mean depth: "<<mkf.mdTotalDepthMean<<" thresh: "<< dThresh << " need: "<<(bNeed ? "yes" : "no"));
  
  return bNeed;
}

// Checks to see if the given MultiKeyFrame is a candidate to be added to the Map
bool MapMakerClientBase::NeedNewMultiKeyFrame(MultiKeyFrame &mkf, int nNumMeas)
{
  if(TrackerQueueSize() > 2)
  {
    ROS_DEBUG("MapMakerClientBase::NeedNewMultiKeyFrame: Queue size too large, returning false");
    return false;
  }
  
  std::vector<MultiKeyFrame*> vpClosestMKFs = NClosestMultiKeyFrames(mkf, 3);
  ROS_ASSERT(vpClosestMKFs.size() > 0);
  
  int nClosestMeasSum = 0;
  
  for(unsigned i=0; i < vpClosestMKFs.size(); ++i)
    nClosestMeasSum += vpClosestMKFs[i]->NumMeasurements();
  
  // Arbitrarily choose 70% of the average measurement number to be the minimum, otherwise declare that we need new MKF
  double dThresh = 0.7 * ((double)nClosestMeasSum / vpClosestMKFs.size());
  bool bNeed = nNumMeas < dThresh;
  
  ROS_INFO_STREAM("NeedNewMultiKeyFrame: num meas: "<<nNumMeas<<" thresh: "<<dThresh<<" need: "<<(bNeed ? "yes" : "no"));
  
  return nNumMeas < dThresh;
}

// Checks to see if the given KeyFrame is a candidate to be added to the Map
bool MapMakerClientBase::NeedNewKeyFrame(KeyFrame &kf, bool bSameCamName)
{
  if(TrackerQueueSize() > 2)
  {
    ROS_DEBUG("MapMakerClientBase::NeedNewMultiKeyFrame: Queue size too large, returning false");
    return false;
  }
  
  KeyFrame *pClosestKF = ClosestKeyFrame(kf, KF_ALL, bSameCamName);
  
  if(pClosestKF == NULL)
    return false;
  
  double dDist = kf.Distance(*pClosestKF);
  dDist *= (1.0 / kf.mdSceneDepthMean); // scale by depth
  
  bool bNeed = dDist > MapMakerClientBase::sdMaxScaledKFDist;
  
  return bNeed;
}

// Is the given MKF way too far away from the nearest other MKF?
bool MapMakerClientBase::IsDistanceToNearestMultiKeyFrameExcessive(MultiKeyFrame &mkf)
{
  MultiKeyFrame *pClosestMKF = ClosestMultiKeyFrame(mkf);
  double dDist = mkf.Distance(*pClosestMKF);
  dDist *= (1.0 / pClosestMKF->mdTotalDepthMean);  // scale by depth
  
  // Arbitrarily choose 3x the scaled mkf dist to be the limit
  return (dDist > MapMakerClientBase::sdMaxScaledMKFDist * 3);
}

// Is the given KF way too far away from the nearest other KF?
bool MapMakerClientBase::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kf)
{
   KeyFrame *pClosestKF = ClosestKeyFrame(kf, KF_ALL);
   
   if(pClosestKF == NULL)
    return true;
   
   double dDist = kf.Distance(*pClosestKF);
   dDist *= (1.0 / pClosestKF->mdSceneDepthMean);  // scale by depth
   
   // Arbitrarily choose 3x the scaled kf dist to be the limit
   return (dDist > MapMakerClientBase::sdMaxScaledKFDist * 3);
}

// Child KeyFrames that don't have the mbActive flag set are deleted
void MapMakerClientBase::ProcessIncomingKeyFrames(MultiKeyFrame &mkf)
{
  for(KeyFramePtrMap::iterator it = mkf.mmpKeyFrames.begin(); it != mkf.mmpKeyFrames.end() ; )
  {
    KeyFrame* pKF = it->second;
    
    if(!pKF->mbActive)
    {
      pKF->EraseBackLinksFromPoints();  // Just a precaution
      delete pKF;
      mkf.mmpKeyFrames.erase(it++);
    }
    else
    {
      pKF->maLevels[0].image.make_unique();  // In case they didn't have deep copies
      it++;
    }
  }
  
  ROS_ASSERT(!mkf.mmpKeyFrames.empty());
}

// Find the closest MKF in the internal queue to the supplied one
MultiKeyFrame* MapMakerClientBase::ClosestMultiKeyFrameInQueue(MultiKeyFrame &mkf)
{
  if(TrackerQueueSize() == 0)
    return NULL;
  
  double dClosestDist = std::numeric_limits<double>::max();
  MultiKeyFrame *pClosestMKF = NULL;
  
  //debug
  std::vector<double> vDists;
  boost::mutex::scoped_lock lock(mQueueMutex);
  
  for(std::deque<MultiKeyFrame*>::iterator it = mqpMultiKeyFramesFromTracker.begin(); it != mqpMultiKeyFramesFromTracker.end(); ++it)
  {
    MultiKeyFrame& currentMKF = *(*it);
    
    if(&currentMKF == &mkf)
      continue;
        
    double dDist = mkf.Distance(currentMKF);
    vDists.push_back(dDist);
   
    if(dDist < dClosestDist)
    {
      dClosestDist = dDist;
      pClosestMKF = &currentMKF;
    }
  }
  
  if(!pClosestMKF)
  {
    ROS_FATAL_STREAM("Got "<<vDists.size()<<" distances: ");
    for(unsigned i=0; i < vDists.size(); ++i)
      ROS_FATAL_STREAM(vDists[i]);
      
    ROS_BREAK();
  }

  return pClosestMKF;
}

void MapMakerClientBase::ClearIncomingQueue()
{
  // Clear out anything in the incoming queue, since there might be a bunch
  // of MKFs piled in there waiting to be added to the map. But since we're
  // done initializing, we don't want to keep these really closely spaced MKFs

  boost::mutex::scoped_lock lock(mQueueMutex);
  while(mqpMultiKeyFramesFromTracker.size() > 0)
  {
    MultiKeyFrame* pMKF = mqpMultiKeyFramesFromTracker.front();
    pMKF->EraseBackLinksFromPoints();
    delete pMKF;
    mqpMultiKeyFramesFromTracker.pop_front();
  }
}

void MapMakerClientBase::LoadCovBundle()
{
  bool bUseRelativePoints = false;
  
  CovHelper& helper = mvCovHelpers[1-mnCurrCov];
  
  if(helper.mpCovBundle != NULL)
  {
    delete helper.mpCovBundle;
  }
  
  helper.mpCovBundle = new ChainBundleIncrementalCovariance(mmCameraModels, true, false, false);
  helper.mmPoint_BundleID.clear();
  helper.mmCamName_BundleID.clear();
  helper.mnTracker_BundleID = -1;
  
  // This one doesn't need to be kept around
  std::map<MultiKeyFrame*, int> mBase_BundleID;
  
  // Start building the bundle adjustment sets
  std::set<MultiKeyFrame*> spAdjustSet;
  std::set<MapPoint*> spMapPoints;
  
  // construct the sets of MKFs and points to be adjusted:
  // in this case, all of them
  ROS_DEBUG_STREAM("There are "<<mMap.mlpMultiKeyFrames.size()<<" MKFs in the map");
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
      continue;
    
    spAdjustSet.insert(&mkf);
  }
  
  ROS_DEBUG_STREAM("Took "<<spAdjustSet.size()<<" of them, the rest are bad");
  /*
  ROS_DEBUG_STREAM("There are "<<mMap.mlpPoints.size()<<" points in the map");
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbBad)
      continue;
      
    if(point.mMMData.GoodMeasCount() < 2 && !point.mbFixed)  // skip if point doesn't have at least two measuring KFs
      continue;
      
    spMapPoints.insert(&point);
  }
  */
  
  int nNumPoses = 0;
  // Add the MultiKeyFrames' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(std::set<MultiKeyFrame*>::iterator mkf_it = spAdjustSet.begin(); mkf_it!= spAdjustSet.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
      continue;
    
    int nBundleID = helper.mpCovBundle->AddPose(mkf.mse3BaseFromWorld, mkf.mbFixed); 
    nNumPoses++;
    mBase_BundleID[&mkf] = nBundleID;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      if(helper.mmCamName_BundleID.count(camName) == 0)   // not yet added
      {
        int nBundleID = helper.mpCovBundle->AddPose(kf.mse3CamFromBase, true); nNumPoses++;
        helper.mmCamName_BundleID[camName] = nBundleID;
      }
    }
  }
  
  int nNumPoints = 0;
  int nWorldID = -1;  // in case we need to add a world pose (if adding fixed points)
  ROS_DEBUG_STREAM("There are "<<mMap.mlpPoints.size()<<" points in the map");
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbBad)
      continue;
      
    if(point.mMMData.GoodMeasCount() < 2 && !point.mbFixed)  // skip if point doesn't have at least two measuring KFs
      continue;
      
  //  spMapPoints.insert(&point);
  //}
  //for(std::set<MapPoint*>::iterator point_it = spMapPoints.begin(); point_it!=spMapPoints.end(); point_it++)
  //{
  //  MapPoint& point = *(*point_it);
    
    //ROS_ASSERT(point.mMMData.GoodMeasCount() >= 2); // checked for this earlier....
    
    TooN::Vector<3> v3Pos;
    std::vector<int> vPoses;
    
    if(point.mbFixed || !bUseRelativePoints)
    {
      if(nWorldID == -1)  // not yet added
        nWorldID = helper.mpCovBundle->AddPose(TooN::SE3<>(), true);
        
      v3Pos = point.mv3WorldPos;
      vPoses.push_back(nWorldID);
    }
    else
    {
      v3Pos = point.mpPatchSourceKF->mse3CamFromWorld * point.mv3WorldPos;
      vPoses.push_back(mBase_BundleID[point.mpPatchSourceKF->mpParent]);
      vPoses.push_back(helper.mmCamName_BundleID[point.mpPatchSourceKF->mCamName]);
    }
    
    int nBundleID = helper.mpCovBundle->AddPoint(v3Pos, vPoses, point.mbFixed);  
    nNumPoints++;
    helper.mmPoint_BundleID[&point] = nBundleID;
  }
  
  int nNumMeas = 0;
  // Add the relevant point-in-keyframe measurements
  for(std::map<MultiKeyFrame*, int>::iterator mkf_it = mBase_BundleID.begin(); mkf_it != mBase_BundleID.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(mkf_it->first);
    int nMKF_BundleID = mkf_it->second;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      std::string camName = kf_it->first;
      
      ROS_ASSERT(helper.mmCamName_BundleID.count(camName));
      int nCamName_BundleID = helper.mmCamName_BundleID[camName];
      
      std::vector<int> vCams(2);
      vCams[0] = nMKF_BundleID;
      vCams[1] = nCamName_BundleID;
      
      for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
  
        if(helper.mmPoint_BundleID.count(&point) == 0)
          continue;
          
        Measurement& meas = *(meas_it->second);
        
        int nPoint_BundleID = helper.mmPoint_BundleID[&point];
        
        helper.mpCovBundle->AddMeas(vCams, nPoint_BundleID, meas.v2RootPos, LevelScale(meas.nLevel) * LevelScale(meas.nLevel), camName);
        nNumMeas++;
      }
    }
  }
  
  ROS_DEBUG_STREAM("Ended up adding "<<nNumPoses<<" poses, "<<nNumPoints<<" points, "<<nNumMeas<<" measurements");
  
  bool bAbortRequested = false;
  helper.mpCovBundle->Compute(&bAbortRequested, 0, 0);
  
  if(helper.mpCovBundle->CovValid())
  {
    // Update the points' cov matrix
    for(std::map<MapPoint*,int>::iterator point_it = helper.mmPoint_BundleID.begin(); point_it != helper.mmPoint_BundleID.end(); ++point_it)
    {
      MapPoint& point = *(point_it->first);
      point.mm3WorldCov = helper.mpCovBundle->GetPointCov(point_it->second);
    }
    
    // Now add the tracker pose (incrementally)
    //helper.mnTracker_BundleID = helper.mpCovBundle->AddPoseIncrementally(TooN::SE3<>()); 
    
    // Swap current helper
    boost::mutex::scoped_lock lock(mCovMutex);
    mnCurrCov = 1-mnCurrCov;
  }
  else
  {
    ROS_WARN_STREAM("Computing covariance in CovBundle failed, not swapping with current!");
  }
}

/*
bool CompCrossCov(PointCrossCov* pXCov1, PointCrossCov* pXCov2) 
{ 
  return pXCov1->mdPriority > pXCov2->mdPriority;
}
*/

bool CompCrossCov(const std::pair<double, PointCrossCov*>& cov1, const std::pair<double, PointCrossCov*> cov2)
{
  return cov1.first > cov2.first;
}

/*
void MapMakerClientBase::UpdateCrossCovariances2(std::unordered_set<MapPoint*> spParticipatingPoints, ros::Duration allowedDur, double dPriorityThresh)
{
  boost::mutex::scoped_lock lock(mCovMutex);
  CovHelper& helper = mvCovHelpers[mnCurrCov];
  
  if(helper.mpCovBundle == NULL)
  {
    std::cerr<<"No CovBundle found in helper!"<<std::endl;
    return;
  }
    
  std::vector<std::pair<double, PointCrossCov*> > vCrossCov;
  vCrossCov.reserve(spParticipatingPoints.size() * spParticipatingPoints.size());
  
  int nNumSkipped = 0;
  
  std::cerr<<"About to build vCrossCov"<<std::endl;
  ros::Time startBuild = ros::Time::now();
  
  for(std::unordered_set<MapPoint*>::iterator point1_it = spParticipatingPoints.begin(); point1_it != spParticipatingPoints.end(); ++point1_it)
  {
    MapPoint* pPoint1 = *point1_it;
    
    for(std::unordered_set<MapPoint*>::iterator point2_it = point1_it; point2_it != spParticipatingPoints.end(); ++point2_it)
    {
      if(point2_it == point1_it)
        continue;
        
      MapPoint* pPoint2 = *point2_it;
      
      CrossCovPtrMap::iterator x_it1 = pPoint1->mmpCrossCov.find(pPoint2);
      if(x_it1 != pPoint1->mmpCrossCov.end())
      {
        PointCrossCov* pCrossCov = x_it1->second;
        ROS_ASSERT(pCrossCov->mdPriority >= 0);
        
        // Don't bother adding anything with priority zero since it doesn't need to be updated
        if(pCrossCov->mdPriority > dPriorityThresh)
        {
          vCrossCov.push_back(std::make_pair(pCrossCov->mdPriority, pCrossCov));
        }
        else
        {
          ++nNumSkipped;
        }
      }
      else
      {
        PointCrossCov* pCrossCov = new PointCrossCov(*pPoint1, *pPoint2);
        
        pPoint1->AddCrossCov(pPoint2, pCrossCov);
        pPoint2->AddCrossCov(pPoint1, pCrossCov);
        
        vCrossCov.push_back(std::make_pair(pCrossCov->mdPriority, pCrossCov));
      }
    }
  }
  
  std::cerr<<">>>>>>>>>>>>>>>>>>>> Build vCrossCov in "<<ros::Time::now() - startBuild<<" seconds"<<std::endl;
  
  std::cerr<<"About to shuffle "<<vCrossCov.size()<<" cross covariance objects"<<std::endl;
  ros::WallTime shuffleStart = ros::WallTime::now();
  std::random_shuffle(vCrossCov.begin(), vCrossCov.end());
  std::cerr<<"Done shuffling, took "<<ros::WallTime::now() - shuffleStart <<" seconds"<<std::endl;
  
  ros::Duration crossCovDuration(0);
  ros::Time startTime = ros::Time::now();
  
  int nNumProcessed = 0;
  for(unsigned i=0; i < vCrossCov.size(); ++i, ++nNumProcessed)
  {
    if(ros::Time::now() - startTime > allowedDur)
      break;
    
    PointCrossCov* pCrossCov = vCrossCov[i].second;
    
    if(helper.mmPoint_BundleID.count(&pCrossCov->mPoint1) == 0)
    {
      std::cerr<<"Participating point 1 not found in cov bundle!"<<std::endl;
      continue;
    }
      
    if(helper.mmPoint_BundleID.count(&pCrossCov->mPoint2) == 0)
    {
      std::cerr<<"Participating point 2 not found in cov bundle!"<<std::endl;
      continue;
    }
    
    int nPoint1_BundleID = helper.mmPoint_BundleID[&pCrossCov->mPoint1];
    int nPoint2_BundleID = helper.mmPoint_BundleID[&pCrossCov->mPoint2]; 
  
    TooN::Matrix<3> m3Cov = TooN::Zeros;
    ros::Time crossCovStartTime = ros::Time::now();
    bool bSuccess = helper.mpCovBundle->GetPointsCrossCov(nPoint1_BundleID, nPoint2_BundleID, m3Cov);
    crossCovDuration += (ros::Time::now() - crossCovStartTime);
    
    if(bSuccess)
    {
      pCrossCov->SetCrossCov(m3Cov);
    }
  }
  
  std::cerr<<"Processed "<<nNumProcessed<<" cross cov in alotted time of "<<allowedDur<<" seconds"<<std::endl;
  std::cerr<<"Actually spent "<<crossCovDuration<<" on GetPointsCrossCov()"<<std::endl;
  std::cerr<<"Skipped "<<nNumSkipped<<" cross cov whose priority was too small"<<std::endl;
  std::cerr<<"There are "<<nNumSkipped+nNumProcessed<<" cross cov ready out of a total of "<<vCrossCov.size()+nNumSkipped<<std::endl;
}

*/

void MapMakerClientBase::SavePointCovMatrix(std::string fileName)
{
  boost::mutex::scoped_lock lock(mCovMutex);
  CovHelper& helper = mvCovHelpers[mnCurrCov];
  
  if(helper.mpCovBundle == NULL)
  {
    ROS_ERROR("MapMakerClientBase::SavePointCovMatrix: No CovBundle found in helper!");
    return;
  }
  
  helper.mpCovBundle->SavePointCovMatrix(fileName);
}

void MapMakerClientBase::SavePointCovMatrix(std::string fileName, std::unordered_set<MapPoint*> spParticipatingPoints)
{
  boost::mutex::scoped_lock lock(mCovMutex);
  CovHelper& helper = mvCovHelpers[mnCurrCov];
  
  if(helper.mpCovBundle == NULL)
  {
    std::cerr<<"No CovBundle found in helper!"<<std::endl;
    return;
  }
  
  std::vector<int> vPointIDs;
  
  // Remove any points from the set of participating points that aren't found in the helper's cov bundle
  for(std::unordered_set<MapPoint*>::iterator point_it = spParticipatingPoints.begin(); point_it != spParticipatingPoints.end(); ++point_it)
  {
    MapPoint* pPoint = *point_it;
    std::map<MapPoint*, int>::iterator found_it = helper.mmPoint_BundleID.find(pPoint);
    ROS_ASSERT(found_it != helper.mmPoint_BundleID.end());
    
    int nID = found_it->second;
    pPoint->mnID = nID;
    
    vPointIDs.push_back(nID);
  }
  
  std::sort(vPointIDs.begin(), vPointIDs.end());
  
  helper.mpCovBundle->SavePointCovMatrix(fileName, vPointIDs);
}

void verifyCrossCovExistance(std::unordered_set<MapPoint*> spParticipatingPoints)
{
  for(std::unordered_set<MapPoint*>::iterator point1_it = spParticipatingPoints.begin(); point1_it != spParticipatingPoints.end(); ++point1_it)
  {
    MapPoint* pPoint1 = *point1_it;
    for(std::unordered_set<MapPoint*>::iterator point2_it = spParticipatingPoints.begin(); point2_it != point1_it; ++point2_it)
    {
      if(point2_it == point1_it)
        continue;
      
      MapPoint* pPoint2 = *point2_it;
      if(pPoint1->mmpCrossCov.count(pPoint2) == 0)
      {
        std::cerr<<">>> No cross cov found!"<<std::endl;
        std::cerr<<"pPoint2: "<<pPoint2<<std::endl;
        ROS_BREAK();
      }
    }
  }
  
  std::cerr<<">>> All cross cov found"<<std::endl;
}


void MapMakerClientBase::UpdateCrossCovariances(std::unordered_set<MapPoint*>& spParticipatingPoints, ros::Duration allowedDur, double dPriorityThresh)
{
  boost::mutex::scoped_lock lock(mCovMutex);
  CovHelper& helper = mvCovHelpers[mnCurrCov];
  
  if(helper.mpCovBundle == NULL)
  {
    std::cerr<<"No CovBundle found in helper!"<<std::endl;
    spParticipatingPoints.clear();
    return;
  }
  
  std::unordered_set<MapPoint*> spTempPoints;
  
  // Remove any points from the set of participating points that aren't found in the helper's cov bundle
  for(std::unordered_set<MapPoint*>::iterator point_it = spParticipatingPoints.begin(); point_it != spParticipatingPoints.end(); ++point_it)
  {
    if(helper.mmPoint_BundleID.count(*point_it))
    {
      spTempPoints.insert(*point_it);
    }
  }
  
  spParticipatingPoints.swap(spTempPoints);
  
  std::unordered_set<MapPoint*> spGoodPoints;
  std::unordered_set<MapPoint*> spBadPoints;
  
  bool bCheckDur = allowedDur > ros::Duration(0);
  
  ros::Duration crossCovDuration(0);
  ros::Time startTime = ros::Time::now();
  
  for(std::unordered_set<MapPoint*>::iterator point1_it = spParticipatingPoints.begin(); point1_it != spParticipatingPoints.end(); ++point1_it)
  {    
    MapPoint* pPoint1 = *point1_it;
    bool bOutOfTime = false;
    
    for(std::unordered_set<MapPoint*>::iterator point2_it = spParticipatingPoints.begin(); point2_it != point1_it; ++point2_it)
    {
      if(bCheckDur && ros::Time::now() - startTime > allowedDur)
      {
        bOutOfTime = true;
        break;
      }
      
      MapPoint* pPoint2 = *point2_it;
      
      CrossCovPtrMap::iterator x_it1 = pPoint1->mmpCrossCov.find(pPoint2);
      PointCrossCov* pCrossCov;
      if(x_it1 != pPoint1->mmpCrossCov.end())
      {
        pCrossCov = x_it1->second;
        ROS_ASSERT(pCrossCov->mdPriority >= 0);
        ROS_ASSERT(pPoint2->mmpCrossCov.count(pPoint1));
        
        // Don't bother adding anything with priority zero since it doesn't need to be updated
        if(pCrossCov->mdPriority < dPriorityThresh)
          continue;
      }
      else
      {
        pCrossCov = new PointCrossCov(*pPoint1, *pPoint2);
        
        pPoint1->AddCrossCov(pPoint2, pCrossCov);
        pPoint2->AddCrossCov(pPoint1, pCrossCov);
      }
      
      int nPoint1_BundleID = helper.mmPoint_BundleID[pPoint1];
      int nPoint2_BundleID = helper.mmPoint_BundleID[pPoint2]; 
    
      TooN::Matrix<3> m3Cov = TooN::Zeros;
      ros::Time crossCovStartTime = ros::Time::now();
      bool bSuccess = helper.mpCovBundle->GetPointsCrossCov(nPoint1_BundleID, nPoint2_BundleID, m3Cov);
      crossCovDuration += (ros::Time::now() - crossCovStartTime);
      
      if(bSuccess)
      {
        pCrossCov->SetCrossCov(pPoint1, m3Cov);
      }
      else
      {
        spBadPoints.insert(pPoint1);
        spBadPoints.insert(pPoint2);
      }
      
      // Test if point2 in point1
      if(pPoint1->mmpCrossCov.count(pPoint2) == 0)
      {
        std::cerr<<"=== No cross cov found!"<<std::endl;
        std::cerr<<"pPoint2: "<<pPoint2<<std::endl;
        ROS_BREAK();
      }
    }
    
    // Check if overtime
    if(bOutOfTime)
      break;
    
    spGoodPoints.insert(pPoint1);
  }
  
  spTempPoints.clear();
  
  for(std::unordered_set<MapPoint*>::iterator point_it = spGoodPoints.begin(); point_it != spGoodPoints.end(); ++point_it)
  {
    if(spBadPoints.count(*point_it))
      continue;
      
    spTempPoints.insert(*point_it);
  }
  
  std::cerr<<"Started with "<<spParticipatingPoints.size()<<" points, out of these "<<spGoodPoints.size()<<" were identified as good, but out of the good points, "<<spBadPoints.size()<<" were actually bad"<<std::endl;
  std::cerr<<"This leaves "<<spTempPoints.size()<<" points that we're keeping"<<std::endl;
  
  verifyCrossCovExistance(spTempPoints);
  
  spParticipatingPoints.swap(spTempPoints);
  // Now, spParticipatingPoints only contains points for which the cross covariance has
  // been updated for all points in the set
}

void MapMakerClientBase::ComputeSelectedPointsCrossCov()
{
  boost::mutex::scoped_lock lock(mCovMutex);
  CovHelper& helper = mvCovHelpers[mnCurrCov];
  
  mdSelectedPointsCrossCovNorm = -1;
  
  if(helper.mpCovBundle == NULL)
  {
    std::cerr<<"No CovBundle found in helper!"<<std::endl;
    return;
  }
    
  if(mvpSelectedPoints[0] == NULL)
  {
    //std::cerr<<"Selected point 1 is null!"<<std::endl;
    return;
  }
    
  if(mvpSelectedPoints[1] == NULL)
  {
    //std::cerr<<"Selected point 2 is null!"<<std::endl;
    return;
  }
    
  if(helper.mmPoint_BundleID.count(mvpSelectedPoints[0]) == 0)
  {
    std::cerr<<"Selected point 1 not found in cov bundle!"<<std::endl;
    return;
  }
    
  if(helper.mmPoint_BundleID.count(mvpSelectedPoints[1]) == 0)
  {
    std::cerr<<"Selected point 2 not found in cov bundle!"<<std::endl;
    return;
  }
        
  int nPoint1_BundleID = helper.mmPoint_BundleID[mvpSelectedPoints[0]];
  int nPoint2_BundleID = helper.mmPoint_BundleID[mvpSelectedPoints[1]]; 
  
  TooN::Matrix<3> m3Cov = TooN::Zeros;
  bool bSuccess = helper.mpCovBundle->GetPointsCrossCov(nPoint1_BundleID, nPoint2_BundleID, m3Cov);
  std::cout<<"==============================================================================="<<std::endl;
  std::cout<<"Cross Cov success: "<<(bSuccess ? "yes" : "no")<<std::endl<<m3Cov<<std::endl;
  std::cout<<"==============================================================================="<<std::endl;
  
  mdSelectedPointsCrossCovNorm = TooN::norm_fro(m3Cov);
}

TooN::Matrix<6> MapMakerClientBase::GetTrackerCov(TooN::SE3<> se3BaseFromWorld, std::vector<std::string>& vCamNames, std::vector<TrackerDataPtrVector>& vIterationSets)
{
  boost::mutex::scoped_lock lock(mCovMutex);
  CovHelper& helper = mvCovHelpers[mnCurrCov];
  
  TooN::Matrix<6> m6Cov = TooN::Zeros;
  
  if(helper.mpCovBundle == NULL)
    return m6Cov;
  
  helper.mpCovBundle->SetPose(helper.mnTracker_BundleID, se3BaseFromWorld);
  
  // Add the tracker measurements (incrementally)
  for(unsigned i=0; i < vCamNames.size(); ++i)
  {
    std::string camName = vCamNames[i];
    ROS_ASSERT(helper.mmCamName_BundleID.count(camName));
    
    std::vector<int> vCams(2);
    vCams[0] = helper.mnTracker_BundleID;
    vCams[1] = helper.mmCamName_BundleID[camName];
    
    for(TrackerDataPtrVector::iterator td_it = vIterationSets[i].begin(); td_it!= vIterationSets[i].end(); ++td_it)
    {
      TrackerData& td = *(*td_it);
      
      if(!td.mbFound || td.mPoint.mbBad)
        continue;
        
      MapPoint& point = td.mPoint;
      
      if(helper.mmPoint_BundleID.count(&point) == 0)
        continue;
        
      int nPoint_BundleID = helper.mmPoint_BundleID[&point];
      
      helper.mpCovBundle->AddMeasIncrementally(vCams, nPoint_BundleID, td.mv2Found, LevelScale(td.mnSearchLevel) * LevelScale(td.mnSearchLevel), camName);
    }
  }
  
  ros::Time start = ros::Time::now();
  m6Cov = helper.mpCovBundle->GetPoseCovIncrementally(helper.mnTracker_BundleID);
  ros::Duration elapsed = ros::Time::now() - start;
  
  std::cout<<"======================================================"<<std::endl;
  std::cout<<"================ EXPERIMENTAL COV ===================="<<std::endl;
  std::cout<<"======================================================"<<std::endl;
  std::cout<<m6Cov<<std::endl;
  std::cout<<"Time taken: "<<elapsed<<" seconds"<<std::endl;
  std::cout<<"======================================================"<<std::endl;
  std::cout<<"======================================================"<<std::endl;
  std::cout<<"======================================================"<<std::endl;
  
  return m6Cov;
}

TooN::Matrix<6> MapMakerClientBase::GetTrackerCovFull(TooN::SE3<> se3BaseFromWorld, std::vector<std::string>& vCamNames, std::vector<TrackerDataPtrVector>& vIterationSets)
{
  bool bUseRelativePoints = false;
  
  // Start building the bundle adjustment sets
  std::set<MultiKeyFrame*> spAdjustSet;
  std::set<MapPoint*> spMapPoints;
  
  // construct the sets of MKFs and points to be adjusted:
  // in this case, all of them
  ROS_DEBUG_STREAM("There are "<<mMap.mlpMultiKeyFrames.size()<<" MKFs in the map");
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
      continue;
    
    spAdjustSet.insert(&mkf);
  }
  
  ROS_DEBUG_STREAM("Took "<<spAdjustSet.size()<<" of them, the rest are bad");
  
  ROS_DEBUG_STREAM("There are "<<mMap.mlpPoints.size()<<" points in the map");
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbBad)
      continue;
      
    if(point.mMMData.GoodMeasCount() < 2 && !point.mbFixed)  // skip if point doesn't have at least two measuring KFs
      continue;
      
    spMapPoints.insert(&point);
  }
  
  // The bundle adjuster does different accounting of MultiKeyFrames and MapPoints;
  // Translation maps are stored:
  std::map<MapPoint*, int> mPoint_BundleID;      ///< %Map FROM MapPoint* TO point id
  std::map<int, MapPoint*> mBundleID_Point;      ///< %Map FROM point id TO MapPoint*
  std::map<MultiKeyFrame*, int> mBase_BundleID;  ///< %Map FROM MultiKeyFrame* TO mkf id
  std::map<int, MultiKeyFrame*> mBundleID_Base;  ///< %Map FROM mkf id TO MultiKeyFrame*
  std::map<std::string, int> mCamName_BundleID;  ///< %Map FROM camera name TO camera id
  std::map<int, std::string> mBundleID_CamName;  ///< %Map FROM camera id TO camera name
  
  
  ROS_DEBUG_STREAM("BundleAdjustTrackerPose received: "<<spAdjustSet.size()<<" movable MKFs, "<<spMapPoints.size()<<" points");
    
  ChainBundleIncrementalCovariance multiBundle(mmCameraModels, true, false, false);
  
  int nNumPoses = 0;
  // Add the MultiKeyFrames' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(std::set<MultiKeyFrame*>::iterator mkf_it = spAdjustSet.begin(); mkf_it!= spAdjustSet.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
      continue;
    
    int nBundleID = multiBundle.AddPose(mkf.mse3BaseFromWorld, mkf.mbFixed); 
    nNumPoses++;
    mBase_BundleID[&mkf] = nBundleID;
    mBundleID_Base[nBundleID] = &mkf;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      if(mCamName_BundleID.count(camName) == 0)   // not yet added
      {
        int nBundleID = multiBundle.AddPose(kf.mse3CamFromBase, true); nNumPoses++;
        mCamName_BundleID[camName] = nBundleID;
        mBundleID_CamName[nBundleID] = camName;
      }
    }
  }
  
  int nNumPoints = 0;
  int nWorldID = -1;  // in case we need to add a world pose (if adding fixed points)
  for(std::set<MapPoint*>::iterator point_it = spMapPoints.begin(); point_it!=spMapPoints.end(); point_it++)
  {
    MapPoint& point = *(*point_it);
    
    //ROS_ASSERT(point.mMMData.GoodMeasCount() >= 2); // checked for this earlier....
    
    TooN::Vector<3> v3Pos;
    std::vector<int> vPoses;
    
    if(point.mbFixed || !bUseRelativePoints)
    {
      if(nWorldID == -1)  // not yet added
        nWorldID = multiBundle.AddPose(TooN::SE3<>(), true);
        
      v3Pos = point.mv3WorldPos;
      vPoses.push_back(nWorldID);
    }
    else
    {
      v3Pos = point.mpPatchSourceKF->mse3CamFromWorld * point.mv3WorldPos;
      vPoses.push_back(mBase_BundleID[point.mpPatchSourceKF->mpParent]);
      vPoses.push_back(mCamName_BundleID[point.mpPatchSourceKF->mCamName]);
    }
    
    int nBundleID = multiBundle.AddPoint(v3Pos, vPoses, point.mbFixed);  
    nNumPoints++;
    mPoint_BundleID[&point] = nBundleID;
    mBundleID_Point[nBundleID] = &point;
  }
  
  int nNumMeas = 0;
  // Add the relevant point-in-keyframe measurements
  for(std::map<MultiKeyFrame*, int>::iterator mkf_it = mBase_BundleID.begin(); mkf_it != mBase_BundleID.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(mkf_it->first);
    int nMKF_BundleID = mkf_it->second;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      std::string camName = kf_it->first;
      
      ROS_ASSERT(mCamName_BundleID.count(camName));
      int nCamName_BundleID = mCamName_BundleID[camName];
      
      std::vector<int> vCams(2);
      vCams[0] = nMKF_BundleID;
      vCams[1] = nCamName_BundleID;
      
      for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
  
        if(mPoint_BundleID.count(&point) == 0)
          continue;
          
        Measurement& meas = *(meas_it->second);
        
        int nPoint_BundleID = mPoint_BundleID[&point];
        
        multiBundle.AddMeas(vCams, nPoint_BundleID, meas.v2RootPos, LevelScale(meas.nLevel) * LevelScale(meas.nLevel), camName);
        nNumMeas++;
      }
    }
  }
  
  // Include tracker pose now
  int nTracker_BundleID = multiBundle.AddPose(se3BaseFromWorld, false); 
  nNumPoses++;
  
  // Add the tracker measurements
  for(unsigned i=0; i < vCamNames.size(); ++i)
  {
    std::string camName = vCamNames[i];
    ROS_ASSERT(mCamName_BundleID.count(camName));
    
    std::vector<int> vCams(2);
    vCams[0] = nTracker_BundleID;
    vCams[1] = mCamName_BundleID[camName];
    
    for(TrackerDataPtrVector::iterator td_it = vIterationSets[i].begin(); td_it!= vIterationSets[i].end(); ++td_it)
    {
      TrackerData& td = *(*td_it);
      
      if(!td.mbFound || td.mPoint.mbBad)
        continue;
        
      MapPoint& point = td.mPoint;
      
      if(mPoint_BundleID.count(&point) == 0)
        continue;
        
      int nPoint_BundleID = mPoint_BundleID[&point];
      
      multiBundle.AddMeas(vCams, nPoint_BundleID, td.mv2Found, LevelScale(td.mnSearchLevel) * LevelScale(td.mnSearchLevel), camName);
      nNumMeas++;
    }
  }

  ROS_DEBUG_STREAM("Ended up adding "<<nNumPoses<<" poses, "<<nNumPoints<<" points, "<<nNumMeas<<" measurements");
  
  bool bAbortRequested = false;
  int nAccepted = multiBundle.Compute(&bAbortRequested, 0, 0);
  
  return multiBundle.GetPoseCov(nTracker_BundleID);
}

