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
#include <gvars3/instances.h>

int MapMakerClientBase::snMinOutliers = 20;
double MapMakerClientBase::sdOutlierMultiplier = 1.0;
double MapMakerClientBase::sdMaxScaledMKFDist = 0.1; //0.3;
double MapMakerClientBase::sdMaxScaledKFDist = 0.1; //0.3;

MapMakerClientBase::MapMakerClientBase(Map& map, RelocaliserFabMap &reloc)
  : MapMakerBase(map, true)  // This will be skipped since inheritance is virtual!
  , mRelocFabMap(reloc)
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
  
  mRelocFabMap.Reset();
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
  
  if(nMarkedBad > 0)
  {
    ROS_INFO_STREAM("======== Handle outlier based on tracker marked "<<nMarkedBad<<" points as bad");
  }
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
  
  ROS_INFO_STREAM("NeedNewMultiKeyFrame: dist: "<<dDist<<" mean depth: "<<mkf.mdTotalDepthMean<<" thresh: "<< dThresh << " need: "<<(bNeed ? "yes" : "no"));
  
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
  for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end() ; )
  {
    KeyFrame* pKF = kf_it->second;
    
    if(!pKF->mbActive)
    {
      pKF->EraseBackLinksFromPoints();  // Just a precaution
      delete pKF;
      mkf.mmpKeyFrames.erase(kf_it++);
    }
    else
    {
      pKF->maLevels[0].image.make_unique();  // In case they didn't have deep copies
      kf_it++;
    }
  }
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

