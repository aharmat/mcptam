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
// Much of the code in this class is from the MapMaker class of PTAM, which is
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <algorithm>

// Static members
int BundleAdjusterBase::snRecentMinSize = 8;
int BundleAdjusterBase::snRecentNum = 3;
int BundleAdjusterBase::snMinMapPoints = 10;

BundleAdjusterBase::BundleAdjusterBase(Map &map, TaylorCameraMap& cameras)
: mMap(map)
, mmCameraModels(cameras)
{  
  Reset();
}

void BundleAdjusterBase::Reset()
{
  mbBundleRunning = false;
  mbBundleRunningIsRecent = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbBundleAbortRequested = false;
  
  mdMaxCov = std::numeric_limits<double>::max();
  mbUseTukey = false;
  mbUseTwoStep = false;
  mbUseRobust = true;
}

// Get a certain number of neighbors of a given MultiKeyFrame
std::vector<MultiKeyFrame*> BundleAdjusterBase::NClosestMultiKeyFrames(MultiKeyFrame &mkfSrc, unsigned int N)
{
  std::vector<std::pair<double, MultiKeyFrame* > > vScoresAndMKFs;
  for(MultiKeyFramePtrList::iterator it = mMap.mlpMultiKeyFrames.begin(); it != mMap.mlpMultiKeyFrames.end(); ++it)
  {
    MultiKeyFrame& mkf = *(*it);
    
    if(&mkf == &mkfSrc)
      continue;
      
    double dDist = mkfSrc.Distance(mkf);
    vScoresAndMKFs.push_back(std::make_pair(dDist, &mkf));
  }
  
  if(N > vScoresAndMKFs.size())   // if we expected too many neighbors
    N = vScoresAndMKFs.size();    // reduce number that will be returned
    
  // Sort the first N entries by score
  std::partial_sort(vScoresAndMKFs.begin(), vScoresAndMKFs.begin() + N, vScoresAndMKFs.end());
  
  std::vector<MultiKeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vScoresAndMKFs[i].second);
    
  return vResult;
}

// Get a certain number of neighbors of a given KeyFrame
std::vector<KeyFrame*> BundleAdjusterBase::NClosestKeyFrames(KeyFrame &kfSrc, unsigned int N)
{
  std::vector<std::pair<double, KeyFrame* > > vScoresAndKFs;
  for(MultiKeyFramePtrList::iterator it = mMap.mlpMultiKeyFrames.begin(); it != mMap.mlpMultiKeyFrames.end(); ++it)
  {
    MultiKeyFrame& mkf = *(*it);
    
    // Have to iterate through each MultiKeyFrame as well to get KeyFrames
    for(KeyFramePtrMap::iterator jit = mkf.mmpKeyFrames.begin(); jit != mkf.mmpKeyFrames.end(); jit++)
    {
      KeyFrame& kf = *(jit->second);
      
      if(&kf == &kfSrc)
        continue;
      
      double dDist = kfSrc.Distance(kf);
      vScoresAndKFs.push_back(std::make_pair(dDist, &kf));
    }
  }
    
  if(N > vScoresAndKFs.size()) // if we expected too many neighbors
    N = vScoresAndKFs.size();  // reduce number that will be returned
    
  // Sort the first N entries by score
  std::partial_sort(vScoresAndKFs.begin(), vScoresAndKFs.begin() + N, vScoresAndKFs.end());
  
  std::vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vScoresAndKFs[i].second);
    
  return vResult;
}

// Adjust all the MapPoints and MultiKeyFrames in the Map
// All the arguments of this function are output variables
int BundleAdjusterBase::BundleAdjustAll(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers)
{
  vOutliers.clear();
  
  std::set<MultiKeyFrame*> spAdjustSet;
  std::set<MultiKeyFrame*> spFixedSet; 
  std::set<MapPoint*> spMapPoints;
  
  ros::WallTime start = ros::WallTime::now();
  
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
  ROS_DEBUG_STREAM("Took "<<spMapPoints.size()<<" of them, the rest are bad or have only one observing KF");
  
  ROS_DEBUG_STREAM("Took "<<ros::WallTime::now() - start<<" seconds to collect MKFs and map points");
  
  // Call implementation of BundleAdjust
  return BundleAdjust(spAdjustSet, spFixedSet, spMapPoints, vOutliers, false);
}

// Adjusts the most recent MultiKeyFrame and a small number of its neighbors, as well as the MapPoints seen from them
// All the arguments of this function are output variables
int BundleAdjusterBase::BundleAdjustRecent(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers)
{
  vOutliers.clear();
  
  std::set<MultiKeyFrame*> spAdjustSet;
  std::set<MultiKeyFrame*> spFixedSet; 
  std::set<MapPoint*> spMapPoints;
  
  if((int)mMap.mlpMultiKeyFrames.size() < BundleAdjusterBase::snRecentMinSize)   // Only continue if map is big enough
  { 
    ROS_INFO("BundleAdjusterBase: Map not big enough for local BA, returning");
    mbBundleConverged_Recent = true;
    return 0;
  }

  // First, make a list of the MultiKeyFrames we want adjusted in the adjuster.
  // This will be the last MultiKeyFrame inserted, and its nearest neighbors
  MultiKeyFrame& mkf_newest = *(mMap.mlpMultiKeyFrames.back());
  spAdjustSet.insert(&mkf_newest);
  std::vector<MultiKeyFrame*> vpClosest = NClosestMultiKeyFrames(mkf_newest, BundleAdjusterBase::snRecentNum);
  for(int i=0; i < BundleAdjusterBase::snRecentNum; i++)
  {
    if(vpClosest[i]->mbBad)
      continue;
    
    if(vpClosest[i]->mbFixed == false)  // only keep movable ones
      spAdjustSet.insert(vpClosest[i]);
  }
  
  // Now we find the set of features which they contain.
  for(std::set<MultiKeyFrame*>::iterator iter = spAdjustSet.begin(); iter!=spAdjustSet.end(); iter++)
  {
    MultiKeyFrame& mkf = *(*iter);
    for(KeyFramePtrMap::iterator it = mkf.mmpKeyFrames.begin(); it != mkf.mmpKeyFrames.end(); ++it)
    {
      MeasPtrMap& meas_map = it->second->mmpMeasurements;
      for(MeasPtrMap::iterator jiter = meas_map.begin(); jiter!= meas_map.end(); ++jiter)
      {
        MapPoint& point = *(jiter->first);
        
        if(point.mbBad)
          continue;
          
        if(point.mMMData.GoodMeasCount() < 2)  // skip if point doesn't have at least two measuring KFs
          continue;
          
        spMapPoints.insert(&point);
        
      }
    }
  }
  
  // Find which KeyFrames make measurements of the points that will be adjusted
  std::set<KeyFrame*> spAffectedKeyFrames;
  for(std::set<MapPoint*>::iterator iter = spMapPoints.begin(); iter!=spMapPoints.end(); iter++)
  {
    MapPoint& point = *(*iter);
    spAffectedKeyFrames.insert(point.mMMData.spMeasurementKFs.begin(), point.mMMData.spMeasurementKFs.end());
  }
  
  // Finally, add all MultiKeyFrames which own any of the affected KeyFrames found above as fixed MultiKeyFrames
  for(std::set<KeyFrame*>::iterator it = spAffectedKeyFrames.begin(); it != spAffectedKeyFrames.end(); ++it)
  {
    KeyFrame& kf = *(*it);
    MultiKeyFrame& mkf = *kf.mpParent;
    
    if(mkf.mbBad)
      continue;
    
    if(spAdjustSet.count(&mkf) == false)  // parent MKF is not already being adjusted
      spFixedSet.insert(&mkf);            // then add to fixed set
  }
      
  // Call implementation of BundleAdjust
  return BundleAdjust(spAdjustSet, spFixedSet, spMapPoints, vOutliers, true);
}
