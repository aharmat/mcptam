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

#include <mcptam/BundleAdjusterSingle.h>
#include <mcptam/ChainBundle.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>

BundleAdjusterSingle::BundleAdjusterSingle(Map &map,  TaylorCameraMap& cameras, bool bApplyUpdates)
: BundleAdjusterBase(map, cameras)
, mbApplyUpdates(bApplyUpdates)
{
  
}

// Loads the given MultiKeyFrames and MapPoints into the underlying ChainBundle, runs computation, extracts results and outliers
int BundleAdjusterSingle::BundleAdjust(std::set<MultiKeyFrame*> spAdjustSet, std::set<MultiKeyFrame*> spFixedSet, std::set<MapPoint*> spMapPoints, std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers, bool bRecent)
{ 
  // The bundle adjuster does different accounting of keyFrames and map points;
  // Translation maps are stored:
  mmPoint_BundleID.clear();
  mmBundleID_Point.clear();
  mmKF_BundleID.clear();
  mmBundleID_KF.clear();
  
  if((int)spMapPoints.size() < BundleAdjusterBase::snMinMapPoints)
  {
    ROS_INFO_STREAM("BundleAdjusterSingle: Not enough good map points, got "<<spMapPoints.size()<<", need "<<BundleAdjusterBase::snMinMapPoints);
    ROS_INFO("BundleAdjusterSingle: Returning 0");
    return 0;
  }
  
  ChainBundle singleBundle(mmCameraModels, mbUseRobust, mbUseTukey, false);
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;
  
  // Add the keyFrames' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(std::set<MultiKeyFrame*>::iterator mkf_it = spAdjustSet.begin(); mkf_it!= spAdjustSet.end(); mkf_it++)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame &kf = *(kf_it->second);
      
      int nBundleID = singleBundle.AddPose(kf.mse3CamFromWorld, mkf.mbFixed);
      mmKF_BundleID[&kf] = nBundleID;
      mmBundleID_KF[nBundleID] = &kf;
    }
  }
  for(std::set<MultiKeyFrame*>::iterator mkf_it = spFixedSet.begin(); mkf_it!= spFixedSet.end(); mkf_it++)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame &kf = *(kf_it->second);
      
      // Force these to be fixed, even if the mbFixed flag is false, because we might be adjusting only a local neighborhood with fixed outer observers
      int nBundleID = singleBundle.AddPose(kf.mse3CamFromWorld, true);
      mmKF_BundleID[&kf] = nBundleID;
      mmBundleID_KF[nBundleID] = &kf;
    }
  }
  
  int nWorldID = -1;  // in case we need to add a world pose (if adding fixed points)
  for(std::set<MapPoint*>::iterator point_it = spMapPoints.begin(); point_it!=spMapPoints.end(); point_it++)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mMMData.GoodMeasCount() < 2 && !point.mbFixed)
      continue;
    
    TooN::Vector<3> v3Pos;
    std::vector<int> vPoses(1);
    
    if(point.mbFixed)
    {
      if(nWorldID == -1)  // not yet added
        nWorldID = singleBundle.AddPose(TooN::SE3<>(), true);
        
      v3Pos = point.mv3WorldPos;
      vPoses[0] = nWorldID;
    }
    else
    {
      v3Pos = point.mpPatchSourceKF->mse3CamFromWorld * point.mv3WorldPos;
      vPoses[0] = mmKF_BundleID[point.mpPatchSourceKF];
    }
    
    int nBundleID = singleBundle.AddPoint(v3Pos, vPoses, point.mbFixed);
    mmPoint_BundleID[&point] = nBundleID;
    mmBundleID_Point[nBundleID] = &point;
  }
  
  
  // Add the relevant point-in-keyframe measurements
  for(std::map<KeyFrame*, int>::iterator kf_it = mmKF_BundleID.begin(); kf_it != mmKF_BundleID.end(); kf_it++)
  {
    KeyFrame& kf = *(kf_it->first);
    int nKF_BundleID = kf_it->second;
    
    for(MeasPtrMap::iterator jiter = kf.mmpMeasurements.begin(); jiter != kf.mmpMeasurements.end(); jiter++)
    {
      MapPoint& point = *(jiter->first);

      if(mmPoint_BundleID.count(&point) == 0)
        continue;
        
      Measurement& meas = *(jiter->second);
      
      std::vector<int> vCams(1);
      vCams[0] = nKF_BundleID;
      int nPoint_BundleID = mmPoint_BundleID[&point];
      singleBundle.AddMeas(vCams, nPoint_BundleID, meas.v2RootPos, LevelScale(meas.nLevel) * LevelScale(meas.nLevel), kf.mCamName);
    }
  }
  
  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = singleBundle.Compute(&mbBundleAbortRequested);
  
  if(nAccepted < 0)
  {
    // Crap: - LM Ran into a serious problem!
    // This is probably because the initial stereo was messed up.
    // Return negative value, MapMaker should probably scrap the map and start over 
    ROS_ERROR("BundleAdjusterSingle: Cholesky failure in bundle adjust. The map is probably corrupt: Should ditch the map.");
    return -1;
  }
  
  // Bundle adjustment did some updates, apply these to the map
  if(nAccepted > 0 && mbApplyUpdates)
  {
    if(bRecent)
      mbBundleConverged_Recent = false;
      
    mbBundleConverged_Full = false;
    
    // First update the KF's CamFromWorld
    for(std::map<KeyFrame*,int>::iterator kf_it = mmKF_BundleID.begin(); kf_it!=mmKF_BundleID.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->first);
      kf.mse3CamFromWorld = singleBundle.GetPose(kf_it->second);
      
      if(&kf == kf.mpParent->mmpKeyFrames.begin()->second) // If first KeyFrame of parent
        kf.mpParent->mse3BaseFromWorld = kf.mse3CamFromWorld; // update base transform
    }
    
    // After CamFromWorld and BaseFromWorld is updated, we can now update CamFromBase
    for(std::map<KeyFrame*,int>::iterator kf_it = mmKF_BundleID.begin(); kf_it!=mmKF_BundleID.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->first);
      kf.mse3CamFromBase = kf.mse3CamFromWorld * kf.mpParent->mse3BaseFromWorld.inverse();
    }
    
    // Update the point world positions
    for(std::map<MapPoint*,int>::iterator point_it = mmPoint_BundleID.begin(); point_it!=mmPoint_BundleID.end(); ++point_it)
    {
      MapPoint& point = *(point_it->first);
      TooN::Vector<3> v3Pos = singleBundle.GetPoint(point_it->second);
      
      if(point.mbFixed)
        point.mv3WorldPos = v3Pos;
      else
        point.mv3WorldPos = point.mpPatchSourceKF->mse3CamFromWorld.inverse() * v3Pos;
        
      point.mbOptimized = true;
      point.RefreshPixelVectors();
    }
    
    // After point positions updated, refresh scene depth
    for(std::map<MultiKeyFrame*,int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it!=mmBase_BundleID.end(); ++mkf_it)
    {
      MultiKeyFrame& mkf = *(mkf_it->first);
      mkf.RefreshSceneDepthRobust();
    }
  }
  
  if(singleBundle.Converged())
  {
    mbBundleConverged_Recent = true;
    if(!bRecent)
      mbBundleConverged_Full = true;
  }
  
  // Handle outlier measurements:
  std::vector<std::tuple<int,int,std::string> > vOutliersEncoded = singleBundle.GetOutlierMeasurements();
  for(unsigned int i=0; i<vOutliersEncoded.size(); i++)
  {
    MapPoint *pPoint = mmBundleID_Point[std::get<0>(vOutliersEncoded[i])];
    KeyFrame *pKF = mmBundleID_KF[std::get<1>(vOutliersEncoded[i])];
  
    vOutliers.push_back(std::make_pair(pKF,pPoint));
  }
  
  mdSigmaSquared = singleBundle.GetSigmaSquared();
  mdMeanChiSquared = singleBundle.GetMeanChiSquared();
  mdMaxCov = singleBundle.GetMaxCov();
  
  mbBundleRunning = false;
  mbBundleAbortRequested = false;
  
  return nAccepted;
  
}

