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

#include <mcptam/BundleAdjusterMulti.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>

BundleAdjusterMulti::BundleAdjusterMulti(Map &map,  TaylorCameraMap& cameras, bool bApplyUpdates, bool bVerbose, bool bUseRelativePoints)
: BundleAdjusterBase(map, cameras)
, mbUseRelativePoints(bUseRelativePoints)
, mbApplyUpdates(bApplyUpdates)
, mbVerbose(bVerbose)
{
  std::cout<<"USE RELATIVE POINTS: "<<mbUseRelativePoints<<std::endl;
}

// Loads the given MultiKeyFrames and MapPoints into the underlying ChainBundleG2O, runs computation, extracts results and outliers
int BundleAdjusterMulti::BundleAdjust(std::set<MultiKeyFrame*> spAdjustSet, std::set<MultiKeyFrame*> spFixedSet, std::set<MapPoint*> spMapPoints, std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers, bool bRecent)
{
  // The bundle adjuster does different accounting of MultiKeyFrames and MapPoints;
  // Translation maps are stored:
  mmPoint_BundleID.clear();
  mmBundleID_Point.clear();
  mmBase_BundleID.clear();
  mmBundleID_Base.clear();
  mmCamName_BundleID.clear();
  mmBundleID_CamName.clear();
  
  if((int)spMapPoints.size() < BundleAdjusterBase::snMinMapPoints)
  {
    ROS_INFO_STREAM("BundleAdjusterMulti: Not enough good map points, got "<<spMapPoints.size()<<", need "<<BundleAdjusterBase::snMinMapPoints);
    ROS_INFO("BundleAdjusterMulti: Returning 0");
    return 0;
  }
  
  ROS_DEBUG_STREAM("BundleAdjusterMulti received: "<<spAdjustSet.size()<<" movable MKFs, "<<spFixedSet.size()<<" fixed MKFs, "<<spMapPoints.size()<<" points");
  
  ChainBundle multiBundle(mmCameraModels, mbUseRobust, mbUseTukey, mbUseMarginalized, mbVerbose);
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;
  
  ros::WallTime start = ros::WallTime::now();
  
  int nNumPoses = 0;
  // Add the MultiKeyFrames' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(std::set<MultiKeyFrame*>::iterator mkf_it = spAdjustSet.begin(); mkf_it!= spAdjustSet.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
      continue;
    
    int nBundleID = multiBundle.AddPose(mkf.mse3BaseFromWorld, mkf.mbFixed); 
    nNumPoses++;
    mmBase_BundleID[&mkf] = nBundleID;
    mmBundleID_Base[nBundleID] = &mkf;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      if(mmCamName_BundleID.count(camName) == 0)   // not yet added
      {
        int nBundleID = multiBundle.AddPose(kf.mse3CamFromBase, true); nNumPoses++;
        mmCamName_BundleID[camName] = nBundleID;
        mmBundleID_CamName[nBundleID] = camName;
      }
    }
  }
  
  for(std::set<MultiKeyFrame*>::iterator mkf_it = spFixedSet.begin(); mkf_it!= spFixedSet.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    if(mkf.mbBad)
      continue;
    
    // Force these to be fixed, even if the mbFixed flag is false, because we might be adjusting only a local neighborhood with fixed outer observers
    int nBundleID = multiBundle.AddPose(mkf.mse3BaseFromWorld, true);   nNumPoses++;
    mmBase_BundleID[&mkf] = nBundleID;
    mmBundleID_Base[nBundleID] = &mkf;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      std::string camName = kf_it->first;
      KeyFrame& kf = *(kf_it->second);
      if(mmCamName_BundleID.count(camName) == 0)   // not yet added
      {
        int nBundleID = multiBundle.AddPose(kf.mse3CamFromBase, true); 
        nNumPoses++;
        mmCamName_BundleID[camName] = nBundleID;
        mmBundleID_CamName[nBundleID] = camName;
      }
    }
  }
  
  int nNumPoints = 0;
  int nWorldID = -1;  // in case we need to add a world pose (if adding fixed points)
  for(std::set<MapPoint*>::iterator point_it = spMapPoints.begin(); point_it!=spMapPoints.end(); point_it++)
  {
    MapPoint& point = *(*point_it);
    
    //ROS_ASSERT(point.mMMData.GoodMeasCount() >= 2); // checked for this in BundleAdjusterBase....
    
    TooN::Vector<3> v3Pos;
    std::vector<int> vPoses;
    
    if(point.mbFixed || !mbUseRelativePoints)
    {
      if(nWorldID == -1)  // not yet added
        nWorldID = multiBundle.AddPose(TooN::SE3<>(), true);
        
      v3Pos = point.mv3WorldPos;
      vPoses.push_back(nWorldID);
    }
    else
    {
      v3Pos = point.mpPatchSourceKF->mse3CamFromWorld * point.mv3WorldPos;
      vPoses.push_back(mmBase_BundleID[point.mpPatchSourceKF->mpParent]);
      vPoses.push_back(mmCamName_BundleID[point.mpPatchSourceKF->mCamName]);
    }
    
    int nBundleID = multiBundle.AddPoint(v3Pos, vPoses, point.mbFixed);  
    nNumPoints++;
    mmPoint_BundleID[&point] = nBundleID;
    mmBundleID_Point[nBundleID] = &point;
  }
  
  int nNumMeas = 0;
  // Add the relevant point-in-keyframe measurements
  for(std::map<MultiKeyFrame*, int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it != mmBase_BundleID.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(mkf_it->first);
    int nMKF_BundleID = mkf_it->second;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      std::string camName = kf_it->first;
      
      ROS_ASSERT(mmCamName_BundleID.count(camName));
      int nCamName_BundleID = mmCamName_BundleID[camName];
      
      std::vector<int> vCams(2);
      vCams[0] = nMKF_BundleID;
      vCams[1] = nCamName_BundleID;
      
      for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
  
        if(mmPoint_BundleID.count(&point) == 0)
          continue;
          
        Measurement& meas = *(meas_it->second);
        
        int nPoint_BundleID = mmPoint_BundleID[&point];
        
        multiBundle.AddMeas(vCams, nPoint_BundleID, meas.v2RootPos, LevelScale(meas.nLevel) * LevelScale(meas.nLevel), camName);
        nNumMeas++;
      }
    }
  }
  
  ROS_DEBUG_STREAM("Took "<<ros::WallTime::now() - start<<" seconds to add data to adjuster");
  ROS_DEBUG_STREAM("Ended up adding "<<nNumPoses<<" poses, "<<nNumPoints<<" points, "<<nNumMeas<<" measurements");
  
  // Do only a couple of iterations to the get map approximately right, this way new points can be used
  // by the tracker sooner than if we waited for a lengthy adjustment
  int nAccepted = 0;
  
  if(mbUseTwoStep)
  {
    nAccepted = AdjustAndUpdate(multiBundle, spAdjustSet, spMapPoints, 10);
  
    if(nAccepted < 0)
      return nAccepted;
    
    if(!multiBundle.Converged())  // Do more iterations
      nAccepted = AdjustAndUpdate(multiBundle, spAdjustSet, spMapPoints);
  }
  else
  {
    // Run the bundle adjuster. This returns the number of successful iterations
    nAccepted = AdjustAndUpdate(multiBundle, spAdjustSet, spMapPoints);
  }
  
  if(nAccepted < 0)
    return nAccepted;
  
  if(multiBundle.Converged())
  {
    ROS_INFO("BundleAdjusterMulti: BA Converged!");
    
    mbBundleConverged_Recent = true;
    if(!mbBundleRunningIsRecent)
      mbBundleConverged_Full = true;
  }
  else
    ROS_INFO("BundleAdjusterMulti: BA Not Converged");
  
  // Handle outlier measurements:
  std::vector<std::tuple<int,int,std::string> > vOutliersEncoded = multiBundle.GetOutlierMeasurements();
  ROS_INFO_STREAM("BundleAdjusterMulti: Got "<<vOutliersEncoded.size()<<" outliers");
  
  for(unsigned int i=0; i<vOutliersEncoded.size(); i++)
  {
    MapPoint *pPoint = mmBundleID_Point[std::get<0>(vOutliersEncoded[i])];
    MultiKeyFrame *pMKF = mmBundleID_Base[std::get<1>(vOutliersEncoded[i])];
    std::string camName = std::get<2>(vOutliersEncoded[i]);
    
    KeyFrame *pKF = pMKF->mmpKeyFrames[camName];
    
    vOutliers.push_back(std::make_pair(pKF,pPoint));
  }
  
  mbBundleRunning = false;
  mbBundleAbortRequested = false;
  
  return nAccepted;
}

// Run the Bundle Adjustment and update the map with the result
int BundleAdjusterMulti::AdjustAndUpdate(ChainBundle& multiBundle, std::set<MultiKeyFrame*> spAdjustSet, std::set<MapPoint*> spMapPoints, int nIterations)
{
  int nAccepted = 0;
  // Run the bundle adjuster. This returns the number of successful iterations
  if(nIterations <= 0)
    nAccepted = multiBundle.Compute(&mbBundleAbortRequested);
  else
    nAccepted = multiBundle.Compute(&mbBundleAbortRequested, nIterations);
  
  if(nAccepted < 0)
  {
    // Crap: - LM Ran into a serious problem!
    // This is probably because the initial map was messed up.
    // Return negative value, MapMaker should probably scrap the map and start over 
    ROS_ERROR("BundleAdjusterMulti: Failure in bundle adjust. The map is probably corrupt. Should ditch the map.");
    return -1;
  }
  
  // Bundle adjustment did some updates, apply these to the map
  if(nAccepted > 0 && mbApplyUpdates)
  {
    if(mbBundleRunningIsRecent)
      mbBundleConverged_Recent = false;
      
    mbBundleConverged_Full = false;
    
    boost::mutex::scoped_lock lock(mMap.mMutex);
    
    // First update the MKF BaseFromWorld poses, and refresh the KF CamFromWorld poses
    for(std::map<MultiKeyFrame*,int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it!=mmBase_BundleID.end(); ++mkf_it)
    {
      MultiKeyFrame& mkf = *(mkf_it->first);
      
      mkf.mse3BaseFromWorld = multiBundle.GetPose(mkf_it->second);
      for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
      {
        KeyFrame& kf = *(kf_it->second);
        kf.mse3CamFromWorld = kf.mse3CamFromBase * mkf.mse3BaseFromWorld ; // Update keyframe mse3CamFromWorld only here!! CHECK!! GOOD
      }
    }
    
    // Update the point world positions
    for(std::map<MapPoint*,int>::iterator point_it = mmPoint_BundleID.begin(); point_it!=mmPoint_BundleID.end(); ++point_it)
    {
      MapPoint& point = *(point_it->first);
      
      if(!point.mbFixed)
      {
        TooN::Vector<3> v3Pos = multiBundle.GetPoint(point_it->second);
        
        if(!mbUseRelativePoints)
        {
          point.mv3WorldPos = v3Pos;
          
          if(multiBundle.GetMaxCov() < std::numeric_limits<double>::max())
          {
            point.mm3WorldCov = multiBundle.GetPointCov(point_it->second);
          }
        }
        else
        {
          point.mv3WorldPos = point.mpPatchSourceKF->mse3CamFromWorld.inverse() * v3Pos;
        }
      }
        
      point.RefreshPixelVectors();
      point.mbOptimized = true;
    }
    
    // After point positions updated, refresh scene depth
    for(std::map<MultiKeyFrame*,int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it!=mmBase_BundleID.end(); ++mkf_it)
    {
      MultiKeyFrame& mkf = *(mkf_it->first);
      mkf.RefreshSceneDepthRobust();
    }
    
    mdSigmaSquared = multiBundle.GetSigmaSquared();
    mdMeanChiSquared = multiBundle.GetMeanChiSquared();
    mdMaxCov = multiBundle.GetMaxCov();
    
    if(!mUpdateCallback.empty())
      mUpdateCallback(spAdjustSet, spMapPoints);
  }

  return nAccepted;
}

