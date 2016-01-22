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

#include <mcptam/BundleAdjusterCalib.h>
#include <mcptam/ChainBundle.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>

using namespace TooN;

BundleAdjusterCalib::BundleAdjusterCalib(Map& map, TaylorCameraMap& cameras, bool bApplyUpdates)
  : BundleAdjusterBase(map, cameras), mbApplyUpdates(bApplyUpdates)
{
}

// Extracts a relative pose map from a set of MKFs. The assumption is that all KFs' mse3CamFromBase
// pose has been properly set so we just have to find the first MKF that has all cameras, and extract
// its' KFs' mse3CamFromBase poses
SE3Map BundleAdjusterCalib::ExtractRelativePoses(std::set<MultiKeyFrame*> spMKFs)
{
  SE3Map mPoses;

  for (std::set<MultiKeyFrame*>::iterator mkf_it = spMKFs.begin(); mkf_it != spMKFs.end(); mkf_it++)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    if (mkf.mmpKeyFrames.size() == mmCameraModels.size())  // found a multikeyframe with all cameras
    {
      for (KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); kf_it++)
      {
        KeyFrame& kf = *(kf_it->second);
        std::string camName = kf_it->first;

        mPoses[camName] = kf.mse3CamFromBase;
      }
      break;
    }
  }

  // Check to make sure that the first pose is identity
  TooN::SE3<> se3FirstTransform = (mPoses.begin())->second;
  TooN::Vector<6> v6FirstTransform = se3FirstTransform.ln();
  ROS_ASSERT(v6FirstTransform * v6FirstTransform < 1e-10);

  return mPoses;
}

// Loads the given MultiKeyFrames and MapPoints into the underlying ChainBundle, runs computation, extracts results and
// outliers
int BundleAdjusterCalib::BundleAdjust(std::set<MultiKeyFrame*> spAdjustSet, std::set<MultiKeyFrame*> spFixedSet,
                                      std::set<MapPoint*> spMapPoints,
                                      std::vector<std::pair<KeyFrame*, MapPoint*>>& vOutliers, bool bRecent)
{
  if (!spFixedSet.empty())  // Shouldn't have any MKFs forced to be fixed when using BundleAdjusterCalib
  {
    ROS_FATAL(
      "BundleAdjusterCalib: Got a non-empty fixed set, you should not call BundleAdjustRecent(...) as the whole map "
      "is needed for calibration, call BundleAdjustAll(...) instead!");
    ROS_BREAK();
  }

  // The bundle adjuster does different accounting of keyFrames and map points;
  // Translation maps are stored:
  mmPoint_BundleID.clear();
  mmBundleID_Point.clear();
  mmBase_BundleID.clear();
  mmBundleID_Base.clear();
  mmCamName_BundleID.clear();
  mmBundleID_CamName.clear();

  if ((int)spMapPoints.size() < BundleAdjusterBase::snMinMapPoints)
  {
    ROS_INFO_STREAM("BundleAdjusterCalib: Not enough good map points, got " << spMapPoints.size() << ", need "
                    << BundleAdjusterBase::snMinMapPoints);
    ROS_INFO("BundleAdjusterCalib: Returning 0");
    return 0;
  }

  ChainBundle calibBundle(mmCameraModels, mbUseRobust, mbUseTukey, false);
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;

  SE3Map mRelativePoses = ExtractRelativePoses(spAdjustSet);

  // Add the relative cameras first
  // Skip the first pose since it's identity
  for (SE3Map::iterator se3_it = ++mRelativePoses.begin(); se3_it != mRelativePoses.end(); ++se3_it)
  {
    std::string camName = se3_it->first;
    SE3<>& se3Pose = se3_it->second;

    ROS_DEBUG_STREAM("From ExtractRelativePoses, " << camName << std::endl
                     << se3Pose);

    int nBundleID = calibBundle.AddPose(se3Pose, false);

    mmCamName_BundleID[camName] = nBundleID;
    mmBundleID_CamName[nBundleID] = camName;
  }

  // Now we can add the absolute cameras, ie the poses of the first KeyFrame
  for (std::set<MultiKeyFrame*>::iterator mkf_it = spAdjustSet.begin(); mkf_it != spAdjustSet.end(); mkf_it++)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    KeyFrame& firstKeyFrame = *(mkf.mmpKeyFrames.begin()->second);

    int nBundleID = calibBundle.AddPose(firstKeyFrame.mse3CamFromWorld, mkf.mbFixed);

    mmBase_BundleID[&mkf] = nBundleID;
    mmBundleID_Base[nBundleID] = &mkf;
  }

  // Add the points' 3D position
  int nWorldID = -1;  // in case we need to add a world pose (if adding fixed points)
  for (std::set<MapPoint*>::iterator point_it = spMapPoints.begin(); point_it != spMapPoints.end(); point_it++)
  {
    MapPoint& point = *(*point_it);

    if (point.mMMData.GoodMeasCount() < 2 && !point.mbFixed)  // allow one measurement only if point is fixed
      continue;

    TooN::Vector<3> v3Pos;
    std::vector<int> vPoses;

    if (point.mbFixed)
    {
      if (nWorldID == -1)  // not yet added
      {
        nWorldID = calibBundle.AddPose(TooN::SE3<>(), true);
        ROS_DEBUG_STREAM("Added world frame at ID: " << nWorldID);
      }

      v3Pos = point.mv3WorldPos;
      vPoses.push_back(nWorldID);
    }
    else
    {
      v3Pos = point.mpPatchSourceKF->mse3CamFromWorld * point.mv3WorldPos;
      vPoses.push_back(mmBase_BundleID[point.mpPatchSourceKF->mpParent]);

      if (mmCamName_BundleID.count(point.mpPatchSourceKF->mCamName))  // this is a relative camera, camera chain is 2
        // poses long
        vPoses.push_back(mmCamName_BundleID[point.mpPatchSourceKF->mCamName]);
    }

    int nBundleID = calibBundle.AddPoint(v3Pos, vPoses, point.mbFixed);
    mmPoint_BundleID[&point] = nBundleID;
    mmBundleID_Point[nBundleID] = &point;
  }

  // Add the relevant point-in-keyframe measurements
  for (std::map<MultiKeyFrame*, int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it != mmBase_BundleID.end();
       ++mkf_it)
  {
    MultiKeyFrame& mkf = *(mkf_it->first);
    int nMKF_BundleID = mkf_it->second;

    for (KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      std::string camName = kf_it->first;

      // Create vector of camera indices that form the chain
      std::vector<int> vCams;
      vCams.push_back(nMKF_BundleID);  // add base camera first

      // if the current KeyFrame's pose is considered a relative pose
      if (mmCamName_BundleID.count(kf.mCamName))
        vCams.push_back(mmCamName_BundleID[kf.mCamName]);  // add that relative pose to the chain

      for (MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); meas_it++)
      {
        MapPoint& point = *(meas_it->first);

        // If we're not dealing with this MapPoint, skip (shouldn't happen in usual case cause we're using them all)
        if (mmPoint_BundleID.count(&point) == 0)
          continue;

        Measurement& meas = *(meas_it->second);
        int nPoint_BundleID = mmPoint_BundleID[&point];

        calibBundle.AddMeas(vCams, nPoint_BundleID, meas.v2RootPos, LevelScale(meas.nLevel) * LevelScale(meas.nLevel),
                            camName);
      }
    }
  }

  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = calibBundle.Compute(&mbBundleAbortRequested, 10);

  if (nAccepted < 0)
  {
    // Crap: - LM Ran into a serious problem!
    // Return negative value, MapMaker should probably scrap the map and start over
    ROS_ERROR("BundleAdjusterCalib: Cholesky failure in bundle adjust. The map is probably corrupt: Should ditch the "
              "map.");
    return -1;
  }

  // Bundle adjustment did some updates, apply these to the map
  if (nAccepted > 0 && mbApplyUpdates)
  {
    for (std::map<MultiKeyFrame*, int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it != mmBase_BundleID.end();
         ++mkf_it)
    {
      MultiKeyFrame& mkf = *(mkf_it->first);
      mkf.mse3BaseFromWorld = calibBundle.GetPose(mkf_it->second);

      KeyFrame& firstKeyFrame = *(mkf.mmpKeyFrames.begin()->second);
      firstKeyFrame.mse3CamFromBase = SE3<>();
      firstKeyFrame.mse3CamFromWorld = mkf.mse3BaseFromWorld;

      // Start at next keyframe
      for (KeyFramePtrMap::iterator kf_it = ++mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
      {
        KeyFrame& kf = *(kf_it->second);
        std::string camName = kf_it->first;

        kf.mse3CamFromBase = calibBundle.GetPose(mmCamName_BundleID[camName]);
        kf.mse3CamFromWorld = kf.mse3CamFromBase * mkf.mse3BaseFromWorld;
      }
    }

    // Update map point world positions
    for (std::map<MapPoint*, int>::iterator point_it = mmPoint_BundleID.begin(); point_it != mmPoint_BundleID.end();
         ++point_it)
    {
      MapPoint& point = *(point_it->first);
      TooN::Vector<3> v3Pos = calibBundle.GetPoint(point_it->second);

      if (point.mbFixed)
        point.mv3WorldPos = v3Pos;
      else
        point.mv3WorldPos = point.mpPatchSourceKF->mse3CamFromWorld.inverse() * v3Pos;

      point.mbOptimized = true;
      point.RefreshPixelVectors();
    }
    // After point positions updated, refresh scene depth
    for (std::map<MultiKeyFrame*, int>::iterator mkf_it = mmBase_BundleID.begin(); mkf_it != mmBase_BundleID.end();
         ++mkf_it)
    {
      MultiKeyFrame& mkf = *(mkf_it->first);
      mkf.RefreshSceneDepthRobust();
    }

    if (bRecent)
      mbBundleConverged_Recent = false;

    mbBundleConverged_Full = false;
  }

  if (calibBundle.Converged())
  {
    mbBundleConverged_Recent = true;
    if (!bRecent)
      mbBundleConverged_Full = true;
  }

  // Handle outlier measurements:
  std::vector<std::tuple<int, int, std::string>> vOutliersEncoded = calibBundle.GetOutlierMeasurements();
  for (unsigned int i = 0; i < vOutliersEncoded.size(); i++)
  {
    MapPoint* pPoint = mmBundleID_Point[std::get<0>(vOutliersEncoded[i])];
    MultiKeyFrame* pMKF = mmBundleID_Base[std::get<1>(vOutliersEncoded[i])];
    std::string camName = std::get<2>(vOutliersEncoded[i]);

    KeyFrame* pKF = pMKF->mmpKeyFrames[camName];

    vOutliers.push_back(std::make_pair(pKF, pPoint));
  }

  mmFinalPoses = ExtractRelativePoses(spAdjustSet);
  mdSigmaSquared = calibBundle.GetSigmaSquared();
  mdMeanChiSquared = calibBundle.GetMeanChiSquared();
  mdMaxCov = calibBundle.GetMaxCov();

  mbBundleRunning = false;
  mbBundleAbortRequested = false;

  return nAccepted;
}
