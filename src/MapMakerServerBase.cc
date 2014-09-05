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

#include <mcptam/MapMakerServerBase.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/Map.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/PatchFinder.h>
#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/Reset.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>
#include <gvars3/instances.h>

using namespace TooN;
using namespace GVars3;

// Static members
int MapMakerServerBase::snMinMapPoints = 20;
int MapMakerServerBase::snMaxConsecutiveFailedBA = 5;
int MapMakerServerBase::snMaxTriangulationKFs = 5;
int MapMakerServerBase::snMaxInitPointsLevelZero = 100;
double MapMakerServerBase::sdInitDepth = 3.0;
std::string MapMakerServerBase::ssInitPointMode = "both";  // options: "stereo", "idp", "both"
double MapMakerServerBase::sdInitCovThresh = 1.0;
bool MapMakerServerBase::sbLargePointTest = true;

MapMakerServerBase::MapMakerServerBase(Map& map, TaylorCameraMap &cameras, BundleAdjusterBase& bundleAdjuster)
  : MapMakerBase(map, cameras, true)  // This will be skipped since inheritance is virtual!
  , mBundleAdjuster(bundleAdjuster)
{
  // This needs to be remapped in the launch file
  mResetSystemClient = mNodeHandle.serviceClient<mcptam::Reset>("reset", true);
  
  ROS_INFO_STREAM("MapMakerServerBase: Reset system client is connecting to: "<<mResetSystemClient.getService());
  
  while(!mResetSystemClient.waitForExistence(ros::Duration(5)) && ros::ok())
  {
		ROS_WARN_STREAM("MapMakerServerBase: Waiting for system reset service to be advertised...");
	}
  
  Reset();
};

MapMakerServerBase::~MapMakerServerBase()
{
  mBundleAdjuster.RequestAbort();
}

void MapMakerServerBase::RequestResetInternal()
{ 
  mcptam::Reset reset_srv;
  
  // The system is online at this point, so resetting to zero would 
  // probably be very undesirable. Ask System to save the tracker's
  // pose, reinitialize the map, and restore the tracker's pose so that
  // everything can continue (relatively) seamlessly
  reset_srv.request.bReInit = true;
  reset_srv.request.bSavePose = true;
  
  ROS_INFO("MapMakerServerBase: Calling reset on system via client");
  while(!mResetSystemClient.call(reset_srv) && ros::ok())
  {
    ROS_INFO("MapMakerServerBase: Trying again to call reset on system...");
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("MapMakerServerBase: Successfully called reset on client");
}

void MapMakerServerBase::Reset()
{
  // The class deriving from this one should call Reset on this class and MapMakerBase
  
  ROS_DEBUG("MapMakerServerBase: Reset");
  
  mlFailureQueue.clear();
  mlpNewQueue.clear();
  
  mBundleAdjuster.Reset();
  mnNumConsecutiveFailedBA = 0;
}

// Finds the 3D position of a point (in reference frame B) given the inputs
Vector<3> MapMakerServerBase::ReprojectPoint(SE3<> se3AfromB, const Vector<3> &v3A, const Vector<3> &v3B)
{
  // Algorithm from Hartley & Zisserman's Multiple View Geometry book section 12.2
  
  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();
  
  Matrix<4> A;
  A[0][0] = -v3B[2]; A[0][1] =  0.0;     A[0][2] = v3B[0];  A[0][3] = 0.0;
  A[1][0] =  0.0;    A[1][1] = -v3B[2];  A[1][2] = v3B[1];  A[1][3] = 0.0;
  A[2] = v3A[0] * PDash[2] - v3A[2] * PDash[0];
  A[3] = v3A[1] * PDash[2] - v3A[2] * PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
    
  return project(v4Smallest);
}

// Build an initial map using epipolar matches between KeyFrames of a MultiKeyFrame
bool MapMakerServerBase::InitFromMultiKeyFrame(MultiKeyFrame* pMKF, bool bPutPlaneAtOrigin)
{
  ROS_INFO("MapMakerServerBase::InitFromMultiKeyFrame");
  
  pMKF->mbFixed = true;
  pMKF->mse3BaseFromWorld = SE3<>();
  
  for(KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);
    
    kf.mse3CamFromWorld = kf.mse3CamFromBase * pMKF->mse3BaseFromWorld;
    kf.mdSceneDepthMean = 100;     
    kf.mdSceneDepthSigma = 100000;  // With a large sigma, the epipolar matches will search along the whole line
    
    kf.MakeKeyFrame_Rest();
  } 

  mMap.mlpMultiKeyFrames.push_back(pMKF);
  
  int nNumCams = pMKF->mmpKeyFrames.size();
  int nNumPointsBefore;
  int nLevelLimit;
  int nLevelPoints;
  int nLevelPointsLeft;
  static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN|SILENT);
  
  ROS_INFO_STREAM("==== STARTING INIT POINT CREATION, MODE: "<<MapMakerServerBase::ssInitPointMode<<"  =========");
  
  // Add points to the map using only the KeyFrames belonging to the last added MultiKeyFrame
  // In this case could set region to KF_ALL since there are no other MultiKeyFrames to begin with
  for(int l = 3; l >= 0; --l)
  {
    if(l == 0 && *gvnLevelZeroPoints == false)
      continue;
    
    nLevelLimit = MapMakerServerBase::snMaxInitPointsLevelZero/LevelScale(l);
    nNumPointsBefore = mMap.mlpPoints.size();
    
    if(MapMakerServerBase::ssInitPointMode == "stereo" || MapMakerServerBase::ssInitPointMode == "both")
      AddStereoMapPoints(*pMKF, l, nLevelLimit, std::numeric_limits<double>::max(), KF_ONLY_SELF);
       
    nLevelPoints = mMap.mlpPoints.size() - nNumPointsBefore;
    nLevelPointsLeft = (nLevelLimit*nNumCams - nLevelPoints)/nNumCams;
    ROS_INFO_STREAM("Level "<<l<<" made "<<nLevelPoints<<" with stereo, "<<nLevelPointsLeft<<" points left to make with each cam");
    
    if(nLevelPointsLeft < 1)
    {
      ROS_INFO_STREAM("Not enough level points left, continuing");
      continue;
    }
    
    nNumPointsBefore = mMap.mlpPoints.size();
    if(MapMakerServerBase::ssInitPointMode == "idp" || MapMakerServerBase::ssInitPointMode == "both")
      AddInitDepthMapPoints(*pMKF, l, nLevelPointsLeft, MapMakerServerBase::sdInitDepth);
      
    ROS_INFO_STREAM("Made "<<mMap.mlpPoints.size() - nNumPointsBefore<<" points with Init Depth");
    
  }
  
  if((int)mMap.mlpPoints.size() < MapMakerServerBase::snMinMapPoints)
  {
    ROS_ERROR_STREAM("MapMakerServerBase: Epipolar matching and init depth method didn't make enough map points: "<<mMap.mlpPoints.size()<<"  Minimum is: "<<MapMakerServerBase::snMinMapPoints);
    return false;
  }
    
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    (*point_it)->mbOptimized = true; // want tracker to track these points, have to set them to optimized
  }
  
  mBundleAdjuster.SetNotConverged();
  
  pMKF->RefreshSceneDepthRobust();
  
  
  /*
  if(bPutPlaneAtOrigin)  // Rotate and translate the map so the dominant plane is at z=0:
  {
    ApplyGlobalTransformationToMap(CalcPlaneAligner());
  }
  */ 

  
  /*
  std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliers;
  mBundleAdjuster.UseTukey(false);
  mBundleAdjuster.UseTwoStep(false);
      
  int nAccepted = mBundleAdjuster.BundleAdjustAll(vOutliers);
  mdMaxCov = mBundleAdjuster.GetMaxCov();
   
  if(nAccepted < 0) // bad
    return false;
    
  if(mdMaxCov < MapMakerServerBase::sdInitCovThresh) 
  {
    ROS_INFO_STREAM("INITIALIZING, Max cov "<<mdMaxCov<<" below threshold "<<MapMakerServerBase::sdInitCovThresh<<", switching to RUNNING");
    mState = MM_RUNNING;
  }
  */
  
  ROS_INFO_STREAM("MapMakerServerBase: Made initial map with " << mMap.mlpPoints.size());
  ROS_INFO_STREAM("MapMakerServerBase: Initial MKF pose: "<<std::endl<<pMKF->mse3BaseFromWorld);
  ROS_INFO_STREAM("MapMakerServerBase: Initial MKF scene depth: "<<pMKF->mdTotalDepthMean);
  
  for(KeyFramePtrMap::iterator kf_it = pMKF->mmpKeyFrames.begin(); kf_it != pMKF->mmpKeyFrames.end(); ++kf_it)
  {
    std::string camName = kf_it->first;
    ROS_INFO_STREAM("MapMakerServerBase: Initial "<<camName<<" scene depth: "<<kf_it->second->mdSceneDepthMean<<" sigma: "<<kf_it->second->mdSceneDepthSigma);
  }
  
  mMap.mbGood = true;
  
  return true; 
}

// Mark the MKF which is furthest from the given one as bad and erase it
void MapMakerServerBase::MarkFurthestMultiKeyFrameAsBad(MultiKeyFrame& mkf)
{
  MultiKeyFrame* pFurthestMKF = FurthestMultiKeyFrame(mkf);
  pFurthestMKF->mbBad = true;
  
  for(KeyFramePtrMap::iterator kf_it = pFurthestMKF->mmpKeyFrames.begin(); kf_it != pFurthestMKF->mmpKeyFrames.end(); ++kf_it)
  {
    KeyFrame& kf = *(kf_it->second);
    for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it!=kf.mmpMeasurements.end(); ++meas_it)
    {
      MapPoint& point = *(meas_it->first);
      //Measurement& meas = *(meas_it->second);
      
      // If point only has 2 or fewer measurements then it won't be constrained anymore when we remove
      // this kf, so remove point too (but only if it's not a fixed point). Also mark point bad if 
      // its patch source is the current KF, since it'll lose its creator.
      
      if((point.mMMData.spMeasurementKFs.size() <= 2 && !point.mbFixed)
          || point.mpPatchSourceKF == &kf)
        point.mbBad = true;
        /*
      else
      {
        // Need to transfer the patch source to another KF, choose randomly (ie first in set)
        KeyFrame& kfNewSource = *(*point.mMMData.spMeasurementKFs.begin());
        MeasPtrMap::iterator meas_it = kfNewSource.mmpMeasurements.find(&point);
         
        ROS_ASSERT(meas_it != kfNewSource.mmpMeasurements.end());
        Measurement& measNewSource = *(meas_it->second);
        
        point.mpPatchSourceKF = &kfNewSource;
        point.mnSourceLevel = measNewSource.nLevel;
        point.mirCenter = CVD::ir( LevelNPos(measNewSource.v2RootPos, measNewSource.nLevel) );
        point.mv3Center_NC = mmCameraModels[kfNewSource.mCamName].UnProject(measNewSource.v2RootPos);
        int nLevelScale = LevelScale(measNewSource.nLevel);
        point.mv3OneRightFromCenter_NC = mmCameraModels[kfNewSource.mCamName].UnProject(measNewSource.v2RootPos + vec(CVD::ImageRef(nLevelScale,0)));
        point.mv3OneDownFromCenter_NC  = mmCameraModels[kfNewSource.mCamName].UnProject(measNewSource.v2RootPos + vec(CVD::ImageRef(0,nLevelScale)));
      
        point.RefreshPixelVectors();
      }
      */
      /*
      if(meas.eSource == Measurement::SRC_ROOT || meas.eSource == Measurement::SRC_EPIPOLAR)
        point.mbBad = true;
      */
    }
  }
  
  // If the furthest MKF was the fixed one, choose the closest one to it to become the new fixed MKF
  if(pFurthestMKF->mbFixed)  
  {
    MultiKeyFrame* pNextFurthestMKF = ClosestMultiKeyFrame(*pFurthestMKF);
    pNextFurthestMKF->mbFixed = true;
  }
}

// Add a new MultikeyFrame to the map, erase the one at the end of the map
void MapMakerServerBase::AddMultiKeyFrameAndMarkLastDeleted(MultiKeyFrame *pMKF, bool bMakeRest)
{
  if(mMap.mlpMultiKeyFrames.size() > 1)
  {
    MultiKeyFrame& mkf = *(mMap.mlpMultiKeyFrames.back());
    mkf.mbDeleted = true;
  }
 
  pMKF->mbFixed = false;
  
  // Make a SmallBlurryImage of the keyframes: The relocaliser uses these. 
  // Alternatively make rest.
  for(KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);
    if(bMakeRest)
      kf.MakeKeyFrame_Rest();
  }
    
  mMap.mlpMultiKeyFrames.push_back(pMKF);
  
  mBundleAdjuster.SetNotConverged();
}

// Common code for adding a MultiKeyFrame to a map and generating new MapPoints through epipolar search
bool MapMakerServerBase::AddMultiKeyFrameAndCreatePoints(MultiKeyFrame *pMKF)
{
  ROS_INFO_STREAM("MapMakerServerBase: Adding new MKF to map, mean depth: "<<pMKF->mdTotalDepthMean);
  
  pMKF->mbFixed = false;
  
  for(KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
    it->second->MakeKeyFrame_Rest();
    
  for(KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);
    ReFindInSingleKeyFrame(kf);
  }
  
  int nLargePointsAdded = 0;
  int nStartMapSize = mMap.mlpPoints.size();

  // Add map points by epipolar search excluding local region, only the two coarsest levels to start
  // If we can't even add coarse level points, then the pose estimate of the new MKF is probably bad
  // so don't continue
  
  AddStereoMapPoints(*pMKF, 3, std::numeric_limits<int>::max(), -1.0, KF_ONLY_OTHER);       
  AddStereoMapPoints(*pMKF, 2, std::numeric_limits<int>::max(), -1.0, KF_ONLY_OTHER);
  nLargePointsAdded = mMap.mlpPoints.size() - nStartMapSize;
  
  static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN|SILENT);
  
  if(!MapMakerServerBase::sbLargePointTest || nLargePointsAdded > 0)  // ok, new MKF good
  {
    AddStereoMapPoints(*pMKF, 1, std::numeric_limits<int>::max(), -1.0, KF_ONLY_OTHER);
    if(*gvnLevelZeroPoints)
      AddStereoMapPoints(*pMKF, 0, std::numeric_limits<int>::max(), -1.0, KF_ONLY_OTHER);
  
    mMap.mlpMultiKeyFrames.push_back(pMKF);
    ROS_INFO("Just added MKF to map");
  
    if(pMKF->mmpKeyFrames.size() > 1)  // we have some siblings
    {
      AddStereoMapPoints(*pMKF, 3, std::numeric_limits<int>::max(), -1.0, KF_ONLY_SELF);       // Add map points by epipolar search, local region only
      AddStereoMapPoints(*pMKF, 2, std::numeric_limits<int>::max(), -1.0, KF_ONLY_SELF);
      AddStereoMapPoints(*pMKF, 1, std::numeric_limits<int>::max(), -1.0, KF_ONLY_SELF);
      if(*gvnLevelZeroPoints)
        AddStereoMapPoints(*pMKF, 0, std::numeric_limits<int>::max(), -1.0, KF_ONLY_SELF);
    }
    
    pMKF->RefreshSceneDepthRobust();
    mBundleAdjuster.SetNotConverged();
    
    ROS_INFO_STREAM("MapMakerServerBase: Created "<<mMap.mlpPoints.size() - nStartMapSize<<" new map points");
  }
  else  // couldn't even add one large point? MKF is probably bad
  {
    ROS_WARN("MapMakerServerBase: Couldn't even add one large point, new MultiKeyFrame probably bad");
    return false;
  }
  
  return true;
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt 
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMakerServerBase::ThinCandidates(KeyFrame &kf, int nLevel)
{
  std::vector<Candidate> &vCSrc = kf.maLevels[nLevel].vCandidates;
  std::vector<Candidate> vCGood;
  std::vector<CVD::ImageRef> vBusyLevelPos;
  
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  for(MeasPtrMap::iterator it = kf.mmpMeasurements.begin(); it!=kf.mmpMeasurements.end(); ++it)
  {
    Measurement& meas = *(it->second);
    
    if(!(meas.nLevel == nLevel || meas.nLevel == nLevel + 1))
      continue;
    vBusyLevelPos.push_back(CVD::ir_rounded(meas.v2RootPos / LevelScale(nLevel)));
  }
  
  // Only keep those candidates further than 10 pixels away from busy positions.
  unsigned int nMinMagSquared = 10*10;
  for(unsigned int i=0; i<vCSrc.size(); i++)
  {
    CVD::ImageRef irC = vCSrc[i].irLevelPos;
    bool bGood = true;
    for(unsigned int j=0; j<vBusyLevelPos.size(); j++)
    {
      CVD::ImageRef irB = vBusyLevelPos[j];
      if((irB - irC).mag_squared() < nMinMagSquared)
      {
        bGood = false;
        break;
      }
    }
    if(bGood)
      vCGood.push_back(vCSrc[i]);
  } 
  vCSrc = vCGood;
}

// Adds map points by epipolar search to the last-added KeyFrames, at a single
// specified pyramid level. Does epipolar search in the target KeyFrames as closest by
// the ClosestKeyFramesWithinDist function.
void MapMakerServerBase::AddStereoMapPoints(MultiKeyFrame& mkfSrc, int nLevel, int nLimit, double dDistThresh, KeyFrameRegion region)
{
  for(KeyFramePtrMap::iterator it = mkfSrc.mmpKeyFrames.begin(); it != mkfSrc.mmpKeyFrames.end(); it++)
  {
    KeyFrame &kfSrc = *(it->second);
    Level &level = kfSrc.maLevels[nLevel];
    ThinCandidates(kfSrc, nLevel);
    
    double dDistThreshUsed = dDistThresh;
    if(dDistThreshUsed < 0) // means we need to compute the thresh on a per-KF basis here
    {
      //dDistThreshUsed = 5 * kfSrc.mdSceneDepthMean * 0.3;
      dDistThreshUsed = 1000;
    }
    
    std::vector<KeyFrame*> vpTargets = ClosestKeyFramesWithinDist(kfSrc,dDistThreshUsed,MapMakerServerBase::snMaxTriangulationKFs, region); 
    
    ROS_DEBUG_STREAM("Adding epipolar points, source kf: "<<kfSrc.mCamName<<"  num targets: "<<vpTargets.size());
        
    int numSuccess = 0;
    for(unsigned j=0; j < vpTargets.size(); ++j)
    {
      KeyFrame& kfTarget = *vpTargets[j];
      if(kfTarget.mpParent->mbBad)
        continue;
        
      ROS_DEBUG_STREAM("Target: "<<kfTarget.mCamName<<"  source candidate points: "<<level.vCandidates.size());
      for(unsigned int i = 0; i<level.vCandidates.size(); ++i)
      {
        if(AddPointEpipolar(kfSrc, kfTarget, nLevel, i))
          numSuccess++;
          
        if(numSuccess >= nLimit)
          break;
      }
    }
  }
}

//  Adds map points at a given initial depth in a specified pyramid level
void MapMakerServerBase::AddInitDepthMapPoints(MultiKeyFrame& mkfSrc, int nLevel, int nLimit, double dInitDepth)
{
  for(KeyFramePtrMap::iterator it = mkfSrc.mmpKeyFrames.begin(); it != mkfSrc.mmpKeyFrames.end(); it++)
  {
    KeyFrame &kfSrc = *(it->second);
    TaylorCamera &cameraSrc = mmCameraModels[kfSrc.mCamName];
    Level &level = kfSrc.maLevels[nLevel];
    ThinCandidates(kfSrc, nLevel);
    
    int nLevelScale = LevelScale(nLevel);
    
    std::cout<<"AddInitDepthMapPoints, processing "<<level.vCandidates.size()<<" candidates"<<std::endl;
    
    for(unsigned int i = 0; i<level.vCandidates.size() && (int)i < nLimit; ++i)
    {
      Vector<2> v2RootPos = LevelZeroPos(level.vCandidates[i].irLevelPos, nLevel);
      Vector<3> v3UnProj = cameraSrc.UnProject(v2RootPos) * dInitDepth;  // This unprojects the noisy image location to a depth of dInitDepth
    
      MapPoint* pPointNew = new MapPoint;
      pPointNew->mv3WorldPos = kfSrc.mse3CamFromWorld.inverse() * v3UnProj;
      pPointNew->mpPatchSourceKF = &kfSrc;
      pPointNew->mbFixed = false;
      pPointNew->mnSourceLevel = nLevel;
      pPointNew->mv3Normal_NC = makeVector( 0,0,-1);
      pPointNew->mirCenter = level.vCandidates[i].irLevelPos;
      pPointNew->mv3Center_NC = cameraSrc.UnProject(v2RootPos); 
      pPointNew->mv3OneRightFromCenter_NC = cameraSrc.UnProject(v2RootPos + vec(CVD::ImageRef(nLevelScale,0))); 
      pPointNew->mv3OneDownFromCenter_NC  = cameraSrc.UnProject(v2RootPos + vec(CVD::ImageRef(0,nLevelScale))); 

      normalize(pPointNew->mv3Center_NC);
      normalize(pPointNew->mv3OneDownFromCenter_NC);
      normalize(pPointNew->mv3OneRightFromCenter_NC);

      pPointNew->RefreshPixelVectors();
  
      mMap.mlpPoints.push_back(pPointNew);
    
      Measurement* pMeas = new Measurement;
      pMeas->v2RootPos = v2RootPos;
      pMeas->nLevel = nLevel;
      pMeas->eSource = Measurement::SRC_ROOT;
      
      kfSrc.AddMeasurement(pPointNew, pMeas);
      //kfSrc.mmpMeasurements[pPointNew] = pMeas;
      //pPointNew->mMMData.spMeasurementKFs.insert(&kfSrc);
    }
  }
}

// Apply a scaling to all the MapPoints and MultiKeyFrames of the map
void MapMakerServerBase::ApplyGlobalScaleToMap(double dScale)
{
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    (*point_it)->mv3WorldPos *= dScale;
  }
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    mkf.mse3BaseFromWorld.get_translation() *= dScale;
    
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      kf.mse3CamFromWorld = kf.mse3CamFromBase * mkf.mse3BaseFromWorld; // CHECK!! GOOD
      kf.RefreshSceneDepthRobust();   
    }
  }
  
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    (*point_it)->RefreshPixelVectors();
  }
  
}

// Rotates/translates the whole map and all keyFrames
void MapMakerServerBase::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  for(MultiKeyFramePtrList::iterator it = mMap.mlpMultiKeyFrames.begin(); it != mMap.mlpMultiKeyFrames.end(); ++it)
  {
    MultiKeyFrame& mkf = *(*it);
    mkf.mse3BaseFromWorld = mkf.mse3BaseFromWorld * se3NewFromOld.inverse();
    for(KeyFramePtrMap::iterator jiter = mkf.mmpKeyFrames.begin(); jiter != mkf.mmpKeyFrames.end(); ++jiter)
    {
      KeyFrame& kf = *(jiter->second);      
      kf.mse3CamFromWorld = kf.mse3CamFromBase * mkf.mse3BaseFromWorld;  // CHECK!! GOOD
    }
  }
  
  for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it)
  {
    MapPoint& point = *(*it);
    point.mv3WorldPos = se3NewFromOld * point.mv3WorldPos;
    point.RefreshPixelVectors();
  }
}

// Used for comparing scoring tuples in AddPointEpipolar
bool compScores(const std::tuple<int,int,Vector<2> >& one, const std::tuple<int,int,Vector<2> >& two) { return std::get<0>(one) < std::get<0>(two); }

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMakerServerBase::AddPointEpipolar(KeyFrame &kfSrc, KeyFrame &kfTarget, int nLevel, int nCandidate)
{
  //debug
  static gvar3<int> gvnCrossCamera("CrossCamera", 1, HIDDEN|SILENT);
  if(!*gvnCrossCamera && kfSrc.mCamName != kfTarget.mCamName)
    return false;
  
  TaylorCamera &cameraSrc = mmCameraModels[kfSrc.mCamName];
  TaylorCamera &cameraTarget = mmCameraModels[kfTarget.mCamName];
  
  int nLevelScale = LevelScale(nLevel);
  Candidate& candidate = kfSrc.maLevels[nLevel].vCandidates[nCandidate];
  CVD::ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel); // The pixel coords of the candidate at level zero
  
  Vector<3> v3Ray_SC = cameraSrc.UnProject(v2RootPos);  // The pixel coords unprojected into a 3d half-line in the source kf frame
  Vector<3> v3LineDirn_TC = kfTarget.mse3CamFromWorld.get_rotation() * (kfSrc.mse3CamFromWorld.get_rotation().inverse() * v3Ray_SC);  // The direction of that line in the target kf frame
  Vector<3> v3CamCenter_TC = kfTarget.mse3CamFromWorld * kfSrc.mse3CamFromWorld.inverse().get_translation(); // The position of the source kf in the target kf frame
  Vector<3> v3CamCenter_SC = kfSrc.mse3CamFromWorld * kfTarget.mse3CamFromWorld.inverse().get_translation(); // The position of the target kf in the source kf frame
  
  double dMaxEpiAngle = M_PI / 3;  // the maximum angle spanned by two view rays allowed
  double dMinEpiAngle = 0.05;     // the minimum angle allowed
  
  // Want to figure out the min and max depths allowed on the source ray, which will be determined by the minimum and 
  // maximum allowed epipolar angle
  // See diagram below, which shows the min and max epipolar angles.
  /*
   *              /\
   *             / m\ 
   *            /  i \
   *           /`. n  \
   *          / m `.   \
   *         /  a   `.  \
   *        /   x     `. \
   *       /____________`.\
   *    Source           Target
   */
  
  
  double dSeparationDist = norm(v3CamCenter_SC);
  double dSourceAngle = acos((v3CamCenter_SC * v3Ray_SC)/dSeparationDist);  // v3Ray_SC is unit length so don't have to divide
  
  double dMinTargetAngle = M_PI - dSourceAngle - dMaxEpiAngle;
  double dStartDepth = dSeparationDist * sin(dMinTargetAngle)/sin(dMaxEpiAngle);
  
  double dMaxTargetAngle = M_PI - dSourceAngle - dMinEpiAngle;
  double dEndDepth = dSeparationDist * sin(dMaxTargetAngle)/sin(dMinEpiAngle);
  
  if(dStartDepth < 0.2)  // don't bother looking too close
    dStartDepth = 0.2;
  
  ROS_DEBUG_STREAM("dStartDepth: "<<dStartDepth<<" dEndDepth: "<<dEndDepth);
  
  Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;   // The start of the epipolar line segment in the target kf frame
  Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;       // The end of the epipolar line segment in the target kf frame
  
  // Project epipolar line segment start and end points onto unit sphere and check for minimum distance between them
  Vector<3> v3A = v3RayStart_TC; 
  normalize(v3A);
  Vector<3> v3B = v3RayEnd_TC; 
  normalize(v3B);
  Vector<3> v3BetweenEndpoints = v3A-v3B;
  
  if(v3BetweenEndpoints * v3BetweenEndpoints < 0.00000001)
  {
    ROS_DEBUG_STREAM("MapMakerServerBase: v3BetweenEndpoints too small.");
    return false;
  }
  
  // Now we want to construct a bunch of hypothetical point locations, so we can warp the source patch
  // into the target KF and look for a match. To do this, need to partition the epipolar arc in the target
  // KF equally, rather than the source ray equally. The epipolar arc lies at the intersection of the epipolar
  // plane and the unit circle of the target KF. We will construct a matrix that projects 3-vectors onto 
  // the epipolar plane, and use it to define the start and stop coordinates of a unit circle by
  // projecting the ray start and ray end vectors. Then it's just a matter of partitioning the unit circle, and 
  // projecting each partition point onto the source ray (keeping in mind that the source ray is in the 
  // epipolar plane too). 
  
  // Find the normal of the epipolar plane
  Vector<3> v3PlaneNormal = v3A ^ v3B; 
  normalize(v3PlaneNormal);
  Vector<3> v3PlaneI = v3A;  // Lets call the vector we got from the start of the epipolar line segment the new coordinate frame's x axis
  Vector<3> v3PlaneJ = v3PlaneNormal ^ v3PlaneI;   // Get the y axis
  
  // This will convert a 3D point to the epipolar plane's coordinate frame
  Matrix<3> m3ToPlaneCoords;
  m3ToPlaneCoords[0] = v3PlaneI;
  m3ToPlaneCoords[1] = v3PlaneJ;
  m3ToPlaneCoords[2] = v3PlaneNormal;
  
  Vector<2> v2PlaneB = (m3ToPlaneCoords * v3B).slice<0,2>();  // The vector we got from the end of the epipolar line segment, in epipolar plane coordinates
  Vector<2> v2PlaneI = makeVector(1,0);
  
  double dMaxAngleAlongCircle = acos(v2PlaneB * v2PlaneI);   // The angle between point B (now a 2D point in the plane) and the x axis

  // For stepping along the circle
  double dAngleStep = cameraTarget.OnePixelAngle() * LevelScale(nLevel) * 3;
  int nSteps = ceil(dMaxAngleAlongCircle / dAngleStep);
  dAngleStep = dMaxAngleAlongCircle / nSteps;
  
  Vector<2> v2RayStartInPlane = (m3ToPlaneCoords * v3RayStart_TC).slice<0,2>();
  Vector<2> v2RayEndInPlane = (m3ToPlaneCoords * v3RayEnd_TC).slice<0,2>();
  Vector<2> v2RayDirInPlane = v2RayEndInPlane - v2RayStartInPlane;
  normalize(v2RayDirInPlane);
  
  std::vector<std::pair<Vector<3>, Vector<3> > > vMapPointPositions;  // first in world frame, second in camera frame
  SE3<> se3WorldFromTargetCam = kfTarget.mse3CamFromWorld.inverse();
  for(int i=0; i < nSteps + 1; ++i)  // stepping along circle
  {
    double dAngle = i*dAngleStep;  // current angle
    Vector<2> v2CirclePoint = makeVector(cos(dAngle), sin(dAngle));  // point on circle
    
    // Backproject onto view ray, this is the depth along view ray where we intersect
    double dAlpha = (v2RayStartInPlane[0]*v2CirclePoint[1] - v2RayStartInPlane[1]*v2CirclePoint[0]) / 
                   (v2RayDirInPlane[1]*v2CirclePoint[0] - v2RayDirInPlane[0]*v2CirclePoint[1]);
    
    Vector<3> v3PointPos_TC = v3RayStart_TC + dAlpha*v3LineDirn_TC;
    Vector<3> v3PointPos = se3WorldFromTargetCam * v3PointPos_TC;
    vMapPointPositions.push_back(std::make_pair(v3PointPos, v3PointPos_TC));
  }
  
  // This will be the map point that we place at the different depths in order to generate warped patches
  MapPoint point;
  point.mpPatchSourceKF = &kfSrc;
  point.mnSourceLevel = nLevel;
  point.mv3Normal_NC = makeVector( 0,0,-1);
  point.mirCenter = irLevelPos;
  point.mv3Center_NC = cameraSrc.UnProject(v2RootPos);
  point.mv3OneRightFromCenter_NC = cameraSrc.UnProject(v2RootPos + vec(CVD::ImageRef(nLevelScale,0))); 
  point.mv3OneDownFromCenter_NC  = cameraSrc.UnProject(v2RootPos + vec(CVD::ImageRef(0,nLevelScale))); 
  
  normalize(point.mv3Center_NC);
  normalize(point.mv3OneRightFromCenter_NC);
  normalize(point.mv3OneDownFromCenter_NC);
  
  PatchFinder finder;
  int nMaxZMSSD = finder.mnMaxSSD + 1;
  int nBestZMSSD = nMaxZMSSD;
  int nBest = -1;
  Vector<2> v2BestMatch = Zeros;
  
  std::vector<std::tuple<int,int, Vector<2> > > vScoresIndicesBestMatches;
  
  for(unsigned i=0; i < vMapPointPositions.size(); ++i)  // go through all our hypothesized map points
  {
    point.mv3WorldPos = vMapPointPositions[i].first;
    point.RefreshPixelVectors();
    
    Vector<2> v2Image = cameraTarget.Project(vMapPointPositions[i].second);
    
    if(cameraTarget.Invalid())
      continue;
      
    if(!kfTarget.maLevels[0].image.in_image(CVD::ir(v2Image)))
      continue;
      
    // Check if projected point is in a masked portion of the target keyframe
    if(kfTarget.maLevels[0].mask.totalsize() > 0 && kfTarget.maLevels[0].mask[CVD::ir(v2Image)] == 0)
      continue;
    
    Matrix<2> m2CamDerivs = cameraTarget.GetProjectionDerivs();
     
    int nSearchLevel = finder.CalcSearchLevelAndWarpMatrix(point,kfTarget.mse3CamFromWorld,m2CamDerivs);
    if(nSearchLevel == -1)
      continue;
      
    finder.MakeTemplateCoarseCont(point);
    
    if(finder.TemplateBad())
      continue;
         
    int nScore;
    bool bExhaustive = false;  // Should we do an exhaustive search of the target area? Should maybe make this into a param
    bool bFound = finder.FindPatchCoarse(CVD::ir(v2Image), kfTarget, 3, nScore, bExhaustive);
    
    if(!bFound)
      continue;
    
    vScoresIndicesBestMatches.push_back(std::make_tuple(nScore, i, finder.GetCoarsePosAsVector()));
      
    if(nScore < nBestZMSSD)
    {
      nBestZMSSD = nScore;
      nBest = i;
      v2BestMatch = finder.GetCoarsePosAsVector();
    }
  }
  
  if(nBest == -1)
  {
    ROS_DEBUG_STREAM("No match found.");
    return false;
  }
  
  std::sort(vScoresIndicesBestMatches.begin(), vScoresIndicesBestMatches.end(), compScores);
  
  // We want matches that are unambigous, so if there are many good matches along the view ray,
  // we can't say for certain where the best one really is. Therefore, implement the following rule:
  // Best zmssd has to be 10% better than nearest other, unless that nearest other is 1 index away 
  // from best
  
  int nResizeTo = 1;
  for(unsigned i=1; i < vScoresIndicesBestMatches.size(); ++i)
  {
    if(std::get<0>(vScoresIndicesBestMatches[i]) > nBestZMSSD*0.9)  // within 10% of best
      nResizeTo++;
  }
  
  // Too many high scoring points, since the best can be within 10% of at most two other points.
  // We can't be certain of what is best, get out of here
  if(nResizeTo > 3)  
    return false;
  
  vScoresIndicesBestMatches.resize(nResizeTo); // chop!
  
  // All the points left in vScoresIndicesBestMatches should be within 1 idx of best, otherwise our matches are ambigous
  // Test index distance:
  for(unsigned i=1; i < vScoresIndicesBestMatches.size(); ++i)
  {
    if(abs(std::get<1>(vScoresIndicesBestMatches[i]) - nBest) > 1) // bad, index too far away, get out of here
      return false;
  }
  
  // Now all the points in vScoresIndicesBestMatches can be considered potential matches
  
  Vector<2> v2SubPixPos = makeVector(-1,-1);
  bool bGotGoodSubpix = false;
  for(unsigned i=0; i < vScoresIndicesBestMatches.size(); ++i)  // go through all potential good matches
  {
    int nCurrBest = std::get<1>(vScoresIndicesBestMatches[i]);
    Vector<2> v2CurrBestMatch = std::get<2>(vScoresIndicesBestMatches[i]);
    
    point.mv3WorldPos = vMapPointPositions[nCurrBest].first;
    point.RefreshPixelVectors();
  
    cameraTarget.Project(vMapPointPositions[nCurrBest].second);
    Matrix<2> m2CamDerivs = cameraTarget.GetProjectionDerivs();
    
    finder.CalcSearchLevelAndWarpMatrix(point,kfTarget.mse3CamFromWorld,m2CamDerivs);
    finder.MakeTemplateCoarseCont(point);
    finder.SetSubPixPos(v2CurrBestMatch);
    
    // Try to get subpixel convergence
    bool bSubPixConverges = finder.IterateSubPixToConvergence(kfTarget,10);

    if(!bSubPixConverges) 
      continue;
      
    // First one to make it here wins. Keep in mind that vScoresIndicesBestMatches is ordered by
    // score, so we're trying the points in descent order of potential
    bGotGoodSubpix = true;
    v2SubPixPos = finder.GetSubPixPos();
    break;
  }
  
  // None of the candidates had subpix converge? Bad match...
  if(!bGotGoodSubpix)
    return false;
  
  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kfTarget.mse3CamFromWorld.inverse() *  
          ReprojectPoint(kfSrc.mse3CamFromWorld * kfTarget.mse3CamFromWorld.inverse(),
                         cameraSrc.UnProject(v2RootPos),   
                         cameraTarget.UnProject(v2SubPixPos));   
  
  MapPoint *pPointNew = new MapPoint;
  
  pPointNew->mv3WorldPos = v3New;
  
  // Patch source stuff:
  pPointNew->mpPatchSourceKF = &kfSrc;
  pPointNew->mnSourceLevel = nLevel;
  pPointNew->mv3Normal_NC = makeVector( 0,0,-1);
  pPointNew->mirCenter = irLevelPos;
  pPointNew->mv3Center_NC = cameraSrc.UnProject(v2RootPos); 
  pPointNew->mv3OneRightFromCenter_NC = cameraSrc.UnProject(v2RootPos + vec(CVD::ImageRef(nLevelScale,0))); 
  pPointNew->mv3OneDownFromCenter_NC  = cameraSrc.UnProject(v2RootPos + vec(CVD::ImageRef(0,nLevelScale))); 
  
  normalize(pPointNew->mv3Center_NC);
  normalize(pPointNew->mv3OneDownFromCenter_NC);
  normalize(pPointNew->mv3OneRightFromCenter_NC);
  
  pPointNew->RefreshPixelVectors();
    
  Measurement* pMeasSrc = new Measurement;
  pMeasSrc->eSource = Measurement::SRC_ROOT;
  pMeasSrc->v2RootPos = v2RootPos;
  pMeasSrc->nLevel = nLevel;
  pMeasSrc->bSubPix = true;
 
  Measurement* pMeasTarget = new Measurement;
  *pMeasTarget = *pMeasSrc;   // copy data
  pMeasTarget->eSource = Measurement::SRC_EPIPOLAR;
  pMeasTarget->v2RootPos = v2SubPixPos;
  
  // Record map point and its measurement in the right places
  kfSrc.AddMeasurement(pPointNew, pMeasSrc);
  kfTarget.AddMeasurement(pPointNew, pMeasTarget);
  
  //kfSrc.mmpMeasurements[pPointNew] = pMeasSrc;
  //kfTarget.mmpMeasurements[pPointNew] = pMeasTarget;
  //pPointNew->mMMData.spMeasurementKFs.insert(&kfSrc);
  //pPointNew->mMMData.spMeasurementKFs.insert(&kfTarget);
  
  mMap.mlpPoints.push_back(pPointNew);
  mlpNewQueue.push_back(pPointNew);
  
  return true;
  
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in 
// TrackerData.h.
bool MapMakerServerBase::ReFind_Common(KeyFrame &kf, MapPoint &point)
{
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption
  if(point.mMMData.spMeasurementKFs.count(&kf) || point.mMMData.spNeverRetryKFs.count(&kf))
    return false;
    
  if(point.mbBad)
    return false;
  
  if(kf.mpParent->mbBad)
    return false;
    
  //debug
  static gvar3<int> gvnCrossCamera("CrossCamera", 1, HIDDEN|SILENT);
  if(!*gvnCrossCamera && kf.mCamName != point.mpPatchSourceKF->mCamName)
    return false;
  
  static PatchFinder finder;
  Vector<3> v3Cam =  kf.mse3CamFromWorld * point.mv3WorldPos;
  
  TaylorCamera &camera = mmCameraModels[kf.mCamName];
  Vector<2> v2Image = camera.Project(v3Cam);
  
  if(camera.Invalid())
  {
    point.mMMData.spNeverRetryKFs.insert(&kf);
    return false;
  }

  CVD::ImageRef irImageSize = kf.maLevels[0].image.size();
  if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
  {
    point.mMMData.spNeverRetryKFs.insert(&kf);
    return false;
  }
  
  Matrix<2> m2CamDerivs = camera.GetProjectionDerivs();
  finder.MakeTemplateCoarse(point, kf.mse3CamFromWorld, m2CamDerivs);
  
  if(finder.TemplateBad())
  {
    point.mMMData.spNeverRetryKFs.insert(&kf);
    return false;
  }
  
  int nScore;
  bool bFound = finder.FindPatchCoarse(CVD::ir(v2Image), kf, 4, nScore);  // Very tight search radius!
  if(!bFound)
  {
    point.mMMData.spNeverRetryKFs.insert(&kf);
    return false;
  }
  
  // If we found something, generate a measurement struct and put it in the map
  Measurement* pMeas = new Measurement;
  pMeas->nLevel = finder.GetLevel();
  pMeas->eSource = Measurement::SRC_REFIND;
  
  if(finder.GetLevel() > 0)
  {
    finder.MakeSubPixTemplate();
    finder.SetSubPixPos(finder.GetCoarsePosAsVector());
    finder.IterateSubPixToConvergence(kf,8);
    pMeas->v2RootPos = finder.GetSubPixPos();
    pMeas->bSubPix = true;
  }
  else
  {
    pMeas->v2RootPos = finder.GetCoarsePosAsVector();
    pMeas->bSubPix = false;
  }
  
  if(kf.mmpMeasurements.count(&point))
    ROS_BREAK(); // This should never happen, we checked for this at the start.
  
  kf.AddMeasurement(&point, pMeas);
  
  //kf.mmpMeasurements[&point] = pMeas;
  //point.mMMData.spMeasurementKFs.insert(&kf);
    
  return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMakerServerBase::ReFindInSingleKeyFrame(KeyFrame &kf)
{
  int nFoundNow = 0;
  
  for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it)
  {
    MapPoint& point = *(*it);
    
    if(ReFind_Common(kf,point))
      nFoundNow++;
  }
  
  return nFoundNow;
}

// When new map points are generated, they're only created from a stereo pair.
// This tries to make additional measurements in other KFs which they might
// be in.
void MapMakerServerBase::ReFindNewlyMade()
{
  if(mlpNewQueue.empty())
    return;
  
  int nFound = 0;
  int nBad = 0;
  while(!mlpNewQueue.empty() && IncomingQueueSize() == 0)
  {
    MapPoint* pPointNew = mlpNewQueue.front();
    mlpNewQueue.pop_front();
    if(pPointNew->mbBad)
    {
      nBad++;
      continue;
    }

    for(MultiKeyFramePtrList::iterator it = mMap.mlpMultiKeyFrames.begin(); it != mMap.mlpMultiKeyFrames.end(); ++it)
    {
      if(IncomingQueueSize() != 0)  // there could be a lot of MultiKeyFrames, let loop bail out if necessary
        break;
      
      MultiKeyFrame& mkf = *(*it);
      if(mkf.mbBad)
        continue;
      
      for(KeyFramePtrMap::iterator jiter = mkf.mmpKeyFrames.begin(); jiter != mkf.mmpKeyFrames.end(); ++jiter)
      {
        KeyFrame& kf = *(jiter->second);
        
        if(ReFind_Common(kf, *pPointNew))
          nFound++;
      }
    }
  }
}

// Dud measurements get a second chance.
void MapMakerServerBase::ReFindFromFailureQueue()
{
  if(mlFailureQueue.size() == 0)
    return;
    
  mlFailureQueue.sort();

  int nFound=0;
  while(!mlFailureQueue.empty() && IncomingQueueSize() == 0)
  {
    std::pair<KeyFrame*, MapPoint*> failurePair = mlFailureQueue.front();
    mlFailureQueue.pop_front();
    
    if(ReFind_Common(*(failurePair.first), *(failurePair.second)))
      nFound++;
  }
  
}


// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMakerServerBase::CalcPlaneAligner()
{
  unsigned int nPoints = mMap.mlpPoints.size();
  if(nPoints < 10)
  {
    ROS_INFO("MapMakerServerBase: CalcPlane: too few points to calc plane.");
    return SE3<>();
  };
  
  int nRansacs = 100;
  Vector<3> v3BestMean = Zeros;
  Vector<3> v3BestNormal = Zeros;
  double dBestDistSquared = 9999999999999999.9;
  
  std::vector<MapPoint*> vpPoints;
  vpPoints.reserve(nPoints);
  
  for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it)
    vpPoints.push_back(*it);
  
  for(int i=0; i<nRansacs; i++)
  {
    int nA = rand()%nPoints;
    int nB = nA;
    int nC = nA;
    while(nB == nA)
      nB = rand()%nPoints;
    while(nC == nA || nC==nB)
      nC = rand()%nPoints;
    
    Vector<3> v3Mean = 0.33333333 * (vpPoints[nA]->mv3WorldPos + 
             vpPoints[nB]->mv3WorldPos + 
             vpPoints[nC]->mv3WorldPos);
    
    Vector<3> v3CA = vpPoints[nC]->mv3WorldPos  - vpPoints[nA]->mv3WorldPos;
    Vector<3> v3BA = vpPoints[nB]->mv3WorldPos  - vpPoints[nA]->mv3WorldPos;
    Vector<3> v3Normal = v3CA ^ v3BA;
    if(v3Normal * v3Normal  == 0)
      continue;
    normalize(v3Normal);
    
    double dSumError = 0.0;
    for(unsigned int i=0; i<nPoints; i++)
    {
      Vector<3> v3Diff = vpPoints[i]->mv3WorldPos - v3Mean;
      double dDistSq = v3Diff * v3Diff;
      if(dDistSq == 0.0)
        continue;
      double dNormDist = fabs(v3Diff * v3Normal);
      
      if(dNormDist > 0.05)
        dNormDist = 0.05;
      dSumError += dNormDist;
    }
    if(dSumError < dBestDistSquared)
    {
      dBestDistSquared = dSumError;
      v3BestMean = v3Mean;
      v3BestNormal = v3Normal;
    }
  }
  
  // Done the ransacs, now collect the supposed inlier set
  std::vector<Vector<3> > vInliers;
  for(unsigned int i=0; i<nPoints; i++)
  {
    Vector<3> v3Diff = vpPoints[i]->mv3WorldPos - v3BestMean;
    double dDistSq = v3Diff * v3Diff;
    if(dDistSq == 0.0)
      continue;
    double dNormDist = fabs(v3Diff * v3BestNormal);
    if(dNormDist < 0.05)
      vInliers.push_back(vpPoints[i]->mv3WorldPos);
  }
  
  // With these inliers, calculate mean and cov
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vInliers.size(); i++)
    v3MeanOfInliers+=vInliers[i];
  v3MeanOfInliers *= (1.0 / vInliers.size());
  
  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vInliers.size(); i++)
  {
    Vector<3> v3Diff = vInliers[i] - v3MeanOfInliers;
    m3Cov += v3Diff.as_col() * v3Diff.as_row();
  };
  
  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];
  
  // If mean of inliers Z is negative, we want positive plane normal to put camera above plane
  // If mean of inliers Z is positive, we want negative plane normal to put camera above plane
  if(v3MeanOfInliers[2] < 0 && v3Normal[2] < 0)
    v3Normal *= -1.0;
  else if(v3MeanOfInliers[2] > 0 && v3Normal[2] > 0)
    v3Normal *= -1.0;
  
  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];
  
  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;
  
  return se3Aligner;
}

// Marks points as bad or puts them in the failure queue for a second chance depending on where the point came from
void MapMakerServerBase::HandleOutliers(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers)
{
  std::set<MapPoint*> spBadPoints;
  std::set<MapPoint*> spFixedOutliers;
  std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliersRemoved;
  
  for(unsigned i=0; i < vOutliers.size(); ++i)
  {
    KeyFrame& kf = *(vOutliers[i].first);
    MapPoint& point = *(vOutliers[i].second);
    Measurement &meas = *(kf.mmpMeasurements[&point]);
    
    if(point.mbFixed) // fixed points can't be considered bad, but count them
    {
      spFixedOutliers.insert(&point);
      continue;
    }
    
    if(point.mMMData.GoodMeasCount() <= 2 || meas.eSource == Measurement::SRC_ROOT)   // Is the original source kf considered an outlier? That's bad.
    {
      point.mbBad = true;
      spBadPoints.insert(&point);
    }
    else if(!point.mbBad)  // need to check because point might have been set as bad from a previous outlier measurement
    {
      // Do we retry it? Depends where it came from!!
      if(meas.eSource == Measurement::SRC_TRACKER || meas.eSource == Measurement::SRC_EPIPOLAR)
        mlFailureQueue.push_back(vOutliers[i]);
      else
        point.mMMData.spNeverRetryKFs.insert(&kf);
        
      kf.EraseMeasurementOfPoint(&point); 
      int nErased = point.mMMData.spMeasurementKFs.erase(&kf);
      ROS_ASSERT(nErased);
      
      if(meas.bTransferred)
        vOutliersRemoved.push_back(vOutliers[i]);
    }
  }
  
  vOutliers.swap(vOutliersRemoved);
  
  ROS_INFO_STREAM("======== Number of outlier fixed points: "<<spFixedOutliers.size());
  
  if(spBadPoints.size() > 0)
  {
    ROS_INFO_STREAM("======== Handle outlier measurements marked "<<spBadPoints.size()<<" points as bad");
  } 
  
}

// Points marked bad are removed from the internal queues.
void MapMakerServerBase::EraseBadEntitiesFromQueues()
{
  for(std::list<std::pair<KeyFrame*, MapPoint*> >::iterator jit = mlFailureQueue.begin(); jit != mlFailureQueue.end();)
  {
    KeyFrame* pKF = jit->first;
    MapPoint* pPoint = jit->second;
    
    if(pPoint->mbBad || pKF->mpParent->mbBad)
      mlFailureQueue.erase(jit++);
    else
      ++jit;
  }
  
  
  for(std::list<MapPoint*>::iterator jit = mlpNewQueue.begin(); jit != mlpNewQueue.end();)
  {
    MapPoint* pPoint = *jit;
    
    if(pPoint->mbBad)
      mlpNewQueue.erase(jit++);
    else
      ++jit;
  }
}

void MapMakerServerBase::ClearInternalQueues()
{
  mlFailureQueue.clear();
  mlpNewQueue.clear();
}
