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
//=========================================================================================

#include <mcptam/MapMakerCalib.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/CalibImageTaylor.h>
#include <mcptam/Utility.h>
#include <mcptam/LevelHelpers.h>
#include <geometry_msgs/PoseArray.h>
#include <TooN/wls.h>

using namespace TooN;

MapMakerCalib::MapMakerCalib(Map& map, TaylorCameraMap& cameras, BundleAdjusterBase &bundleAdjuster)
: MapMakerBase(map, true)
, MapMaker(map, cameras, bundleAdjuster)  // this will start parent's run thread
{
  // Increase outlier limits to be more lenient
  MapMakerClientBase::snMinOutliers = 40;
  MapMakerClientBase::sdOutlierMultiplier = 2.0;
  
  // Limit the number of triangulation KFs
  MapMakerServerBase::snMaxTriangulationKFs = 3;
  
  mnSavedRunState = -1;
  mbPaused = false;
  meCalibState = CALIB_INVALID;
};

MapMakerCalib::~MapMakerCalib()
{
  
}

// Initialize the map
bool MapMakerCalib::InitFromCalibImage(CalibImageTaylor &calibImage, double dSquareSize, std::string cameraName, SE3<> &se3TrackerPose)
{
  // Create a new MKF
  MultiKeyFrame *pMKF = new MultiKeyFrame;
  
  pMKF->mse3BaseFromWorld = calibImage.mse3CamFromWorld; // set mkf pose to be pose from tracker
  pMKF->mse3BaseFromWorld.get_translation() *= dSquareSize;   // scale it
  pMKF->mbFixed = false;
 
  // Create a new KF (there will only be one for now)
  KeyFrame *pKF = new KeyFrame(pMKF, cameraName);
  pMKF->mmpKeyFrames[cameraName] = pKF;
  
  pKF->mse3CamFromWorld = pMKF->mse3BaseFromWorld;  // same as parent MKF
  pKF->mse3CamFromBase = SE3<>(); // set relative pose to identity;
  pKF->mbActive = true;

  pKF->MakeKeyFrame_Lite(calibImage.mImage, true);
  pKF->MakeKeyFrame_Rest();

  // Create MapPoints where the calib image corners are
  // Testing to see if we should create points at all levels or just level 0
  for(int l=0; l < LEVELS; ++l)
  {
    if(l >= 1)
      break;
      
    int nLevelScale = LevelScale(l);
    
    for(unsigned i=0; i < calibImage.mvGridCorners.size(); ++i)
    {
      MapPoint *pNewPoint = new MapPoint;
      pNewPoint->mv3WorldPos.slice<0,2>() = dSquareSize * CVD::vec(calibImage.mvGridCorners[i].mirGridPos);
      pNewPoint->mv3WorldPos[2] = 0.0;  // on z=0 plane
      pNewPoint->mbFixed = true;  // the calibration pattern is fixed
      pNewPoint->mbOptimized = true; // since it won't move it's already in its optimal location
      
      // Patch source stuff:
      pNewPoint->mpPatchSourceKF = pKF;
      pNewPoint->mnSourceLevel = l;
      
      Vector<2> v2RootPos = calibImage.mvGridCorners[i].mParams.v2Pos;
      
      // Same code as in MapMakerServerBase::AddPointEpipolar
      pNewPoint->mirCenter = CVD::ir_rounded(LevelNPos(v2RootPos, l));
      pNewPoint->mv3Center_NC = mmCameraModels[cameraName].UnProject(v2RootPos); 
      pNewPoint->mv3OneRightFromCenter_NC = mmCameraModels[cameraName].UnProject(v2RootPos + CVD::vec(CVD::ImageRef(nLevelScale,0))); 
      pNewPoint->mv3OneDownFromCenter_NC  = mmCameraModels[cameraName].UnProject(v2RootPos + CVD::vec(CVD::ImageRef(0,nLevelScale))); 
        
      normalize(pNewPoint->mv3Center_NC);
      normalize(pNewPoint->mv3OneDownFromCenter_NC);
      normalize(pNewPoint->mv3OneRightFromCenter_NC);
      
      pNewPoint->RefreshPixelVectors();
        
      mMap.mlpPoints.push_back(pNewPoint);
      
      // Create a measurement of the point
      Measurement* pMeas = new Measurement;
      pMeas->eSource = Measurement::SRC_ROOT;
      pMeas->v2RootPos = v2RootPos;
      pMeas->nLevel = l;
      pMeas->bSubPix = true;
      pKF->mmpMeasurements[pNewPoint] = pMeas;

      pNewPoint->mMMData.spMeasurementKFs.insert(pKF);
    }
  }
  
  mMap.mlpMultiKeyFrames.push_back(pMKF);
  
  
  /*
  {
    ROS_DEBUG("After creating points, before optimization: ");
    int i=0;
    double dErrorSum = 0;
    for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it, ++i)
    {
      std::cout<<"Point pos: "<<(*it)->mv3WorldPos;
      Vector<3> v3Cam = pKF->mse3CamFromWorld * (*it)->mv3WorldPos;
      Vector<2> v2Image = mmCameraModels[cameraName].Project(v3Cam);
      std::cout<<" Reprojected: "<<v2Image<<" Original: "<<calibImage.mvGridCorners[i].mParams.v2Pos;
      std::cout<<std::endl;
      
      TooN::Vector<2> v2Error = v2Image - calibImage.mvGridCorners[i].mParams.v2Pos;
      dErrorSum += v2Error * v2Error;
    }
    
    std::cout<<"Error sum: "<<dErrorSum<<" mean: "<<dErrorSum/i<<std::endl;
  }
  */

  mBundleAdjuster.SetNotConverged();
  
  int nSanityCounter = 0;
  std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliers;
  
  ROS_DEBUG("MapMakerCalib: Initialized from calib image, running bundle adjuster");
  
  while(!mBundleAdjuster.ConvergedFull())
  {
    mBundleAdjuster.BundleAdjustAll(vOutliers);
    if(ResetRequested() || nSanityCounter > 5)
    {
      ROS_ERROR("MapMakerCalib: Exceeded sanity counter or reset requested, bailing");
      return false;
    }
      
    nSanityCounter++;
  }
  
  // Don't allow any outliers when we're initializing, all of the calibration points turned MapPoints
  // should have been found
  if(vOutliers.size() > 0)
  {
    ROS_ERROR("MapMakerCalib: Found outliers in initialization, bailing");
    return false;
  }
  
  /*
  {
    ROS_DEBUG("After optimization: ");
    int i=0;
    double dErrorSum = 0;
    for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it, ++i)
    {
      std::cout<<"Point pos: "<<(*it)->mv3WorldPos;
      Vector<3> v3Cam = pKF->mse3CamFromWorld * (*it)->mv3WorldPos;
      Vector<2> v2Image = mmCameraModels[cameraName].Project(v3Cam);
      std::cout<<" Reprojected: "<<v2Image<<" Original: "<<calibImage.mvGridCorners[i].mParams.v2Pos;
      std::cout<<std::endl;
      
      TooN::Vector<2> v2Error = v2Image - calibImage.mvGridCorners[i].mParams.v2Pos;
      dErrorSum += v2Error * v2Error;
    }
    
    std::cout<<"Error sum: "<<dErrorSum<<" mean: "<<dErrorSum/i<<std::endl;
  }
  */
  
  // Get the updated pose, tracker will initialize with this
  se3TrackerPose = pKF->mse3CamFromWorld;
  
  ROS_INFO_STREAM("MapMakerCalib: Made initial map with " << mMap.mlpPoints.size() << " points.");
  ROS_INFO_STREAM("MapMakerCalib: tracker pose: "<<se3TrackerPose);
  
  mState = MM_RUNNING;   // not doing anything special for initialization (unlike MapMaker)
  mMap.mbGood = true;
  
  return true; 
}

// Removes MultiKeyFrames from the Map if it does/doesn't have a certain camera
void MapMakerCalib::RemoveMultiKeyFrames(std::string camName, bool bShouldHave)
{
  ROS_ASSERT(mbPaused);
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    
    // If should have, but doesn't have, or vice versa, mark as bad
    if((bool)mkf.mmpKeyFrames.count(camName) != bShouldHave)  
    {
      mkf.mbBad = true;
    }
  }
  
  HandleBadEntities(); // from MapMaker, this will move bad MKFs to trash
}

// Finds the average poses of all KFs relative to the first
SE3Map MapMakerCalib::FindAverageRelativePoses()
{
  SE3Map mPoses;
  
  // Need to initialize the poses for iterative finding of rotation, so search for first MKF that has all cameras
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    if(mkf.mmpKeyFrames.size() == mmCameraModels.size())  // found a multikeyframe with all cameras, use as source of initial poses
    {
      TooN::SE3<> se3FirstCamFromBase = (mkf.mmpKeyFrames.begin())->second->mse3CamFromBase;
      
      // Fill poses map
      for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); kf_it++)
      {
        mPoses[kf_it->first] = se3FirstCamFromBase.inverse() * kf_it->second->mse3CamFromBase;  
      }
      break;
    }      
  }
  
  if(mPoses.empty())  // didn't find any MKFs that have all the cameras
    return mPoses;
  
  // Check to make sure that the first pose is identity
  TooN::SE3<> se3FirstTransform = (mPoses.begin())->second;
  TooN::Vector<6> v6FirstTransform = se3FirstTransform.ln();
  ROS_ASSERT(v6FirstTransform * v6FirstTransform < 1e-10);
  
  double dEpsilon = 1e-3;
  
  // Loop through all the poses, and refine them by using the poses of ALL corresponding KeyFrames
  // Start at the second pose since the first will be identity
  for(SE3Map::iterator it = ++mPoses.begin(); it != mPoses.end(); it++)
  {
    std::string camName = it->first;
    SE3<>& se3Pose = it->second;   // the pose we'll refine
    
    SO3<> R = se3Pose.get_rotation();
    
    // Start iterative calculation of the rotation
    // Follows the algorithm of "Rotation Averaging with Application to Camera-Rig Calibration"
    // by Dai et. al. section "geodesic L2-mean"
    while(1)
    {
      Vector<3> r;
      r = Zeros;
      int n = 0;
      
      // Go through all MultiKeyFrames in the map
      for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
      {
        MultiKeyFrame& mkf = *(*mkf_it);
        
        if(mkf.mmpKeyFrames.count(camName))  // mkf contains the camera we're working on
        {
          n++;        
          r += (R.inverse() * mkf.mmpKeyFrames[camName]->mse3CamFromBase.get_rotation()).ln();  // incorporate KeyFrame's pose
        }
      }
      
      ROS_ASSERT(n > 0);
      
      r *= 1.0/n;
      
      if(r*r < dEpsilon*dEpsilon)  // converged, so get out of here
        break;
        
      R = R * SO3<>::exp(r);  // update R
    }
    
    se3Pose.get_rotation() = R;   // set the rotation
    
    // Do a similar thing for the translation vector, but its calculation is simpler since we can just average these vectors 
    Vector<3> v3Trans;
    v3Trans = Zeros;
    int n = 0;
    
    // Go through all MultiKeyFrames in the map
    for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
    {
      MultiKeyFrame& mkf = *(*mkf_it);
      
      if(mkf.mmpKeyFrames.count(camName))  // mkf contains the camera we're working on
      { 
        n++;
        v3Trans +=  mkf.mmpKeyFrames[camName]->mse3CamFromBase.get_translation();  // incorporate KeyFrame's pose
      }
    }
    
    ROS_ASSERT(n > 0);
    v3Trans *= 1.0/n;
    
    se3Pose.get_translation() = v3Trans;   // set the translation
  }
  
  return mPoses;
}

// Cleans up the map and gets it ready for calibration
bool MapMakerCalib::CalibInit()
{
  ROS_ASSERT(mbPaused); // The MapMaker thread was paused by an external call
  
  ROS_INFO("****************************************************************");
  ROS_INFO("***************** STARTING FINAL OPTIMIZATION ******************");
  ROS_INFO("****************************************************************");
  
  // Adjust the whole map one more time before loading things into BundleAdjusterCalib
  std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliers;
  
  mBundleAdjuster.UseTukey(true);
  mBundleAdjuster.UseTwoStep(false);
  
  int nAccepted = mBundleAdjuster.BundleAdjustAll(vOutliers);
  mdMaxCov = mBundleAdjuster.GetMaxCov();
  
  ROS_DEBUG_STREAM("Accepted iterations: "<<nAccepted);
  ROS_DEBUG_STREAM("Number of outliers: "<<vOutliers.size());
  ROS_DEBUG_STREAM("Max cov: "<<mdMaxCov);
  
  ROS_INFO("****************************************************************");
  ROS_INFO("***************** STARTING CALIB BUNDLE ADJUSTMENT *************");
  ROS_INFO("****************************************************************");
  
  ROS_INFO_STREAM("BEFORE RemoveMultiKeyFrames there are "<<mMap.mlpMultiKeyFrames.size()<<" MKFs and "<<mMap.mlpPoints.size()<<" points");
  std::string firstCamName = mmCameraModels.begin()->first;
  RemoveMultiKeyFrames(firstCamName, true);
  ROS_INFO_STREAM("AFTER RemoveMultiKeyFrames there are "<<mMap.mlpMultiKeyFrames.size()<<" MKFs and "<<mMap.mlpPoints.size()<<" points");
  
  SE3Map mRelativePoses = FindAverageRelativePoses();  
  
  if(mRelativePoses.empty())
  {
    ROS_ERROR("Couldn't find any MKFs that have all cameras! Can't do Calibration BA!");
    return false;
  }
  
  // Now we need to update all the KFs' mse3CamFromBase with the values from mRelativePoses, 
  // except for the first KF in each MKF, which should have the identity pose. However, just before
  // doing that, we want to update the base pose (mse3BaseFromWorld) of each MKF to distribute the error
  // between the new MKF/KF poses and the old poses more evenly. 
  //
  // The reasoning is this: if you just replace the mse3CamFromBase for each KF (other than the first one)
  // with a new value, you're introducing error into the reprojections in these KFs, but not in the first KF. 
  // Bundle adjustment works better if all the errors are evenly distributed, so we'll add an offset to 
  // mse3BaseFromWorld to make the errors evenly distributed. Note that this compensates for the pose error, not
  // the reprojection error. However, assuming all measurements are evenly distributed between all KFs, and evenly
  // over the images in all the KFs, then this step will help distribute the reprojection errors pretty well.
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
      
    TooN::SE3<> se3BaseShift;
    ROS_DEBUG_STREAM("Starting se3BaseShift calculation for MKF with "<<mkf.mmpKeyFrames.size()<<" KFs");
    for(int i=0; i < 10; ++i)  // ten iterations of Gauss-Newton should suffice
    {
      TooN::WLS<6> wls;
      wls.add_prior(1.0); // Stabilising prior
      
      double dErrorSum = 0;
      
      for(SE3Map::iterator se3_it = mRelativePoses.begin(); se3_it != mRelativePoses.end(); ++se3_it)
      {
        std::string camName = se3_it->first;
        TooN::SE3<> se3RelPose = se3_it->second;
        
        if(!mkf.mmpKeyFrames.count(camName))
          continue;
          
        /* The pose loop looks like this:
         * 
         *   KF2 ------ mse3CamFromBase ------> KF1
         *    |                                  |
         *    |                                  |
         * se3Error                         se3BaseShift
         *    |                                  |
         *    V                                  V
         *   KF2 --------- se3RelPose --------> KF1
         * 
         */

        TooN::SE3<> se3Error = mkf.mmpKeyFrames[camName]->mse3CamFromBase * se3BaseShift * se3RelPose.inverse();
        dErrorSum += (se3Error.ln() * se3Error.ln());
        TooN::SE3<> se3ErrorInBase = se3BaseShift * se3RelPose.inverse();
        
        
        // The partial derivative of a pose (in this case, the error pose) with respect to another pose (the base shift pose)
        // will be a 6x6 matrix. It is calculated similarly to the partial derivative of a point projection wrt to a pose
        // (see ChainBundle code), at least for the translation component of the error pose. For the rotation component of the 
        // error pose, the partials wrt the translation of the base shift pose will be zero, and wrt to the rotation component
        // of the base shift pose it is derived from the calculus of SO3 manifold. In this case, it is simply the rotation 
        // component of se3RelPose.
        
        TooN::Matrix<6> m6Jac = TooN::Zeros;
        const TooN::Vector<4> v4Base = TooN::unproject(se3ErrorInBase.get_translation());
        // For each of six degrees of freedom...
        for(int m=0;m<6;m++)
        {
          // Get the motion of the point in the base frame's pose when the pose changes by one of the degrees of freedom
          const TooN::Vector<4> v4Motion_Base = TooN::SE3<>::generator_field(m, v4Base); 
          // Then the motion of the point in another frame depends only on the relative rotation between it and the base frame, not the translation
          TooN::SE3<> se3CamFromBase_onlyrot = mkf.mmpKeyFrames[camName]->mse3CamFromBase;
          se3CamFromBase_onlyrot.get_translation() = TooN::Zeros;
          
          const TooN::Vector<4> v4Motion_Cam = se3CamFromBase_onlyrot * v4Motion_Base;  // CHECK!! GOOD
          
          // Insert into the top half of the 6x6 matrix
          m6Jac.slice(0,m,3,1) = v4Motion_Cam.slice<0,3>().as_col();
        }
      
        // Insert into the bottom right
        m6Jac.slice<3,3,3,3>() = se3RelPose.get_rotation().get_matrix();
          
        // Add errors and derivatives to weighted lease squares
        TooN::Vector<6> v6Error = se3Error.ln();
        for(int m=0;m<6;m++)
        {
          wls.add_mJ(v6Error[m], m6Jac[m], 1);
        }
      }
      
      wls.compute();
      TooN::Vector<6> v6Update = wls.get_mu();
      se3BaseShift = (SE3<>::exp(v6Update)).inverse() * se3BaseShift;
    }
    
    ROS_DEBUG_STREAM("Final se3BaseShift for MKF: "<<se3BaseShift);
    mkf.mse3BaseFromWorld = se3BaseShift.inverse() * mkf.mse3BaseFromWorld;
    
    // Now update the KeyFrames' CamFromWorld and CamFromBase variables
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      std::string camName = kf_it->first;
      
      kf.mse3CamFromBase = mRelativePoses[camName];
      kf.mse3CamFromWorld = kf.mse3CamFromBase * mkf.mse3BaseFromWorld;
    }
  }
  
  meCalibState = CALIB_INITIALIZED;
  return true;
}

//Take one step with the BundleAdjusterCalib adjuster (actually, 10 LM steps)
bool MapMakerCalib::CalibOneStep()
{
  ROS_ASSERT(meCalibState == CALIB_INITIALIZED || meCalibState == CALIB_RUNNING);
  
  BundleAdjusterCalib adjusterCalib(mMap, mmCameraModels, true);
  adjusterCalib.UseTukey(true);
  
  if(meCalibState == CALIB_INITIALIZED)
  {
    adjusterCalib.UseRobust(false);  // do one round without robustification
    meCalibState = CALIB_RUNNING;
  }
  else
  {
    adjusterCalib.UseRobust(true);
  }
  
  std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliers;
  
  int nAccepted = adjusterCalib.BundleAdjustAll(vOutliers);
  mdMaxCov = mBundleAdjuster.GetMaxCov();
  
  ROS_DEBUG_STREAM("Accepted iterations: "<<nAccepted);
  ROS_DEBUG_STREAM("Number of outliers: "<<vOutliers.size());
  ROS_DEBUG_STREAM("Max cov: "<<mdMaxCov);
  
  mmFinalPoses = adjusterCalib.mmFinalPoses;
  mdSigmaSquared = adjusterCalib.GetSigmaSquared();
  mdMeanChiSquared = adjusterCalib.GetMeanChiSquared();
  
  HandleOutliers(vOutliers);
  
  return nAccepted >= 0;
}

// Pause the map making optimization (not the calibration optimization)
void MapMakerCalib::PauseRun()
{ 
  mnSavedRunState = (int)mMap.mbGood;
  mMap.mbGood = false; 
  
  if(mBundleAdjuster.Running())  
    mBundleAdjuster.RequestAbort();
    
  while(mBundleAdjuster.Running())
  {
    ROS_INFO("Sleeping and waiting for Bundle Adjuster to stop");
    ros::Duration(0.5).sleep();  
  }
  
  // Sleep a bit more to allow outlier handling and map publication to finish.
  // It would be nice to have a more reliable way of making sure the map making
  // thread is done making modifications to the map, but this'll work
  ros::Duration(1.0).sleep(); 
  
  ROS_INFO("Bundle Adjuster has stopped, we are now paused");
  mbPaused = true;
}

// Resume the map making optimization (not the calibration optimization)
void MapMakerCalib::ResumeRun()
{
  if(mnSavedRunState != -1)
  {
    mMap.mbGood = (bool)mnSavedRunState;
    mbPaused = false;
    meCalibState = CALIB_INVALID;
    mnSavedRunState = -1;
    ROS_INFO("And we're back!");
  }
}
