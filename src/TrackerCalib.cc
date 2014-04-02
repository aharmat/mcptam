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
// Some parts of this code are from the original PTAM which is
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/TrackerCalib.h>
#include <mcptam/OpenGL.h>
#include <mcptam/CameraCalibrator.h>
#include <mcptam/MEstimator.h>
#include <mcptam/ShiTomasi.h>
#include <mcptam/SmallMatrixOpts.h>
#include <mcptam/PatchFinder.h>
#include <mcptam/TrackerData.h>
#include <mcptam/CalibImageTaylor.h>
#include <mcptam/MapMakerClientBase.h>
#include <mcptam/Map.h>
#include <mcptam/ChainBundle.h> 

#include <gvars3/instances.h>

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <cvd/convolution.h>

#include <TooN/wls.h>
#include <TooN/SVD.h>

using namespace TooN;
using namespace GVars3;

TrackerCalib::TrackerCalib(Map &map, MapMakerClientBase& mapMaker, TaylorCameraMap cameras, std::string cameraName, CVD::ImageRef irOffset, CVD::ImageRef irPatternSize, double dSquareSize, GLWindow2* pWindow)
    // Tracker wants pose and draw offset maps, but we only have one camera so those don't really make sense. Generate some default values for it.
  : Tracker(map, mapMaker, cameras, GetInitPoseMap(cameraName), GetInitOffsetMap(cameraName, irOffset), pWindow)
  , mirPatternSize(irPatternSize)
  , mdSquareSize(dSquareSize)
  , mCamName(cameraName)
{
  mpCalibImage = new CalibImageTaylor(irOffset, mpGLWindow);
  Reset(false);
  
  v4FirstStageColor = makeVector(0,0,1,1);  // blue
  v4SecondStageColor = makeVector(0,0.75,1,1);  // light blue
  v4RunningColor = makeVector(0,1,0,1);  // green
  v4LostColor = makeVector(1,0,0,1);   // red
  v4InactiveColor = makeVector(0,0,0.5,1); // dark blue
}

// Calls parent's Reset and resets some internal variables
void TrackerCalib::Reset(bool bResetMap)
{
  ROS_INFO("TrackerCalib: Calling reset on underlying tracker");
  Tracker::Reset(false, bResetMap);  // the map will be reset multiple times if there are multiple CalibratorTrackers, but we'll live with it
  
  // Get a pointer to the only KF in the underlying Tracker's MKF to make it easier to work with
  mpCurrentKF = mpCurrentMKF->mmpKeyFrames[mCamName];
  
  meCheckerboardStage = CHECKERBOARD_INACTIVE;
  mbNeedToDrop = false;
}

// Draw a border of given thickness and color
void TrackerCalib::DrawBorder(CVD::ImageRef irOffset, int nThickness, const TooN::Vector<-1>& vColor)
{
  if(nThickness < 1)
    nThickness = 1;
  
  CVD::ImageRef irSize = mpCurrentKF->maLevels[0].image.size();
  CVD::ImageRef irTopLeft, irBottomRight;
  
  CVD::glColor(vColor);
  
  // Four parts, top, bottom, left, right
  // Draw top bar
  irTopLeft = CVD::ImageRef(0, 0);
  irBottomRight = CVD::ImageRef(irSize.x, nThickness-1);
  CVD::glRect(irOffset+irTopLeft, irOffset+irBottomRight);
  
  // Draw bottom bar
  irTopLeft = CVD::ImageRef(0, irSize.y-(nThickness-1));
  irBottomRight = CVD::ImageRef(irSize.x, irSize.y);
  CVD::glRect(irOffset+irTopLeft, irOffset+irBottomRight);
  
  // Draw left bar
  irTopLeft = CVD::ImageRef(0, 0);
  irBottomRight = CVD::ImageRef(nThickness-1, irSize.y);
  CVD::glRect(irOffset+irTopLeft, irOffset+irBottomRight);
  
  // Draw right bar
  irTopLeft = CVD::ImageRef(irSize.x-(nThickness-1), 0);
  irBottomRight = CVD::ImageRef(irSize.x, irSize.y);
  CVD::glRect(irOffset+irTopLeft, irOffset+irBottomRight);
  
}

// Draw a border around the camera image indicating status (ie running, initializing, lost)
void TrackerCalib::DrawStatusBorder()
{
  CVD::ImageRef irOffset = mmDrawOffsets[mCamName];
  
  if(meCheckerboardStage == CHECKERBOARD_FIRST_STAGE)  // currently finding checkerboard
  {
    DrawBorder(irOffset, 5, v4FirstStageColor); // 5 pixel thick blue border
  }
  else if(meCheckerboardStage == CHECKERBOARD_SECOND_STAGE)  // currently finding checkerboard
  {
    DrawBorder(irOffset, 5, v4SecondStageColor); // 5 pixel thick light blue border
  }
  else if(meCheckerboardStage == CHECKERBOARD_RUNNING)  // running
  {
    if(IsLost())
    {
      DrawBorder(irOffset, 5, v4LostColor); // 5 pixel thick red border
    }
    else
    {
      DrawBorder(irOffset, 5, v4RunningColor); // 5 pixel thick green border
    }
  }
  else  // not yet active in any way
  {
    DrawBorder(irOffset, 5, v4InactiveColor); // 5 pixel thick dark blue border
  }
  
}

// Optimize the pose computed by the calibration image using bundle adjustment
void TrackerCalib::OptimizeCalibImagePose()
{
  ChainBundle tempBundle(mmCameraModels, false, false, false);

  int nWorldID = tempBundle.AddPose(TooN::SE3<>(), true);  // dummy pose so we can insert points fixed in the world frame
  int nCamID = tempBundle.AddPose(mpCalibImage->mse3CamFromWorld, false);
  
  for(unsigned i=0; i < mpCalibImage->mvGridCorners.size(); ++i)
  {
    TooN::Vector<3> v3WorldPos;
    v3WorldPos.slice<0,2>() = CVD::vec(mpCalibImage->mvGridCorners[i].mirGridPos);
    v3WorldPos[2] = 0.0;
    
    std::vector<int> vPoses(1);
    vPoses[0] = nWorldID;
    int nPointID = tempBundle.AddPoint(v3WorldPos, vPoses, true);
    
    std::vector<int> vCams(1);
    vCams[0] = nCamID;
    tempBundle.AddMeas(vCams, nPointID, mpCalibImage->mvGridCorners[i].mParams.v2Pos, 1, mCamName);
  }
  
  bool bAbort = false;
  int nAccepted = tempBundle.Compute(&bAbort);
  
  if(nAccepted >= 0)
    mpCalibImage->mse3CamFromWorld = tempBundle.GetPose(nCamID);
    
}

// Finish computing the internal pose of the calibration image using the known parameters of our camera
void TrackerCalib::FinishCalibImagePose()
{
  TaylorCamera& camera = mmCameraModels[mCamName];
  TooN::Vector<2> v2Center = camera.GetCenter();
  
  mpCalibImage->GuessInitialPose(v2Center);  // Update internal pose, make sure this is called before building big matrix!
  
  int nPoints = mpCalibImage->mvGridCorners.size();
  int nViews = 1;
  
  // This follows Scaramuzza thesis section 3.2.2
  TooN::Matrix<> M(2*nPoints, 4+nViews);
  TooN::Vector<> b(2*nPoints);
  
  int nIdx = 0;
  
  std::pair< TooN::Matrix<>, TooN::Vector<> > MandB = mpCalibImage->BuildIntrinsicMatrixEntries(v2Center, 4, 0, nViews);
  int nNumEntries = MandB.second.size();
  
  // Insert into big matrix
  M.slice(nIdx,0,nNumEntries,4+nViews) = MandB.first;
  b.slice(nIdx,nNumEntries) = MandB.second;
  
  nIdx += nNumEntries;
  
  
  TooN::Vector<4> v4Poly = camera.GetParams().slice<0,4>();
  TooN::Matrix<> M_param = M.slice(0,0,2*nPoints,4);
  TooN::Vector<> b_mod = b -  M_param * v4Poly;
  
  TooN::Matrix<> M_t3 = M.slice(0,4,2*nPoints,1);
  TooN::SVD<> svd(M_t3);
  
  //std::cout<<"M_t3: "<<std::endl<<M_t3<<std::endl;
  //std::cout<<"b_mod: "<<std::endl<<b_mod<<std::endl;
  
  // Some camera parameters are very small (ie in the affine matrix), to handle this correctly we need to override SVD's condition number
  double dConditionNumber = 1/(M_t3.num_rows() * std::numeric_limits<double>::epsilon());
  TooN::Vector<> vResult = svd.backsub(b_mod, dConditionNumber);
  
  mpCalibImage->mse3CamFromWorld.get_translation()[2] = vResult[0];
  if(vResult[0] < 0)
  {
    ROS_ERROR_STREAM("Z component of calib image is "<<vResult[0]);
  }
  
  //TooN::Vector<> vxResidual = M_t3 * vResult - b_mod;
  //std::cout<<"%%%%%%% residual error: "<<sqrt(vxResidual * vxResidual)<<std::endl;

}

// TrackFrame is called by PoseCalibrator with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
void TrackerCalib::TrackFrame(CVD::Image<CVD::byte>& imFrame, ros::Time timestamp, bool bDraw, bool bFindCheckerboard)
{
  ROS_ASSERT(mbNeedToDrop == false);  // otherwise there was an unhandled request to drop a new keyframe last time, bad
  
  CVD::convolveGaussian(imFrame, 1.0);
  
  ImageBWMap imageMap;  // need to put image into a map to call base class TrackFrameSetup
  imageMap.insert(make_pair(mCamName, imFrame));
  TrackFrameSetup(imageMap, timestamp, bDraw);
  
  // Decide what to do - if there is a map and initialized with checkerboard, try to track the map ...
  if(mMap.mbGood && (meCheckerboardStage == CHECKERBOARD_RUNNING || meCheckerboardStage == CHECKERBOARD_SECOND_STAGE))
  {
    // In the second stage of checkerboard finding, the checkerboard location will help the tracker by updating the 
    // MKF pose before tracking of map points even begins.
    if(meCheckerboardStage == CHECKERBOARD_SECOND_STAGE)
    {
      bool bGood = mpCalibImage->MakeFromImage(imFrame, mirPatternSize);  // fill calib image data
      if(bGood) 
      {
        FinishCalibImagePose();
        
        TaylorCamera& camera = mmCameraModels[mCamName];
        mpCalibImage->Draw3DGrid(camera, true);
        
        OptimizeCalibImagePose();
      
        mpCurrentMKF->mse3BaseFromWorld = mpCalibImage->mse3CamFromWorld;
        mpCurrentMKF->mse3BaseFromWorld.get_translation() *= mdSquareSize;
        UpdateCamsFromWorld();
      }
    }
    
    if(!IsLost())  // .. but only if we're not lost!
    {
      // Only apply the motion model if we're running. If in second stage, the checkerboard finding will have 
      // updated the MKF pose already, so applying the motion model would just ruin that
      if(meCheckerboardStage == CHECKERBOARD_RUNNING) 
        ApplyMotionModel();       
        
      TrackMap();    // The main work of tracking the map
      
      if(meCheckerboardStage == CHECKERBOARD_RUNNING) 
        UpdateMotionModel();     
      
      AssessOverallTrackingQuality();  //  Check if we're lost or if tracking is poor.
            
      { // Provide some feedback for the user:
        mMessageForUser << "Tracker "<<mCamName<<": quality ";
        if(mOverallTrackingQuality == GOOD)  mMessageForUser << "good.";
        if(mOverallTrackingQuality == DODGY) mMessageForUser << "poor.";
        if(mOverallTrackingQuality == BAD)   mMessageForUser << "bad.";
        
        mMessageForUser << "  Scene depth: " << mpCurrentMKF->mdTotalDepthMean<<std::endl;
        mMessageForUser << "Found: ";
        
        for(int i=0; i<LEVELS; i++) 
          mMessageForUser << " " << (mmMeasFoundLevels[mCamName])[i] << "/" << (mmMeasAttemptedLevels[mCamName])[i];
            
        mMessageForUser<<std::endl;
        mMessageForUser << "Map: " << mMap.mlpPoints.size() << "P, " << mMap.mlpMultiKeyFrames.size() << "MKF, " << mMap.mlpPointsTrash.size()<<"P in Trash" ;
      }
      
      static gvar3<int> gvnAddingMKFs("AddingMKFs", 1, HIDDEN|SILENT);
      // Heuristics to check if a key-frame should be added to the map:
      if(*gvnAddingMKFs &&
         mOverallTrackingQuality == GOOD &&
         mnLostFrames == 0 &&
         ros::Time::now() - mtLastMultiKeyFrameDropped > ros::Duration(1.0)  &&
         mMapMaker.NeedNewKeyFrame(*mpCurrentKF, true))
      {
        mMessageForUser << " SHOULD BE Adding MultiKeyFrame to Map";
        RecordMeasurements(); // We've decided to add, so make measurements
        SignalNewKeyFrame();  // Make our desire known
      }
      
      ReleasePointLock();  // Important! Do this whenever tracking step has finished
      
    }
    else  // what if there is a map, but tracking has been lost?
    {
      mMessageForUser << "** Attempting recovery **.";
      if(AttemptRecovery())
      {
        TrackMap();
        AssessOverallTrackingQuality();
        ReleasePointLock();  // Important! Do this whenever tracking step has finished
      }
    }
    
    if(mbDraw)
      RenderGrid();
  } 
  // If checkerboard has not yet been found but it's our turn to find checkerboard
  else if((meCheckerboardStage == CHECKERBOARD_INACTIVE || meCheckerboardStage == CHECKERBOARD_FIRST_STAGE) && bFindCheckerboard) 
  {
    meCheckerboardStage = CHECKERBOARD_FIRST_STAGE;
    bool bGood = mpCalibImage->MakeFromImage(imFrame, mirPatternSize);  // fill calib image data
    if(bGood && mbInitRequested) 
    {
      FinishCalibImagePose();
      
      ROS_INFO_STREAM("TrackerCalib: "<<mCamName<<" just initialized with checkerboard!");
      
      if(!mMap.mbGood)  // map has not been properly initialized yet with calibration pattern points
      {
        meCheckerboardStage = CHECKERBOARD_SECOND_STAGE;
        SignalNewKeyFrame();
      }
      else
      {
        OptimizeCalibImagePose();
        
        mpCurrentMKF->mse3BaseFromWorld = mpCalibImage->mse3CamFromWorld;
        mpCurrentMKF->mse3BaseFromWorld.get_translation() *= mdSquareSize;
        UpdateCamsFromWorld();
        
        TrackMap();
        AssessOverallTrackingQuality();  //  Check if we're lost or if tracking is poor.
        
        ROS_INFO_STREAM("TrackerCalib: "<<mCamName<<" quality after checkerboard: "<<mOverallTrackingQuality);
        
        if(mOverallTrackingQuality == GOOD)
        {
          RecordMeasurements(); // We've decided to add, so make measurements
          SignalNewKeyFrame();  // Make our desire known
        
          meCheckerboardStage = CHECKERBOARD_RUNNING;  // don't need to do second stage if there's already a map
        }
        else
        {
          ROS_INFO_STREAM("TrackerCalib: "<<mCamName<<" quality bad after checkerboard!");
        }
        
        ReleasePointLock();  // Important! Do this whenever tracking step has finished
      }
      
      mbJustRecoveredSoUseCoarse = true;
      mOverallTrackingQuality = GOOD;
    } 
  }
  
  if(meCheckerboardStage == CHECKERBOARD_FIRST_STAGE && !bFindCheckerboard)  // we are no longer trying to find checkerboard 
    ROS_BREAK(); // should never happen
    
  if(mbDraw)
    DrawStatusBorder();
  
  // Do this here instead of after testing for mbInitRequested because only the TrackerCalib (in PoseCalibrator's posession) that 
  // has been asked to find the checkerboard will act on mbInitRequested, but all of them will have mbInitRequested set to true
  // when the button/spacebar is pressed.
  mbInitRequested = false;
}

// Set the need-to-drop flag to true
void TrackerCalib::SignalNewKeyFrame()
{
  mbNeedToDrop = true;
}

// Set the need-to-drop flag to false and regenerate our current MultiKeyFrame because it has been taken by the Map
void TrackerCalib::MarkKeyFrameAdded(KeyFrame& kfSrc)
{
  // Abuse of notation, since we're dropping KeyFrames (not MultiKeyFrames) but no point 
  // redefining a new variable that does exactly the same thing
  mnLastMultiKeyFrameDropped = mnFrame;
  mtLastMultiKeyFrameDropped = ros::Time::now();
  mbNeedToDrop = false;
  
  InitCurrentMKF(kfSrc.mse3CamFromWorld);  // When PoseCalibrator took the KeyFrame to add it to map, it destroyed currentMKF so regenerate it
  mpCurrentKF = mpCurrentMKF->mmpKeyFrames[mCamName];  // re-link pointer
  
  // Copy masks
  for(int i=0; i<LEVELS; i++)
    mpCurrentKF->maLevels[i].mask = kfSrc.maLevels[i].mask;  // use CVD's reference counting system to avoid deep copy
  
  // Copy depth
  mpCurrentKF->mdSceneDepthMean = kfSrc.mdSceneDepthMean;
  mpCurrentKF->mdSceneDepthSigma = kfSrc.mdSceneDepthSigma;
  
  if(meCheckerboardStage == CHECKERBOARD_SECOND_STAGE)
    meCheckerboardStage = CHECKERBOARD_RUNNING;
  
}

// Set the need-to-drop flag to false
void TrackerCalib::MarkKeyFrameAdded()
{
  // Abuse of notation, since we're dropping KeyFrames (not MultiKeyFrames) but no point 
  // redefining a new variable that does exactly the same thing
  mnLastMultiKeyFrameDropped = mnFrame;
  mtLastMultiKeyFrameDropped = ros::Time::now();
  mbNeedToDrop = false;
}
