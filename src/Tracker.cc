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

#include <mcptam/Tracker.h>
#include <mcptam/OpenGL.h>
#include <mcptam/MEstimator.h>
#include <mcptam/SmallMatrixOpts.h>
#include <mcptam/PatchFinder.h>
#include <mcptam/TrackerData.h>
#include <mcptam/MapMakerClientBase.h>
#include <mcptam/MapPoint.h>
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <mcptam/TrackerState.h>

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <TooN/SVD.h>

#include <gvars3/instances.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//debug
#include <tf/transform_broadcaster.h>

using namespace TooN;
using namespace GVars3;

// Static members
double Tracker::sdRotationEstimatorBlur = 0.75;
bool Tracker::sbUseRotationEstimator = true;
bool Tracker::sbDrawFASTCorners = true;
int Tracker::snMaxPatchesPerFrame = 1000;
int Tracker::snMinPatchesPerFrame = 10;
int Tracker::snCoarseMin = 15;              
int Tracker::snCoarseMax = 60;              
int Tracker::snCoarseRange = 30;            
int Tracker::snCoarseSubPixIts = 8;        
bool Tracker::sbDisableCoarse = false;        
double Tracker::sdCoarseMinVelocity = 0.006;   
std::string Tracker::sMEstimatorName = "Tukey";
double Tracker::sdTrackingQualityGood = 0.3;
double Tracker::sdTrackingQualityBad = 0.13;
int Tracker::snLostFrameThresh = 3;
bool Tracker::sbCollectAllPoints = true;

// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(Map &map, RelocaliserFabMap &reloc, MapMakerClientBase &mapmaker, TaylorCameraMap &cameras, SE3Map poses, ImageRefMap offsets, GLWindow2* pWindow) 
  : mpCurrentMKF(NULL)
  , mMap(map)
  , mRelocFabMap(reloc)
  , mMapMaker(mapmaker)
  , mRelocaliser(mMap, cameras)
  , mmFixedPoses(poses)
  , mNodeHandlePrivate("~")
  , mpGLWindow(pWindow)
{
  ROS_DEBUG("Tracker: In constructor");
  
  mmCameraModels = cameras;
  mmCameraModelsSBI = mmCameraModels;  // the SBI version will be used with the frame-to-frame rotation detector which needs to resize the camera image internally
  mmDrawOffsets = offsets;
  
  // Save camera names and image sizes
  for(SE3Map::iterator se3_it = mmFixedPoses.begin(); se3_it != mmFixedPoses.end(); ++se3_it)
  {
    std::string camName = se3_it->first;
    mmSizes[camName] = mmCameraModels[camName].GetImageSize();
    mvAllCamNames.push_back(camName);
  }
  
  // Most of the initialisation is done in Reset()
  ROS_DEBUG("Tracker: In constructor, calling Reset");
  Reset(false, true);
  ROS_DEBUG("Tracker: In constructor, done");
  
  maskPub = mNodeHandlePrivate.advertise<sensor_msgs::Image>("mask", 1);
  timingPub = mNodeHandlePrivate.advertise<mcptam::TrackerTiming>("timing_tracker", 1);
}

Tracker::~Tracker()
{
  if(mpCurrentMKF)
    delete mpCurrentMKF;
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset(bool bSavePose, bool bResetMap)
{
  mbDidCoarse = false;
  mOverallTrackingQuality = NONE;
  mnLostFrames = 0;
  mnLastMultiKeyFrameDropped = -20;
  mtLastMultiKeyFrameDropped = ros::Time(ros::Time::now().toSec() - 1.0);
  mnFrame=0;
  mv6BaseVelocity = Zeros;
  mdMSDScaledVelocityMagnitude = 0;
  mbJustRecoveredSoUseCoarse = true;
  mbInitRequested = false;
  mbPutPlaneAtOrigin = true;
  mbAddNext = false;
  mbForceRecovery = false;
  mmSimpleMeas.clear();
   
  mLastProcessTime = ros::Time::now();
  mse3StartPose = SE3<>();  

  if(mpCurrentMKF)
  {
    if(bSavePose)
      mse3StartPose = mpCurrentMKF->mse3BaseFromWorld;
      
    mpCurrentMKF = NULL;
  }
  
  // Regenerate current MultiKeyFrame
  InitCurrentMKF(mse3StartPose);
  
  for(unsigned i=0; i < mvAllCamNames.size(); ++i)
  {
    std::string camName = mvAllCamNames[i];
    
    mmpSBILastFrame[camName] = NULL;
    mmpSBIThisFrame[camName] = NULL;
  }
  
  SetMasks(mmMasks);
  
  // Tell the MapMaker to reset itself.. 
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.
  if(bResetMap)
  {
    ROS_INFO("Tracker: Requesting map reset");
    mMapMaker.RequestReset();
    
    // THIS MIGHT HAVE PROBLEMS IN NETWORK MODE!
    while(!mMapMaker.ResetDone() && ros::ok())
    {
      ROS_INFO("Tracker: Trying to reset map...");
      ros::Duration(0.5).sleep();
    }
    
  }
}

void Tracker::RequestInit(bool bPutPlaneAtOrigin)
{
  mbInitRequested = true;
  mbPutPlaneAtOrigin = bPutPlaneAtOrigin;
}

// Will add next MKF if not lost
void Tracker::AddNext()
{ 
  if(mMap.mbGood && !IsLost())
    mbAddNext = true; 
}

void Tracker::ForceRecovery()
{ 
  mbForceRecovery = true;
}

// Generate a new MultiKeyFrame with a given pose and its children KeyFrames with the fixed camera poses
void Tracker::InitCurrentMKF(const SE3<>& pose)
{
  ROS_ASSERT(mpCurrentMKF == NULL);
  
  mpCurrentMKF = new MultiKeyFrame;
  mpCurrentMKF->mse3BaseFromWorld = pose;
  
  for(unsigned i=0; i < mvAllCamNames.size(); ++i)
  {
    std::string camName = mvAllCamNames[i];
    KeyFrame* pKF = new KeyFrame(mpCurrentMKF, camName);
    pKF->mse3CamFromBase = mmFixedPoses[camName];
    mpCurrentMKF->mmpKeyFrames[camName] = pKF;
  }
  
  mpCurrentMKF->UpdateCamsFromWorld();
}

// Set the feature extraction masks for the camera images
void Tracker::SetMasks(ImageBWMap& mMasks)
{
  mmMasks = mMasks;  
  
  for(KeyFramePtrMap::iterator kf_it = mpCurrentMKF->mmpKeyFrames.begin(); kf_it != mpCurrentMKF->mmpKeyFrames.end(); ++kf_it)
  {
    if(mmMasks.count(kf_it->first)) // we've got a mask for this keyframe
    {
      CVD::Image<CVD::byte> imMask = mmMasks[kf_it->first];  // not deep copy
      ROS_ASSERT(imMask.size() == mmSizes[kf_it->first]); // the LoadMask function should have resized the masks
      kf_it->second->SetMask(imMask);
    }
  }
}

// Copy masks from the given MKF to the current MKF, used when map maker takes our MKF and we need to regenerate our own masks
void Tracker::CopyMasks(MultiKeyFrame& mkf)
{
  for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
  {
    std::string camName = kf_it->first;
    KeyFrame& kfCurr = *(mpCurrentMKF->mmpKeyFrames[camName]);
    KeyFrame& kfOther = *(kf_it->second);
    for(int i=0; i<LEVELS; i++)
    {
      kfCurr.maLevels[i].mask = kfOther.maLevels[i].mask;  // use CVD's reference counting system to avoid deep copy
    }
  }
}

// Get the current poses of the cameras
SE3Map Tracker::GetCurrentCameraPoses()
{
  SE3Map mPoses;
  for(KeyFramePtrMap::iterator kf_it = mpCurrentMKF->mmpKeyFrames.begin(); kf_it != mpCurrentMKF->mmpKeyFrames.end(); ++kf_it)
  {
    mPoses.insert(std::pair<std::string, TooN::SE3<> >(kf_it->first, kf_it->second->mse3CamFromWorld));
  }
  
  return mPoses;
}

// Helper function that updates the current MultiKeyFrame and optionally draws the camera images
void Tracker::TrackFrameSetup(ImageBWMap& imFrames, ros::Time timestamp, bool bDraw)
{
  mLastProcessDur = timestamp - mLastProcessTime;
  mLastProcessTime = timestamp;
  
  mbDraw = bDraw;
  mnFrame++;
  
  mMessageForUser.str("");   // Wipe the user message clean
  
  // First clear all the keyframe measurements
  mpCurrentMKF->EraseBackLinksFromPoints();  // In case
  mpCurrentMKF->ClearMeasurements();
  for(KeyFramePtrMap::iterator kf_it = mpCurrentMKF->mmpKeyFrames.begin(); kf_it != mpCurrentMKF->mmpKeyFrames.end(); ++kf_it)
  {
    kf_it->second->mbActive = false;  // Will be reactivated depending on which images we receive.
  }
  
  mvCurrCamNames.clear();
  
  timingMsg.kf_downsample = 0;
  timingMsg.kf_mask = 0;
  timingMsg.kf_feature = 0;
  timingMsg.sbi = 0;
  ros::WallTime startTime;
  
  static gvar3<int> gvnGlareMasking("GlareMasking", 0, HIDDEN|SILENT);
  
  // Go through all received images, and convert them into the tracker's KeyFrames
  // This does things like generate the image pyramid and find FAST corners
  for(ImageBWMap::iterator img_it = imFrames.begin(); img_it != imFrames.end(); ++img_it)
  {
    std::string camName = img_it->first;
    mvCurrCamNames.push_back(camName); 
    
    std::tuple<double,double,double> kfTimes;
    
    kfTimes = mpCurrentMKF->mmpKeyFrames[camName]->MakeKeyFrame_Lite(img_it->second, false, *gvnGlareMasking);  // deep copy,  glare masking
      
    timingMsg.kf_downsample += std::get<0>(kfTimes);
    timingMsg.kf_mask += std::get<1>(kfTimes);
    timingMsg.kf_feature += std::get<2>(kfTimes);
      
    mpCurrentMKF->mmpKeyFrames[camName]->mbActive = true;
    startTime = ros::WallTime::now();
    
    if(mmpSBIThisFrame[camName] == NULL) // if we don't have an SBI entry here
    {
      // Means this is the first time here, so make both this frame and last frame's SBI's the same
      mmpSBIThisFrame[camName] = new SmallBlurryImage(*mpCurrentMKF->mmpKeyFrames[camName], Tracker::sdRotationEstimatorBlur);
      mmpSBILastFrame[camName] = new SmallBlurryImage(*mpCurrentMKF->mmpKeyFrames[camName], Tracker::sdRotationEstimatorBlur);
    }
    else  //otherwise both this and last frames have SBI
    {
      delete  mmpSBILastFrame[camName];
      mmpSBILastFrame[camName] = mmpSBIThisFrame[camName];  // transfer to last frame
      mmpSBIThisFrame[camName] = new SmallBlurryImage(*mpCurrentMKF->mmpKeyFrames[camName], Tracker::sdRotationEstimatorBlur);
    }
    
    timingMsg.sbi += (ros::WallTime::now() - startTime).toSec();
  }
  

  static gvar3<int> gvnDrawLevel("DrawLevel", 0, HIDDEN|SILENT);
  static gvar3<int> gvnDrawMasks("DrawMasks", 0, HIDDEN|SILENT);
  
  double dMaxPointCov = mMapMaker.GetMaxCov();
  
  if(mbDraw)  // Draw the camera images and optionally the FAST corners
  {
    ROS_ASSERT(mpGLWindow);
    
    for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
    {
      std::string camName = mvCurrCamNames[i];
      CVD::ImageRef irSize = mmSizes[camName];
      CVD::ImageRef irOffset = mmDrawOffsets[camName];
      KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[camName];
      
      CVD::Image<CVD::byte> imDraw;
      if(*gvnDrawMasks && kf.maLevels[*gvnDrawLevel].lastMask.totalsize() > 0)
        imDraw = kf.maLevels[*gvnDrawLevel].lastMask;
      else
        imDraw = kf.maLevels[*gvnDrawLevel].image;
        
      if(imDraw.size() != irSize)  // only do resizing if necessary
      {
        CVD::Image<CVD::byte> imDrawResized(irSize);
        cv::Mat imDrawWrapped(imDraw.size().y, imDraw.size().x, CV_8U, imDraw.data(), imDraw.row_stride());
        cv::Mat imDrawResizedWrapped(imDrawResized.size().y, imDrawResized.size().x, CV_8U, imDrawResized.data(), imDrawResized.row_stride());
        cv::resize(imDrawWrapped, imDrawResizedWrapped, imDrawResizedWrapped.size(), 0, 0, cv::INTER_LINEAR);
        imDraw = imDrawResized;
      }
        
      glRasterPos(irOffset);
      glDrawPixels(imDraw);
        
      if(Tracker::sbDrawFASTCorners)
      {
        glColor3f(1,0,1);  
        glPointSize(LevelScale(*gvnDrawLevel)); 
        glBegin(GL_POINTS);
        
        for(unsigned int j=0; j<kf.maLevels[*gvnDrawLevel].vCorners.size(); ++j) 
        {
          CVD::ImageRef irLevelZero = CVD::ir_rounded(LevelZeroPos(kf.maLevels[*gvnDrawLevel].vCorners[j], *gvnDrawLevel));
          CVD::glVertex(irLevelZero + irOffset);
        }
          
        glEnd();
      }
      
      glColor3f(0,1.0,0);
      mpGLWindow->PrintString(irOffset + CVD::ImageRef(10,20), kf.mCamName, 15);
    }
    
    if(dMaxPointCov > 0)
    {
      double dRedFrac = dMaxPointCov/10;
      if(dRedFrac > 1)
        dRedFrac = 1.0;
      double dGreenFac = 1-dRedFrac;
      std::stringstream sscov;
      sscov<<"Max cov: "<<dMaxPointCov;
      std::string covstring = sscov.str();
    
      glColor3f(dRedFrac,dGreenFac,0);
      mpGLWindow->PrintString(CVD::ImageRef(10,80), covstring, 15);
    }
  }
  
}

// TrackFrame is called by System with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
void Tracker::TrackFrame(ImageBWMap& imFrames, ros::Time timestamp, bool bDraw)
{
  ros::WallTime startTime;
  ros::WallTime startTimeTotal = ros::WallTime::now();
  
  //debug
  /*
  static gvar3<int> gvnDrawReloc("DrawReloc", 0, HIDDEN|SILENT);
  if(*gvnDrawReloc && bDraw)
    bDraw = false;
  else if(*gvnDrawReloc && !bDraw)
    *gvnDrawReloc = 0;
  */
  
  TrackFrameSetup(imFrames, timestamp, bDraw);
  
  //debug
  /*
  TooN::SE3<> se3Saved = mpCurrentMKF->mse3BaseFromWorld;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  if(mRelocFabMap.FindBestPose(*mpCurrentMKF, *gvnDrawReloc))
  {
    tf::poseMsgToTF(util::SE3ToPoseMsg(GetCurrentPose().inverse()), transform);
    br.sendTransform(tf::StampedTransform(transform, GetCurrentTimestamp(), "vision_world", "vision_pose_fabmap_recover"));
  
    //restore
    mpCurrentMKF->mse3BaseFromWorld = se3Saved;
    mpCurrentMKF->UpdateCamsFromWorld();
  }
  */
  /*
  if(AttemptRecovery())
  {
	  tf::poseMsgToTF(util::SE3ToPoseMsg(GetCurrentPose().inverse()), transform);
    br.sendTransform(tf::StampedTransform(transform, GetCurrentTimestamp(), "vision_world", "vision_pose_sbi_recover"));
    
    //restore
    mpCurrentMKF->mse3BaseFromWorld = se3Saved;
    mpCurrentMKF->UpdateCamsFromWorld();
  }
  */
  
  for(unsigned j=0; j < mvAllCamNames.size(); ++j)
  {
    std::string camName = mvAllCamNames[j];
    mMessageForUser<<camName<<" Thresh: ";
    for(int i=0; i<LEVELS; i++) 
      mMessageForUser<<mpCurrentMKF->mmpKeyFrames[camName]->maLevels[i].nFastThresh<<" ";
    mMessageForUser<<std::endl;
  }
  
  // Decide what to do - if there is a map, try to track the map ...
  if(mMap.mbGood)
  {
    if(!IsLost() && !mbForceRecovery)  // .. but only if we're not lost! and not requesting forced relocalization
    {
      startTime = ros::WallTime::now();
      ApplyMotionModel(); 
      timingMsg.motion = (ros::WallTime::now() - startTime).toSec();     // 
      TrackMap();               //  These three lines do the main tracking work.
      UpdateMotionModel();      //
      
      mMessageForUser << "Last tracker sigma squared: "<<mdLastSigmaSquared << std::endl;
    
      AssessOverallTrackingQuality();  //  Check if we're lost or if tracking is poor.
      
      // Provide some feedback for the user:
      mMessageForUser << "Scene depth: " << mpCurrentMKF->mdTotalDepthMean<<std::endl;
    
      mMessageForUser << "Overall quality: ";
      if(mOverallTrackingQuality == GOOD)  mMessageForUser << "good.";
      if(mOverallTrackingQuality == DODGY) mMessageForUser << "poor.";
      if(mOverallTrackingQuality == BAD)   mMessageForUser << "bad.";
      mMessageForUser << std::endl;
    
      for(unsigned j=0; j < mvAllCamNames.size(); ++j)
      {
        std::string camName = mvAllCamNames[j];
        mMessageForUser<<" Found: ";
        for(int i=0; i<LEVELS; i++) 
          mMessageForUser << (mmMeasFoundLevels[camName])[i] << "/" << (mmMeasAttemptedLevels[camName])[i] << " ";
          
        TrackingQuality quality = mmTrackingQuality[camName];
        mMessageForUser<<"  Quality: ";
        if(quality == GOOD)  mMessageForUser << "good.";
        if(quality == DODGY) mMessageForUser << "poor.";
        if(quality == BAD)   mMessageForUser << "bad.";
        mMessageForUser<<std::endl;
      }
      
      mMessageForUser << "Map: " << mMap.mlpPoints.size() << "P, " << mMap.mlpMultiKeyFrames.size() << "MKF, " << mMap.mlpPointsTrash.size()<<"P in Trash, "<<mMap.mlpMultiKeyFramesTrash.size()<<" MKF in Trash" ;
    
      startTime = ros::WallTime::now();
      ROS_DEBUG_STREAM("Since last dropped: "<<ros::Time::now() - mtLastMultiKeyFrameDropped);
      ROS_DEBUG_STREAM("About to test neednew, mkf depth: "<<mpCurrentMKF->mdTotalDepthMean);
      
      static gvar3<int> gvnAddingMKFs("AddingMKFs", 1, HIDDEN|SILENT);
      // Heuristics to check if a key-frame should be added to the map:
      if(mbAddNext || //mMapMaker.Initializing() ||
         (*gvnAddingMKFs &&
          mOverallTrackingQuality == GOOD &&
          mnLostFrames == 0 &&
          ros::Time::now() - mtLastMultiKeyFrameDropped > ros::Duration(0.5) &&
          //mMapMaker.NeedNewMultiKeyFrame(*mpCurrentMKF, CountMeasurements()))
          mMapMaker.NeedNewMultiKeyFrame(*mpCurrentMKF)))
      {
        if(mbAddNext)
          ROS_DEBUG("Adding MKF because add next was clicked");
          
        if(mMapMaker.Initializing())
          ROS_DEBUG("Adding MKF because map is initializing");
          
        mMessageForUser << " SHOULD BE Adding MultiKeyFrame to Map";
        RecordMeasurements();  // We've decided to add, so make measurements
        AddNewKeyFrame(); 
        mbAddNext = false;
      }
      timingMsg.add = (ros::WallTime::now() - startTime).toSec();
      
      ReleasePointLock();  // Important! Do this whenever tracking step has finished
    }
    else  // what if there is a map, but tracking has been lost?
    {
      mMessageForUser << "** Attempting recovery **";
      if(AttemptRecovery())
      {
        mbForceRecovery = false;
        TrackMap();
        AssessOverallTrackingQuality();
        ReleasePointLock();  // Important! Do this whenever tracking step has finished
      }
    }
    
    if(mbDraw)
      RenderGrid();
      
  }
  else // If there is no map, try to make one.
  {
    TrackForInitialMap(); 
  }
  
  timingMsg.total = (ros::WallTime::now() - startTimeTotal).toSec();
  timingMsg.map_num_points = mMap.mlpPoints.size();
  timingMsg.map_num_mkfs = mMap.mlpMultiKeyFrames.size();
  timingMsg.header.stamp = ros::Time::now();
  timingPub.publish(timingMsg);
}

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
  static gvar3<int> gvnDrawReloc("DrawReloc", 0, HIDDEN|SILENT);
  static gvar3<int> gvnRelocFabMap("RelocFabMap", 1, HIDDEN|SILENT);
  
  bool bSuccess;
  
  if(*gvnRelocFabMap)
  {
    if(*gvnDrawReloc)
    {
      // A bit hacky, since we've already drawn some stuff to the screen
      // in TrackFrameSetup, and now we want to draw the relocalizer instead
      // so just clear with black and draw some more
      glClearColor(0,0,0,1);
      glClear(GL_COLOR_BUFFER_BIT);
      
      // Disable drawing in the main Tracker routines (told you this was hacky)
      mbDraw = false;
    }
    
    bSuccess = mRelocFabMap.FindBestPose(*mpCurrentMKF, *gvnDrawReloc);
  }
  else
  {
    bSuccess = false;
    for(unsigned i=0; i < mvAllCamNames.size(); ++i)
    {
      std::string camName = mvAllCamNames[i];
      KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[camName];
      
      if(!kf.mbActive)
        continue;
      
      bool bRelocGood = mRelocaliser.AttemptRecovery(kf);
      
      if(!bRelocGood)
        continue;
      
      // The pose returned is for a KeyFrame, so we have to calculate the appropriate base MultiKeyFrame pose from it
      SE3<> se3Best = mRelocaliser.BestPose();
      mpCurrentMKF->mse3BaseFromWorld = mpCurrentMKF->mmpKeyFrames[camName]->mse3CamFromBase.inverse() * se3Best;  // CHECK!! GOOD
      mpCurrentMKF->UpdateCamsFromWorld();
      
      bSuccess = true;
      break;
    }
  }
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::poseMsgToTF(util::SE3ToPoseMsg(GetCurrentPose().inverse()), transform);
  br.sendTransform(tf::StampedTransform(transform, GetCurrentTimestamp(), "vision_world", "vision_pose_recover"));
  
  if(!bSuccess)
    return false;
  
  mse3StartPose = mpCurrentMKF->mse3BaseFromWorld;
  mv6BaseVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not, drawn at z=0
void Tracker::RenderGrid()
{
  // The colour of the ref grid shows if the coarse stage of tracking was used
  // (it's turned off when the camera is sitting still to reduce jitter.)
  if(mbDidCoarse)
    glColor4f(.0, 0.5, .0, 0.6);
  else
    glColor4f(0,0,0,0.6);
  
  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  int nHalfCells = 8;
  int nTot = nHalfCells * 2 + 1;
  CVD::Image<Vector<2> >  imVertices(CVD::ImageRef(nTot,nTot));
  
  static gvar3<int> gvnDrawLevel("DrawLevel", 0, HIDDEN|SILENT);
  
  std::string firstCamName = mvAllCamNames[0];
  SE3<> se3FirstCamFromWorld = mpCurrentMKF->mmpKeyFrames[firstCamName]->mse3CamFromWorld; 
  TaylorCamera& camera =  mmCameraModels[firstCamName];
  Vector<2> v2Offset = vec(mmDrawOffsets[firstCamName]);
  //Vector<2> v2Extents = LevelNPos(vec(mmSizes[firstCamName]), *gvnDrawLevel);
  Vector<2> v2Extents = vec(mmSizes[firstCamName]);
  
  for(int i=0; i<nTot; i++)
  {
    for(int j=0; j<nTot; j++)
    {
      Vector<3> v3;
      v3[0] = (i - nHalfCells) * 0.1;
      v3[1] = (j - nHalfCells) * 0.1;
      v3[2] = 0.0;
      Vector<3> v3Cam = se3FirstCamFromWorld * v3;
      if(v3Cam[2] < 0.001)
        v3Cam[2] = 0.001;
        
      //imVertices[i][j] = LevelNPos(camera.Project(v3Cam), *gvnDrawLevel);
      imVertices[i][j] = camera.Project(v3Cam);
    }
  }
    
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  
  for(int i=0; i<nTot; i++)
  {
    glBegin(GL_LINE_STRIP);
    for(int j=0; j<nTot; j++)
    {
      if(util::PointInRectangle(imVertices[i][j], v2Extents))
        CVD::glVertex(imVertices[i][j] + v2Offset);
    }
    glEnd();
    
    glBegin(GL_LINE_STRIP);
    for(int j=0; j<nTot; j++)
    {
      if(util::PointInRectangle(imVertices[j][i], v2Extents))
        CVD::glVertex(imVertices[j][i] + v2Offset);
    }
    glEnd();
  };
  
  glLineWidth(1);
  glColor3f(1,0,0);
}


// Calls the map maker's initialization routine with the current MultiKeyFrame
void Tracker::TrackForInitialMap()
{
  ROS_ASSERT(!mMap.mbGood);  // should only be here if no map exists
  
  if(mbInitRequested)
  {
    mbInitRequested = false;
    if(mMapMaker.Init(mpCurrentMKF, mbPutPlaneAtOrigin))
    {
      mse3StartPose = mMap.mlpMultiKeyFrames.back()->mse3BaseFromWorld;  // set starting pose from first MultiKeyFrame pose
      InitCurrentMKF(mse3StartPose);  // need to regenerate MultiKeyFrame
      CopySceneDepths(*mMap.mlpMultiKeyFrames.back());
      CopyMasks(*mMap.mlpMultiKeyFrames.back());
      mnLastMultiKeyFrameDropped = mnFrame;
      mtLastMultiKeyFrameDropped = ros::Time::now();
    }
    else
    {
      ROS_ERROR("Tracker: Couldn't make initial map, try again");
      Reset(true, true);
    }
  }
  else
    mMessageForUser << "Press spacebar to build initial map." << std::endl;
  
}
  
// Finds the Potentialy Visible Set, ie TrackerDatas referencing MapPoints that should be seen from one camera
void Tracker::FindPVS(std::string cameraName, TDVLevels& vPVSLevels)
{
  TaylorCamera& camera = mmCameraModels[cameraName];
  KeyFrame& kf = *(mpCurrentMKF->mmpKeyFrames[cameraName]);
  
  boost::mutex::scoped_lock lock(mMap.mMutex);
   
  std::set<MapPoint*> spNearestPoints;
  CollectNearestPoints(kf, spNearestPoints);
  
  ROS_INFO_STREAM("Found "<<spNearestPoints.size()<<" nearest points");
   
  // For all points we collected
  for(std::set<MapPoint*>::iterator point_it = spNearestPoints.begin(); point_it != spNearestPoints.end(); ++point_it)
  {
    MapPoint &point = *(*point_it); 
    
    if(point.mbBad || !point.mbOptimized)
      continue;
      
    // Ensure that this map point has an associated TrackerData struct.
    if(!point.mmpTData.count(cameraName))
    {
      point.mmpTData[cameraName] = new TrackerData(&point, mmSizes[cameraName]);
    }
   
    // Use boost intrusive_ptr to increment MapPoint's using counter, preventing its deletion while we're
    // holding onto this pointer
    boost::intrusive_ptr<TrackerData> pTData(point.mmpTData[cameraName]);
    
    // Project according to current view, and if it's not in the image, skip.
    pTData->Project(kf.mse3CamFromWorld,  camera); 
    if(!pTData->mbInImage)
      continue;
      
    // If we have a mask, make sure point is not projected into masked area
    if(kf.maLevels[0].mask.totalsize() > 0 && kf.maLevels[0].mask[CVD::ir(pTData->mv2Image)] == 0)
      continue;
    
    // Calculate camera projection derivatives of this point.
    pTData->GetDerivsUnsafe(camera);

    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
    pTData->mnSearchLevel = pTData->mFinder.CalcSearchLevelAndWarpMatrix(pTData->mPoint, kf.mse3CamFromWorld, pTData->mm2CamDerivs);
    if(pTData->mnSearchLevel == -1)
      continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

    // Debugging, rough check for a counting leak
    if(point.mnUsing > mmCameraModels.size())
    {
      ROS_FATAL_STREAM("Tracker: mnUsing greater than number of cameras, counting leak!: "<<point.mnUsing);
      ROS_BREAK();
    }
    
    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
    pTData->mbSearched = false;
    pTData->mbFound = false;
    vPVSLevels[pTData->mnSearchLevel].push_back(pTData);
  }
    
}

// Gather some TrackerDatas from coarse levels (ie high pyramid level number), try to find them and return the number found
int Tracker::TestForCoarse(TDVLevels& vPVSLevels, std::string cameraName, unsigned int nCoarseRange, unsigned int nCoarseMax, int nCoarseSubPixIts, TrackerDataPtrVector& vIterationSet)
{
  //std::cout<<"In Tracker::TestForCoarse, working on "<<cameraName<<std::endl;
  TrackerDataPtrVector vNextToSearch;

  // Fill the vNextToSearch struct with an appropriate number of 
  // TrackerDatas corresponding to coarse map points! This depends on how many
  // there are in different pyramid levels compared to CoarseMax.
      
  if(vPVSLevels[LEVELS-1].size() <= nCoarseMax) 
  { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
    vNextToSearch = vPVSLevels[LEVELS-1];
    vPVSLevels[LEVELS-1].clear();
  }
  else
	{ // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
	  for(unsigned int i=0; i<nCoarseMax; i++)
	    vNextToSearch.push_back(vPVSLevels[LEVELS-1][i]);
      
	  vPVSLevels[LEVELS-1].erase(vPVSLevels[LEVELS-1].begin(), vPVSLevels[LEVELS-1].begin() + nCoarseMax);
	}
      
  // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
  if(vNextToSearch.size() < nCoarseMax)
	{
	  unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
	  if(vPVSLevels[LEVELS-2].size() <= nMoreCoarseNeeded)
    {
      vNextToSearch.insert(vNextToSearch.end(), vPVSLevels[LEVELS-2].begin(), vPVSLevels[LEVELS-2].end());
      vPVSLevels[LEVELS-2].clear();
    }
	  else
    {
      for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
        vNextToSearch.push_back(vPVSLevels[LEVELS-2][i]);
        
      vPVSLevels[LEVELS-2].erase(vPVSLevels[LEVELS-2].begin(), vPVSLevels[LEVELS-2].begin() + nMoreCoarseNeeded);
    }
	}
  
  // Now go and attempt to find these points in the image!
  unsigned int nFound = SearchForPoints(vNextToSearch, cameraName, nCoarseRange, nCoarseSubPixIts);
  vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
  
  return nFound;
  
}
      
// Takes one nonlinear step in updating the pose of the current MultiKeyFrame, using the TrackerDatas collected by (potentially) TestForCoarse and SetupFineTracking
Vector<6> Tracker::PoseUpdateStep(std::vector<TrackerDataPtrVector>& vIterationSets, int nIter, double dOverrideSigma, bool bMarkOutliers)
{
  for(unsigned j=0; j < mvCurrCamNames.size(); ++j)
  {
    std::string camName = mvCurrCamNames[j];
    TaylorCamera& camera = mmCameraModels[camName];
    KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[camName];
    
    if(nIter != 0)
    { // Re-project the points on all but the first iteration.
      for(unsigned int i=0; i<vIterationSets[j].size(); i++)
      {
        if(vIterationSets[j][i]->mbFound)  
          vIterationSets[j][i]->ProjectAndDerivs(kf.mse3CamFromWorld, camera);
      }
    }
    
    for(unsigned int i=0; i<vIterationSets[j].size(); i++)
    {
      if(vIterationSets[j][i]->mbFound)
        vIterationSets[j][i]->CalcJacobian(mpCurrentMKF->mse3BaseFromWorld, kf.mse3CamFromBase);
    }
        
  }
      
  // Force the MEstimator to be pretty brutal with outliers beyond the fifth iteration.
  if(nIter <= 5)
    dOverrideSigma = 0.0;
      
  // Calculate and apply the pose update...
  Vector<6> v6Update =  CalcPoseUpdate(vIterationSets, dOverrideSigma, bMarkOutliers);
  mpCurrentMKF->mse3BaseFromWorld = SE3<>::exp(v6Update) * mpCurrentMKF->mse3BaseFromWorld;
  
  // Update the KeyFrame cam-from-world poses
  mpCurrentMKF->UpdateCamsFromWorld();
    
  return v6Update;
}

// Takes one linear step in updating the pose of the current MultiKeyFrame, using the TrackerDatas collected by (potentially) TestForCoarse and SetupFineTracking
Vector<6> Tracker::PoseUpdateStepLinear(std::vector<TrackerDataPtrVector>& vIterationSets, Vector<6>& v6LastUpdate, int nIter, double dOverrideSigma, bool bMarkOutliers)
{
  for(unsigned j=0; j < mvCurrCamNames.size(); ++j)
  {
    for(unsigned int i=0; i<vIterationSets[j].size(); i++)
    {
      if(vIterationSets[j][i]->mbFound)
        vIterationSets[j][i]->LinearUpdate(v6LastUpdate);
    }
  }
	    
  // Force the MEstimator to be pretty brutal with outliers beyond the fifth iteration.
  if(nIter <= 5)
    dOverrideSigma = 0.0;
      
  // Calculate and update pose;
  Vector<6> v6Update = 	CalcPoseUpdate(vIterationSets, dOverrideSigma, bMarkOutliers);
  mpCurrentMKF->mse3BaseFromWorld = SE3<>::exp(v6Update) * mpCurrentMKF->mse3BaseFromWorld;
  
  // Update the KeyFrame cam-from-world poses
  mpCurrentMKF->UpdateCamsFromWorld();
      
  return v6Update;
}

//  Collect more TrackerDatas at fine levels (low pyramid number), try to find them, add them to the ones found by TestForCoarse
void Tracker::SetupFineTracking(TDVLevels& vPVSLevels, TrackerDataPtrVector& vIterationSet, std::string cameraName, bool bDidCoarse)
{
  TrackerDataPtrVector vNextToSearch;
  
  TaylorCamera& camera = mmCameraModels[cameraName];
  KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[cameraName];
  
  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!
  
  // should be 10
  int nFineRange = 20;  // Pixel search range for the fine stage. 
  if(bDidCoarse)       // Can use a tighter search if the coarse stage was already done.
    nFineRange = 10;
    
  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.

  int l = LEVELS - 1;
  for(unsigned int i=0; i<vPVSLevels[l].size(); i++)
    vPVSLevels[l][i]->ProjectAndDerivs(kf.mse3CamFromWorld, camera);
    
  SearchForPoints(vPVSLevels[l], cameraName, nFineRange, 8);
  for(unsigned int i=0; i<vPVSLevels[l].size(); i++)
    vIterationSet.push_back(vPVSLevels[l][i]);  // Again, plonk all searched points onto the (maybe already populated) vIterationSet.
  
  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  for(int l=LEVELS - 2; l>=0; l--)
  {
    for(unsigned int i=0; i<vPVSLevels[l].size(); i++)
      vNextToSearch.push_back(vPVSLevels[l][i]);
  }
  
  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit 
  // ourselves to snMaxPatchesPerFrame, and choose these randomly.
  int nFinePatchesToUse = Tracker::snMaxPatchesPerFrame - vIterationSet.size();
  if(nFinePatchesToUse < 0)
    nFinePatchesToUse = 0;
    
  if((int) vNextToSearch.size() > nFinePatchesToUse)
  {
    random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
    vNextToSearch.resize(nFinePatchesToUse); // Chop!
  }
  
  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(bDidCoarse)
  {
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
      vNextToSearch[i]->ProjectAndDerivs(kf.mse3CamFromWorld, camera);
  }
  
  // Find fine points in image:
  // Want at least a few sub pixel refinements, otherwise tracker
  // is prone to getting lost with bad patch matches
  // Never mind, just had maxSSD set way too high in PatchFinder
  // Keep sub pix its at 0 to be a bit lenient and allow recovery from slightly
  // bad poses
  //std::cout<<"About to call SearchForPoints with "<<vNextToSearch.size()<<" TrackerDatas"<<std::endl;
  SearchForPoints(vNextToSearch, cameraName, nFineRange, 0);// 10);
  
  // And attach them all to the end of the optimisation-set.
  for(unsigned int i=0; i<vNextToSearch.size(); i++)
    vIterationSet.push_back(vNextToSearch[i]);
}
    
    
// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the images to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the images;
// Then it updates the current MultiKeyFrame pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current KeyFrames
void Tracker::TrackMap()
{
  //std::cout<<"Starting TrackMap"<<std::endl;
  
  // This will collect TrackerDatas that will be used in the pose update near the end of the function
  mvIterationSets.clear();
  mvIterationSets.resize(mvCurrCamNames.size());
  
  
  // The per-camera Potentially-Visible-Set (PVS) is split into pyramid levels
  // So it looks like this:
  //                camera1                 camera2                 camera3
  //          --------------------------------------------------------------------
  // LEVEL0   vector<TrackerData*>    vector<TrackerData*>    vector<TrackerData*>
  // LEVEL1   vector<TrackerData*>    vector<TrackerData*>    vector<TrackerData*>
  // LEVEL2   vector<TrackerData*>    vector<TrackerData*>    vector<TrackerData*>
  // LEVEL3   vector<TrackerData*>    vector<TrackerData*>    vector<TrackerData*>
  //
  // The first dimension of vPVSLevels is along the camera names, the second dimension along the LEVELs, 
  // and the third dimension along the TrackerData* vectors
  // Note that the TrackerData pointers are really boost intrusive_ptr
  
  std::vector< TDVLevels > vPVSLevels;
  vPVSLevels.resize(mvCurrCamNames.size());
  
  int anNumPVSPoints[LEVELS];
  for(int i=0; i < LEVELS; ++i)
  { 
    anNumPVSPoints[i] = 0;
  }
  
  ros::WallTime startTime;
  
  startTime = ros::WallTime::now();
  
  for(unsigned j=0; j < mvCurrCamNames.size(); ++j)
  {
    std::string camName = mvCurrCamNames[j];
    for(int i=0; i<LEVELS; i++)
    {
      // Some accounting which will be used for tracking quality assessment:
      mmMeasAttemptedLevels[camName][i] = mmMeasFoundLevels[camName][i] = 0;
      vPVSLevels[j][i].reserve(500);
    }
    FindPVS(camName, vPVSLevels[j]);
    
    for(int i=0; i<LEVELS; i++)
      anNumPVSPoints[i] += vPVSLevels[j][i].size();
  }
  
  timingMsg.pvs = (ros::WallTime::now() - startTime).toSec();
  
  
  for(int i=0; i<LEVELS; i++)
  {
    std::cerr<<"PVS["<<i<<"]: "<<anNumPVSPoints[i]<<std::endl;
  }
  
  
  
  startTime = ros::WallTime::now();
  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for(unsigned j=0; j < mvCurrCamNames.size(); ++j)
  {
    for(int i=0; i<LEVELS; i++)
    {
      random_shuffle(vPVSLevels[j][i].begin(), vPVSLevels[j][i].end());
    }
  }
  
  timingMsg.shuffle = (ros::WallTime::now() - startTime).toSec();
  

  unsigned int nCoarseMax = Tracker::snCoarseMax;
  unsigned int nCoarseRange = Tracker::snCoarseRange;
  
  mbDidCoarse = false;

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(Tracker::sbDisableCoarse || mdMSDScaledVelocityMagnitude < Tracker::sdCoarseMinVelocity || nCoarseMax == 0)
    bTryCoarse = false;
    
  //std::cout<<"mdMSDScaledVelocityMagnitude: "<<mdMSDScaledVelocityMagnitude<<"  coarse min vel: "<<Tracker::sdCoarseMinVelocity<<std::endl;
    
  if(mbJustRecoveredSoUseCoarse)
  {
    bTryCoarse = true;
    nCoarseMax *=2;
    nCoarseRange *=2;
    mbJustRecoveredSoUseCoarse = false;
  }
  
  startTime = ros::WallTime::now();
  
  if(bTryCoarse)
  {
    unsigned int nNumCoarseFound = 0;
    
    // Try finding coarse points in each camera
    for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
    {
      nNumCoarseFound += TestForCoarse(vPVSLevels[i], mvCurrCamNames[i], nCoarseRange, nCoarseMax, Tracker::snCoarseSubPixIts, mvIterationSets[i]);
    }
    
    //std::cout<<"Found "<<nNumCoarseFound<<" coarse points (minimum for coarse step: "<<Tracker::snCoarseMin<<std::endl;
    
    if((int)nNumCoarseFound > Tracker::snCoarseMin)// Were enough found to do any meaningful optimisation?
    {
      mbDidCoarse = true;
      for(int iter = 0; iter<10 && mvCurrCamNames.size() > 0; iter++) // If so: do ten Gauss-Newton pose updates iterations.
      {
        PoseUpdateStep(mvIterationSets, iter, 1.0, false);  // Don't mark outliers yet since we'll do more optimization later
      }
    }
    else
    {
      ROS_DEBUG("Not enough for coarse");
    }
  }
  
  timingMsg.coarse = (ros::WallTime::now() - startTime).toSec();
    
  startTime = ros::WallTime::now();
    
  // Collect and find fine points for all cameras
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    SetupFineTracking(vPVSLevels[i], mvIterationSets[i], mvCurrCamNames[i], mbDidCoarse);
    
    int nNumGoodPatches = 0;
    for(unsigned j=0; j < mvIterationSets[i].size(); ++j)
    {
      if(mvIterationSets[i][j]->mbFound)
        ++nNumGoodPatches;
    }
    
    ROS_DEBUG_STREAM("Camera "<<mvCurrCamNames[i]<<" num good patches: "<<nNumGoodPatches);
  }
  
  timingMsg.fine = (ros::WallTime::now()-startTime).toSec();
  
  startTime = ros::WallTime::now();
  Vector<6> v6LastUpdate;
  v6LastUpdate = Zeros;
  
  static gvar3<int> gvnUpdatingPoints("UpdatingPoints", 1, HIDDEN|SILENT);
  
  // Again, ten gauss-newton pose update iterations.
  for(int iter = 0; iter<10 && mvCurrCamNames.size() > 0; iter++)
  {
    // For a bit of time-saving: don't do full nonlinear
    // reprojection at every iteration - it really isn't necessary!
    if(iter == 0 || iter == 4 || iter == 9)
      v6LastUpdate = PoseUpdateStep(mvIterationSets, iter, 16.0, iter==9 && *gvnUpdatingPoints);   // Even this is probably overkill, the reason we do many
    else                                                                      // iterations is for M-Estimator convergence rather than 
      v6LastUpdate = PoseUpdateStepLinear(mvIterationSets, v6LastUpdate, iter, 16.0, iter==9 && *gvnUpdatingPoints);   // linearisation effects.
      
  }
  
  
  timingMsg.pose = (ros::WallTime::now()-startTime).toSec();
  
  // Want to penalize those points that weren't found but should have, ie their parent keyframe
  // right now is pretty close to where it was when the point was generated
  /*
  std::map<std::string, double> mDistFromOriginal;
  
  for(unsigned j=0; j < mvCurrCamNames.size(); ++j)
  {
    for(unsigned int i=0; i<mvIterationSets[j].size(); i++)
    {
      if(mvIterationSets[j][i]->mbFound)  // point was found, don't penalize
        continue;
        
      // This is different than the usual camName variable since it's the name of the camera that observed
      // a point originally rather than the name of the camera at the current j index
      std::string camName = mvIterationSets[j][i]->mPoint.mpPatchSourceKF->mCamName;
      
      if(mpCurrentMKF->mmpKeyFrames.count(camName) == 0)  // camera not in currentMKF (ie in calibration) so ignore
        continue;
        
      if(mDistFromOriginal.count(camName) == 0)
      {
        KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[camName];
        double dist = kf.Distance(*(mvIterationSets[j][i]->mPoint.mpPatchSourceKF));
        mDistFromOriginal.insert(std::pair<std::string,double>(camName,dist));
      }
      
      if(mDistFromOriginal[camName] < 0.3)  // penalize if kf really close to original location
        mvIterationSets[j][i]->mPoint.mnMEstimatorOutlierCount++;
      
    }
  }
  */
  
  if(mbDraw)  // We'll draw mvIterationSets with different colors based on pyramid level
  {
    ROS_ASSERT(mpGLWindow);
    
    static gvar3<int> gvnDrawLevel("DrawLevel", 0, HIDDEN|SILENT);
    static gvar3<int> gvnDrawOnlyLevel("DrawOnlyLevel", 0, HIDDEN|SILENT);
    
    glPointSize(LevelScale(*gvnDrawLevel) + 5);
    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_POINTS);
    
    for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
    {
      std::string camName = mvCurrCamNames[i];
      for(TrackerDataPtrVector::iterator td_it = mvIterationSets[i].begin(); td_it!= mvIterationSets[i].end(); ++td_it)
      {
        TrackerData& td = *(*td_it);
        
        if(!td.mbFound)
          continue;
          
        if(*gvnDrawOnlyLevel && td.mnSearchLevel != *gvnDrawLevel)
          continue;
          
        CVD::glColor(gavLevelColors[td.mnSearchLevel]);
        //CVD::glVertex(LevelNPos(td.mv2Image, *gvnDrawLevel) + vec(mmDrawOffsets[camName]));
        CVD::glVertex(td.mv2Image + vec(mmDrawOffsets[camName]));
      }
    }
    
    glEnd();
    glDisable(GL_BLEND);
  }
  
  // This is needed for testing the map to see if a new MultiKeyFrame is needed
  // because that test uses the distance between MultiKeyFrames, and that needs a mean scene depth
  startTime = ros::WallTime::now();
  RefreshSceneDepth(mvIterationSets);
  timingMsg.depth = (ros::WallTime::now() - startTime).toSec();
  
  SaveSimpleMeasurements(mvIterationSets);
}

// Save a simplified copy of the measurements made during tracking
void Tracker::SaveSimpleMeasurements(std::vector<TrackerDataPtrVector>& vIterationSets)
{
  mmSimpleMeas.clear();
  
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    std::string camName = mvCurrCamNames[i];
  
    ROS_DEBUG_STREAM("Tracker: Recording "<<vIterationSets[i].size()<<" simple measurements in camera "<<i);
    
    for(TrackerDataPtrVector::iterator td_it = vIterationSets[i].begin(); td_it!= vIterationSets[i].end(); ++td_it)
    {
      TrackerData& td = *(*td_it);
      
      if(!td.mbFound || td.mPoint.mbBad)
        continue;
        
      mmSimpleMeas[camName][td.mnSearchLevel].push_back(td.mv2Found);
    }
  }
}

// Update the scene depth variables of the current MultiKeyFrame and KeyFrames without recording any measurements
void Tracker::RefreshSceneDepth(std::vector<TrackerDataPtrVector>& vIterationSets)
{
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    std::string camName = mvCurrCamNames[i];
    KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[camName];
    
    std::vector<std::pair<double, double> > vDepthsAndWeights;
    vDepthsAndWeights.reserve(vIterationSets[i].size());
  
    for(TrackerDataPtrVector::iterator td_it = vIterationSets[i].begin(); td_it!= vIterationSets[i].end(); ++td_it)
    {
      TrackerData& td = *(*td_it);
      
      if(!td.mbFound)
        continue;
        
      MapPoint& point = td.mPoint;
      
      ROS_ASSERT(point.mnMEstimatorInlierCount > 0); // should be set to 1 when point is created
      
      // Calculate the weight the same way as KeyFrame's internal scene depth calculator
      double weight = point.mnMEstimatorInlierCount / (double)(point.mnMEstimatorInlierCount + point.mnMEstimatorOutlierCount);
      
      vDepthsAndWeights.push_back(std::make_pair(norm(td.mv3Cam), weight));
    }
    
    // Use the version of KeyFrame's RefreshSceneDepthRobust that accepts an external set of depths and weights
    // This allows us to calculate the robust scene depth without creating measurements and saving them in KeyFrames
    kf.RefreshSceneDepthRobust(vDepthsAndWeights);
  }
  
  double dSumDepth = 0.0;
  int nNum = 0;
  
  for(KeyFramePtrMap::iterator kf_it = mpCurrentMKF->mmpKeyFrames.begin(); kf_it != mpCurrentMKF->mmpKeyFrames.end(); ++kf_it)
  {
    KeyFrame& kf = *(kf_it->second);
    if(!kf.mbActive)
      continue;
      
    dSumDepth += kf.mdSceneDepthMean;
    ++nNum;
  }
  
  ROS_ASSERT(nNum > 0);
  mpCurrentMKF->mdTotalDepthMean = dSumDepth/nNum;
  
}

// Clears mvIterationSets, which causes the "using" count of MapPoints to decrement as the boost intrusive pointers destruct 
void Tracker::ReleasePointLock()
{
  mvIterationSets.clear();
}

// Create measurements of the MapPoints referenced in mvIterationSets, add them to the current KeyFrames
void Tracker::RecordMeasurements()
{
  //debug
  static gvar3<int> gvnCrossCamera("CrossCamera", 1, HIDDEN|SILENT);
  
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    std::string camName = mvCurrCamNames[i];
  
    ROS_DEBUG_STREAM("Tracker: Recording "<<mvIterationSets[i].size()<<" measurements in camera "<<i);
    
    for(TrackerDataPtrVector::iterator td_it = mvIterationSets[i].begin(); td_it!= mvIterationSets[i].end(); ++td_it)
    {
      TrackerData& td = *(*td_it);
      
      if(!td.mbFound || td.mPoint.mbBad)
        continue;
        
      //debug
      if(!*gvnCrossCamera && td.mPoint.mpPatchSourceKF->mCamName != camName)
        continue;
      
      Measurement* pMeas = new Measurement;
      pMeas->eSource = Measurement::SRC_TRACKER;
      pMeas->v2RootPos = td.mv2Found;
      pMeas->nLevel = td.mnSearchLevel;
      pMeas->bSubPix = td.mbDidSubPix; 
      
      KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[camName];
      MapPoint& point = td.mPoint;
      
      kf.AddMeasurement(&point, pMeas, false);
      //kf.mmpMeasurements[&point] = pMeas;
      //point.mMMData.spMeasurementKFs.insert(&kf);
      
    }
  }
}

// Count the number of potential measurements
int Tracker::CountMeasurements()
{
  int sum = 0;
  
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    for(TrackerDataPtrVector::iterator td_it = mvIterationSets[i].begin(); td_it!= mvIterationSets[i].end(); ++td_it)
    {
      TrackerData& td = *(*td_it);
      
      if(!td.mbFound)
        continue;
     
      ++sum;
    }
  }
  
  return sum;
}


// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(TrackerDataPtrVector& vTD, std::string cameraName, int nRange, int nSubPixIts, bool bExhaustive)
{
  KeyFrame& kf = *mpCurrentMKF->mmpKeyFrames[cameraName];
  
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
  {
    int nRangeForPoint = nRange;
    int nSubPixItsForPoint = nSubPixIts;
    
    // First, attempt a search at pixel locations which are FAST corners.
    TrackerData &td = *vTD[i];
    PatchFinder &finder = td.mFinder;
    finder.MakeTemplateCoarseCont(td.mPoint);
    
    //std::cout<<"Trying to find "<<td.mv2Image<<std::endl;
    
    if(finder.TemplateBad())
    {
      td.mbInImage = td.mbFound = false;
      //std::cout<<"Finder template bad"<<std::endl;
      continue;
    }
    mmMeasAttemptedLevels[cameraName][finder.GetLevel()]++;  // Stats for tracking quality assessment
    
    // For fixed points, we want to try REALLY hard to find them, since they are being used
    // for some kind of calibration and finding them is extremely important
    bool bExhaustiveSearch = td.mPoint.mbFixed || bExhaustive;
    if(bExhaustiveSearch)
    {
      //nRangeForPoint /= 2;        // Reduce the search range
      nSubPixItsForPoint = 10;    // But force sub-pixel iterations
    }
    
    int nScore;
    bool bFound = finder.FindPatchCoarse(CVD::ir(td.mv2Image), kf, nRangeForPoint, nScore, bExhaustiveSearch);
    td.mbSearched = true;
    if(!bFound) 
    {
      //std::cout<<"Not found in coarse stage!"<<std::endl;
      td.mbFound = false;
      continue;
    }
    
    td.mbFound = true;
    td.mdSqrtInvNoise = (1.0 / finder.GetLevelScale());
    
    nFound++;
    mmMeasFoundLevels[cameraName][finder.GetLevel()]++;
    
    // Found the patch in coarse search - are Sub-pixel iterations wanted too?
    if(nSubPixItsForPoint > 0)
    {
      td.mbDidSubPix = true;
      finder.MakeSubPixTemplate();  // NOT NEEDED SINCE MakeTemplateCoarseCont does it??
      finder.SetSubPixPos(finder.GetCoarsePosAsVector());
      bool bSubPixConverges=finder.IterateSubPixToConvergence(kf, nSubPixItsForPoint);
      
      if(!bSubPixConverges)
      { // If subpix doesn't converge, the patch location is probably very dubious!
        td.mbFound = false;
        nFound--;
        mmMeasFoundLevels[cameraName][finder.GetLevel()]--;
        //std::cout<<"Subpix didn't converge!"<<std::endl;
        continue;
      }
      
      td.mv2Found = finder.GetSubPixPos();
      //std::cout<<"Found at "<<td.mv2Found<<std::endl;
    }
    else
    {
      td.mv2Found = finder.GetCoarsePosAsVector();
      td.mbDidSubPix = false;
      //std::cout<<"Found at "<<td.mv2Found<<std::endl;
    }
  }
  return nFound;
}

//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the MEstimator.
Vector<6> Tracker::CalcPoseUpdate(std::vector<TrackerDataPtrVector>& vIterationSets, double dOverrideSigma, bool bMarkOutliers)
{
  // Which M-estimator are we using?
  int nEstimator = 0;
  if(Tracker::sMEstimatorName == "Tukey")
    nEstimator = 0;
  else if(Tracker::sMEstimatorName == "Cauchy")
    nEstimator = 1;
  else if(Tracker::sMEstimatorName == "Huber")
    nEstimator = 2;
  else 
  {
    ROS_FATAL_STREAM("Tracker: Invalid Tracker MEstimator selected: "<<Tracker::sMEstimatorName<<", choices are [Tukey, Cauchy, Huber]");
    ros::shutdown();
  }
  
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  std::vector<double> vErrorSquared;
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
      
    for(unsigned int j=0; j < vIterationSets[i].size(); ++j)
    {
      TrackerData &td = *vIterationSets[i][j];
      if(!td.mbFound)
        continue;
        
      td.mv2Error_CovScaled = td.mdSqrtInvNoise * (td.mv2Found - td.mv2Image);
      vErrorSquared.push_back(td.mv2Error_CovScaled * td.mv2Error_CovScaled);
    }
  }
  
  // No valid measurements? Return null update.
  if(vErrorSquared.size() == 0)
    return makeVector(0,0,0,0,0,0);
  
  // Find the sigma squared that will be used in assigning weights
  double dSigmaSquared;
  if(dOverrideSigma > 0)
    dSigmaSquared = dOverrideSigma; 
  else
  {
    if (nEstimator == 0)
      dSigmaSquared = Tukey::FindSigmaSquared(vErrorSquared);
    else if(nEstimator == 1)
      dSigmaSquared = Cauchy::FindSigmaSquared(vErrorSquared);
    else 
      dSigmaSquared = Huber::FindSigmaSquared(vErrorSquared);
  
    mdLastSigmaSquared = dSigmaSquared;
  }
  
  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls; //, wls_noweight;
  wls.add_prior(100.0); // Stabilising prior
  //wls_noweight.add_prior(100);
  mnNumInliers = 0;
  
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    
    for(unsigned int j=0; j < vIterationSets[i].size(); ++j)
    {
      TrackerData &td = *vIterationSets[i][j];
      if(!td.mbFound)
      { 
        if(td.mbSearched && bMarkOutliers && !IsLost())  // testing: mark outliers even if they're not found (not just if they're outliers in pose minimization)
          td.mPoint.mnMEstimatorOutlierCount++;
        
        continue;
      }
      
      Vector<2> &v2Error = td.mv2Error_CovScaled;
      double dErrorSq = v2Error * v2Error;
      double dWeight;
      
      if(nEstimator == 0)
        dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
        dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else 
        dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
      
      //wls_noweight.add_mJ(v2Error[0], td.mdSqrtInvNoise * td.mm26Jacobian[0], 1); // These two lines are currently
      //wls_noweight.add_mJ(v2Error[1], td.mdSqrtInvNoise * td.mm26Jacobian[1], 1); // the slowest bit of poseits
      
      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
      {
        if(bMarkOutliers)
          td.mPoint.mnMEstimatorOutlierCount++;
          
        continue;
      }
      else
      {
        if(bMarkOutliers)
        {
          td.mPoint.mnMEstimatorInlierCount++;
          mnNumInliers++;
        }
      }
      
      Matrix<2,6> &m26Jac = td.mm26Jacobian;
      wls.add_mJ(v2Error[0], td.mdSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2Error[1], td.mdSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
    
  }
  
  wls.compute();
  
  if(bMarkOutliers)	
  {
		mm6PoseCovariance = TooN::SVD<6>(wls.get_C_inv()).get_pinv();   // from ethzasl_ptam
    
    //wls_noweight.compute();
    //mm6PoseCovarianceNoOutliers = TooN::SVD<6>(wls_noweight.get_C_inv()).get_pinv();   // from ethzasl_ptam
  }
  
  //std::cout<<"CalcPoseUpdate: "<<wls.get_mu()<<std::endl;
  
  return wls.get_mu();
}


// Calculate the assumed motion based on the last known velocity and the last time step
// and add it to the current pose to get an updated pose
void Tracker::ApplyMotionModel()
{
  mse3StartPose = mpCurrentMKF->mse3BaseFromWorld;
  
  // Old style motion update with simple velocity  model
  //Vector<6> v6Motion = mv6BaseVelocity * mLastProcessDur.toSec();
  Vector<6> v6Motion = Zeros;
  
  if(Tracker::sbUseRotationEstimator)  // estimate the rotation component of the motion from SmallBlurryImage differences
  {
    Vector<3> v3SBIRot = Zeros;
    bool bSuccess = CalcSBIRotation(v3SBIRot);
    if(bSuccess)
      v6Motion.slice<3,3>() = v3SBIRot;
  }
  
  mpCurrentMKF->mse3BaseFromWorld = SE3<>::exp(v6Motion) * mse3StartPose;
  
  // Need to do this last after base pose updated
  mpCurrentMKF->UpdateCamsFromWorld();
}

// The motion model is updated after TrackMap has found a new pose
void Tracker::UpdateMotionModel()
{
  // Old style updating
  SE3<> se3NewFromOld = mpCurrentMKF->mse3BaseFromWorld * mse3StartPose.inverse();
  Vector<6> v6NewVel = SE3<>::ln(se3NewFromOld) / mLastProcessDur.toSec();
  Vector<6> v6OldVel = mv6BaseVelocity;
  
  // Updated velocity is average of new and old
  mv6BaseVelocity = (0.5 * v6NewVel + 0.5 * v6OldVel) * 0.9;
  
  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  Vector<6> v6 = mv6BaseVelocity;
  v6.slice<0,3>() *= 1.0 / mpCurrentMKF->mdTotalDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

// Gives the current MultiKeyFrame to the map maker. The current MultiKeyFrame needs to be regenerated.
void Tracker::AddNewKeyFrame()
{
  ROS_DEBUG("Tracker: About to add new key frame");
  
  // Save current pose because we'll need it to regenerate current MultiKeyFrame
  SE3<> se3Pose = mpCurrentMKF->mse3BaseFromWorld;
  MultiKeyFrame* pCurrentMKF_Temp = mpCurrentMKF;
  mpCurrentMKF = NULL;
  InitCurrentMKF(se3Pose);
  CopySceneDepths(*pCurrentMKF_Temp);
  CopyMasks(*pCurrentMKF_Temp);
  mMapMaker.AddMultiKeyFrame(pCurrentMKF_Temp);  // map maker takes ownership
  
  mnLastMultiKeyFrameDropped = mnFrame;
  mtLastMultiKeyFrameDropped = ros::Time::now();
}

// Find the tracking quality of the system by taking the top quality out of all the cameras
void Tracker::AssessOverallTrackingQuality()
{
  TrackingQuality overall_quality = BAD;
  
  if(mdLastSigmaSquared < 5000)
  {
    for(unsigned i=0; i < mvAllCamNames.size(); ++i)
    {
      std::string camName = mvAllCamNames[i];
      TrackingQuality quality = AssessTrackingQuality(camName);
      mmTrackingQuality[camName] = quality;
    
      // TrackingQuality enums are ordered from BAD to GOOD, so GOOD is highest numerical value
      if(quality > overall_quality)
        overall_quality = quality;
    }
  }
  
  if(overall_quality == GOOD && mdLastSigmaSquared > 2000)
  {
    overall_quality = DODGY;
  }
  
  if(overall_quality == DODGY)
  {
    // Further heuristics to see if it's actually bad, not just dodgy...
    // If the camera pose estimate has run miles away, it's probably bad.
    if(mMapMaker.IsDistanceToNearestMultiKeyFrameExcessive(*mpCurrentMKF))
      overall_quality = BAD;
  }
  
  if(overall_quality==BAD)
  {
    mnLostFrames++;
    if(mnLostFrames > Tracker::snLostFrameThresh)
      mnLostFrames = Tracker::snLostFrameThresh;
  }
  else if(overall_quality == DODGY)
  {
    mnLostFrames--;
    if(mnLostFrames < 0)
      mnLostFrames = 0;
  }
  else if(overall_quality == GOOD)
  {
    mnLostFrames = 0;
  }
    
  mOverallTrackingQuality = overall_quality;
}


// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
Tracker::TrackingQuality Tracker::AssessTrackingQuality(std::string cameraName)
{
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;
  
  TrackingQuality quality;
  
  for(int i=0; i<LEVELS; i++)
  {
    nTotalAttempted += mmMeasAttemptedLevels[cameraName][i];
    nTotalFound += mmMeasFoundLevels[cameraName][i];
    if(i>=2) nLargeAttempted += mmMeasAttemptedLevels[cameraName][i];
    if(i>=2) nLargeFound += mmMeasFoundLevels[cameraName][i];
  }
  
  mnTotalAttempted = nTotalAttempted;
  mnTotalFound = nTotalFound;
  
  if(nTotalFound < Tracker::snMinPatchesPerFrame) // == 0 || nTotalAttempted == 0)
    quality = BAD;
  else
  {
    double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
    double dLargeFracFound;
    if(nLargeAttempted > Tracker::snCoarseMin)
      dLargeFracFound = (double) nLargeFound / nLargeAttempted;
    else
      dLargeFracFound = dTotalFracFound;

    if(dTotalFracFound > Tracker::sdTrackingQualityGood)
      quality = GOOD;
    else if(dLargeFracFound < Tracker::sdTrackingQualityBad)
      quality = BAD;
    else
      quality = DODGY;
  }
  
  return quality;
}

// Gets messages to be printed on-screen for the user
std::string Tracker::GetMessageForUser()
{
  return mMessageForUser.str();
}
/*
void Tracker::CalcSBIRotation(TooN::SE3<>& se3RelativePose, TooN::Matrix<6>& m6Covariance)
{
  // Use only the first camera of the current group being worked on
  // Could use all cameras and then average the result, but this works for now
  std::string& firstCamName = mvCurrCamNames[0];
  
  mmpSBILastFrame[firstCamName]->MakeJacs();
  std::pair<SE2<>, double> result_pair;
  result_pair = mmpSBIThisFrame[firstCamName]->IteratePosRelToTarget(*mmpSBILastFrame[firstCamName], 6);
  
  SE3<> se3AdjustCamFrame;
  Matrix<6> m6AdjustCov;
  SmallBlurryImage::SE3fromSE2(result_pair.first, mmCameraModelsSBI[firstCamName], mmCameraModelsSBI[firstCamName], se3AdjustCamFrame, m6AdjustCov);
  SE3<> se3AdjustBaseFrame = mpCurrentMKF->mmpKeyFrames[firstCamName]->mse3CamFromBase.get_rotation().inverse() * se3AdjustCamFrame;
  
  se3RelativePose = se3AdjustBaseFrame;
  m6Covariance = m6AdjustCov; 
}
*/

// Calculate the difference between the last pose and the current pose by comparing SmallBlurryImages
bool Tracker::CalcSBIRotation(TooN::Vector<3>& v3SBIRot)
{
  std::vector<Vector<3> > vRots;
  int nNumUsed = 0;
  for(unsigned i=0; i < mvCurrCamNames.size(); ++i)
  {
    std::string camName = mvCurrCamNames[i];
    
    if(mmTrackingQuality[camName] != GOOD)
      continue;
    
    mmpSBILastFrame[camName]->MakeJacs();
    std::pair<SE2<>, double> result_pair;
    result_pair = mmpSBIThisFrame[camName]->IteratePosRelToTarget(*mmpSBILastFrame[camName], 6);
    SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mmCameraModelsSBI[camName], mmCameraModelsSBI[camName]);
    
    Vector<3> v3AxisAngle_Cam = se3Adjust.get_rotation().ln();
    // Pose found was between KeyFrames, calculate effect on base pose
    Vector<3> v3AxisAngle_Base = mpCurrentMKF->mmpKeyFrames[camName]->mse3CamFromBase.get_rotation().inverse() * v3AxisAngle_Cam; // CHECK !! GOOD
    
    vRots.push_back(v3AxisAngle_Base);
    ++nNumUsed;
  }
  
  if(nNumUsed > 0)
  {
    v3SBIRot = FindAverageRotation(vRots);
    return true;
  }
  else
  {
    v3SBIRot = Zeros;
    return false;
  }
}

TooN::Vector<3> Tracker::FindAverageRotation(std::vector<TooN::Vector<3> >& vRots)
{
  double dEpsilon = 1e-3;
  SO3<> R = SO3<>::exp(vRots[0]);
  
  // Start iterative calculation of the rotation
  // Follows the algorithm of "Rotation Averaging with Application to Camera-Rig Calibration"
  // by Dai et. al. section "geodesic L2-mean"
  while(1)
  {
    Vector<3> r;
    r = Zeros;
    
    // Go through all MultiKeyFrames in the map
    for(unsigned i=0; i < vRots.size(); ++i)       
      r += (R.inverse() * SO3<>::exp(vRots[i])).ln();  // incorporate KeyFrame's pose
    
    r *= 1.0/vRots.size();
    
    if(r*r < dEpsilon*dEpsilon)  // converged, so get out of here
      break;
      
    R = R * SO3<>::exp(r);  // update R
  }
  
  return R.ln();
}

// Get the image from the named camera
CVD::Image<CVD::byte> Tracker::GetKeyFrameImage(std::string camName, unsigned nLevel)
{
  KeyFramePtrMap::iterator kf_it = mpCurrentMKF->mmpKeyFrames.find(camName);
  ROS_ASSERT(kf_it != mpCurrentMKF->mmpKeyFrames.end());
  ROS_ASSERT(nLevel < LEVELS);
  
  KeyFrame& kf = *(kf_it->second);
  return kf.maLevels[nLevel].image;
}

std::vector<TooN::Vector<2> > Tracker::GetKeyFrameSimpleMeas(std::string camName, unsigned nLevel)
{
  ROS_ASSERT(nLevel < LEVELS);
  
  if(!mmSimpleMeas.count(camName))
    return std::vector<TooN::Vector<2> >();
  
  return mmSimpleMeas[camName][nLevel];
}

// Copy the scene depth from the given MKF, used when the map maker take our MKF and we want to regenerate our scene depths
void Tracker::CopySceneDepths(MultiKeyFrame& mkf)
{
  mpCurrentMKF->mdTotalDepthMean = mkf.mdTotalDepthMean;
  for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
  {
    std::string camName = kf_it->first;
    mpCurrentMKF->mmpKeyFrames[camName]->mdSceneDepthMean = kf_it->second->mdSceneDepthMean;
    mpCurrentMKF->mmpKeyFrames[camName]->mdSceneDepthSigma = kf_it->second->mdSceneDepthSigma;
  }
}

// Find the points "nearest" to the current keyframe
void Tracker::CollectNearestPoints(KeyFrame& kf, std::set<MapPoint*>& spNearestPoints)
{
  if(Tracker::sbCollectAllPoints)
  {
    // This version just gets all map points
    spNearestPoints.clear();
    for(MapPointPtrList::iterator it = mMap.mlpPoints.begin(); it != mMap.mlpPoints.end(); ++it)
    {
      spNearestPoints.insert(*it);
    }
  }
  else
  {
    /*
    // This version finds the 3 nearest keyframes and collects the map points seen by those keyframes
    std::vector<KeyFrame*> vpNearest = mMapMaker.NClosestKeyFrames(kf, 3);
    
    spNearestPoints.clear();
    // Go through the collected nearby KFs
    for(unsigned i=0; i < vpNearest.size(); ++i)
    {
      KeyFrame& kfOther = *(vpNearest[i]);
      
      boost::mutex::scoped_lock lock(kfOther.mMeasMutex);
       
      for(MeasPtrMap::iterator meas_it = kfOther.mmpMeasurements.begin(); meas_it != kfOther.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
        spNearestPoints.insert(&point);
      }
    }
    */
    
    // This version gets the map points seen by the closest keyframe, then looks for keyframes that see those points, and collects
    // all other points also seen by those keyframes
    KeyFrame* pNearestKF = mMapMaker.ClosestKeyFrame(kf);  // region flag doesn't make a difference since kf not in map
    
    ROS_ASSERT(pNearestKF != NULL);
    
    std::set<KeyFrame*> spNearestNeighborKFs;
     
    boost::mutex::scoped_lock lockNearest(pNearestKF->mMeasMutex);
    
    // Go through nearest KF's measurements
    for(MeasPtrMap::iterator meas_it = pNearestKF->mmpMeasurements.begin(); meas_it != pNearestKF->mmpMeasurements.end(); ++meas_it)
    {
      // For each measured point
      MapPoint& point = *(meas_it->first);  
      // Collect all KF's that measure this point
      spNearestNeighborKFs.insert(point.mMMData.spMeasurementKFs.begin(), point.mMMData.spMeasurementKFs.end());  
    }
     
    lockNearest.unlock();
    
    spNearestPoints.clear();
    // Go through the collected nearby KFs
    for(std::set<KeyFrame*>::iterator kf_it = spNearestNeighborKFs.begin(); kf_it != spNearestNeighborKFs.end(); ++kf_it)
    {
      KeyFrame& kfOther = *(*kf_it);
      
      boost::mutex::scoped_lock lock(kfOther.mMeasMutex);
      
      for(MeasPtrMap::iterator meas_it = kfOther.mmpMeasurements.begin(); meas_it != kfOther.mmpMeasurements.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
        spNearestPoints.insert(&point);
      }
    }
  }
}

