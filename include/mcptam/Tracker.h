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


/****************************************************************************************
 *
 * \file Tracker.h
 * \brief Declaration of Tracker class
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * This header declares the Tracker class.
 * The Tracker is one of main components of the system,
 * and is responsible for determining the pose of a camera
 * from a video feed. It uses the Map to track, and communicates 
 * with the MapMaker (which runs in a different thread)
 * to help construct this map.
 *
 * Externally, the tracker should be used by calling TrackFrame()
 * with every new input video frame. This then calls either 
 * TrackForInitialMap() or TrackMap() as appropriate.
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * Tracker has been modified in two ways: it now supports an unlimited number of cameras
 * for pose estimation, and it doesn't do trail tracking for Map initialization, using
 * instead the stereo effect of multiple KeyFrames in a MultiKeyFrame. In the case
 * that there is no stereo effect because the KeyFrames' views don't overlap, there
 * is still the possibility of generating map points using the initial depth approach
 * (see MapMakerServerBase::InitFromMultiKeyFrame)
 *
 * The original TrackMap function has been broken into several logical parts both to aid
 * readibility and to allow easy code reuse for tracking multiple cameras
 *
 ****************************************************************************************/

#ifndef __TRACKER_H
#define __TRACKER_H

#include <mcptam/Types.h>
#include <mcptam/Relocaliser.h>
#include <mcptam/KeyFrame.h>  // needed for LEVELS define
#include <mcptam/GLWindow2.h>
#include <mcptam/TrackerTiming.h>
#include <mcptam/TrackerCovariance.h>
#include <sstream>
#include <vector>
#include <TooN/TooN.h>
#include <ros/ros.h>

class Map;
class MapMakerClientBase;

/// Can't put a LEVEL-sized array of objects into another STL container, so use this intermediary
template<typename T>
struct LevelsArray
{
  inline T& operator[] (unsigned i)
  {
    return array[i];
  }
  
  T array[LEVELS];
};

typedef LevelsArray<TrackerDataPtrVector> TDVLevels;
typedef LevelsArray<int> IntLevels;
typedef LevelsArray<std::vector<TooN::Vector<2> > > V2Levels;

typedef std::map<std::string, IntLevels > IntLevelsMap;
typedef std::map<std::string, V2Levels > V2LevelsMap;

/** @brief Determines the pose of the system from stream of camera images
 * 
 *  The current pose of the system is represented by a MultiKeyFrame, which contains KeyFrames that
 *  process the images coming from all the cameras. Although it is called a MultiKeyFrame, strictly
 *  speaking it shouldn't be called that because it's not part of the Map. However, the needs of a class
 *  that would perform this role in the Tracker is virtually the same as the MultiKeyFrame, so why
 *  build another class just so it can be called something else? 
 * 
 *  Uses MultiKeyFrames and MapPoints in the Map to calculate the pose of the current MultiKeyFrame. 
 *  Communicates with a map maker to help build the Map as the environment is explored */
class Tracker
{
public:

  enum TrackingQuality{NONE, BAD, DODGY, GOOD};
  
  enum RunMode{NORMAL, TRIALS};
  
  /** @param map The Map
   *  @param mapmaker One of MapMaker or MapMakerClient, which share the MapMakerClientBase parent class
   *  @param cameras All camera models in the system
   *  @param poses The fixed poses of the cameras relative to a base frame
   *  @param offsets The drawing offsets for the images, determines layout of video stream drawn to window
   *  @param pWindow Pointer to the OpenGL window */
  Tracker(Map &map, MapMakerClientBase &mapmaker, TaylorCameraMap &cameras, SE3Map poses, ImageRefMap offsets, GLWindow2* pWindow); 
  
  /// Destructor
  ~Tracker();
  
  /** @brief The main working part of the tracker, call this every frame
   *  @param imFrames A map of camera name => CVD::Images containing the latest image acquisition. Doesn't need to contain images from all cameras
   *  @param bDraw Should I draw the images and detected corner features? */
  void TrackFrame(ImageBWMap& imFrames, ros::Time timestamp, bool bDraw);
  
  /// Set the feature extraction masks for the camera images
  void SetMasks(ImageBWMap& imMasks);
  
  void SetCurrentPose(TooN::SE3<> se3Pose);
  
  /// Get the current poses of the cameras
  SE3Map GetCurrentCameraPoses();

  /** @brief Get the pose of the base frame in the system
   *  @return The pose */
  inline TooN::SE3<> GetCurrentPose() { return mpCurrentMKF->mse3BaseFromWorld; }
  
  /** @brief Get the current pose covariance
   *  @return The pose covariance */  
  inline TooN::Matrix<6> GetCurrentCovariance(int nCovType) 
  { 
    if(nCovType < 0)
      return mm6PoseCovarianceOld;
    else if(nCovType > 0)
      return mm6PoseCovarianceExperimental;
    else
      return mm6PoseCovariance;
  }
  
  inline ros::Duration GetCovarianceDuration() { return mCovDur; }
  
  inline int GetMeasNum() { return mnMeasNum; }
  
  /** @brief Get the Frobenius norm of the current pose covariance
   *  @return The pose covariance norm */  
  inline double GetCurrentCovarianceNorm() { return norm_fro(mm6PoseCovariance) * ( mnTotalAttempted/(double)mnNumInliers );  }
  
  /** @brief Get the current MultiKeyFrame
   *  @return The current MKF */  
  inline MultiKeyFrame& GetCurrentMKF(){ return *mpCurrentMKF; }
  
  /** @brief Get the current timestamp
   *  @return The current timestamp */  
  inline ros::Time GetCurrentTimestamp(){ return mLastProcessTime; }
  
  /** @brief Gets messages to be printed on-screen for the user
   *  @return The message string */
  std::string GetMessageForUser();
  
  /** @brief Get the image from the named camera
   *  @param camName The name of the camera
   *  @param nLevel The pyramid level of the image to get
   *  @return The image from the camera */
  CVD::Image<CVD::byte> GetKeyFrameImage(std::string camName, unsigned nLevel);
  
  /** @brief Get the points corresponding to the measurements in a camera at a certain level
   *  @param camName The name of the camera
   *  @param nLevel The pyramid level to get measurements from
   *  @return Vector of measurement points (IN LEVEL ZERO COORDINATES!) */
  std::vector<TooN::Vector<2> > GetKeyFrameSimpleMeas(std::string camName, unsigned nLevel);
  
  /** @brief Get the current tracking quality metric
   *  @return The tracking quality */
  TrackingQuality GetTrackingQuality(){ return mOverallTrackingQuality; }
  
  /** @brief Check if the Tracker is lost
   *  @return Tracker is lost */
  bool IsLost(){ return mnLostFrames >= Tracker::snLostFrameThresh; }
  
  /// Clear all internal variables, reset the Map
  void Reset(bool bSavePose, bool bResetMap);
  
  /// Signal a request for initialization
  void RequestInit(bool bPutPlaneAtOrigin);
  
  //testing
  /// Set the addnext flag to true if map is good and we are not lost, will add subsequent MKF to map
  /// regardless of other metrics
  void AddNext();
  
  void StartTrials(int nMaxTrials);
  
  bool NextTrial();
  
  void DoCovAnalysis();
  
  // Static members
  static double sdRotationEstimatorBlur; ///< Amount of blur when constructing SmallBlurryImage
  static bool sbUseRotationEstimator; ///< Should pre-estimate the rotation using SmallBlurryImages
  static bool sbDrawFASTCorners;      ///< Draw the detected FAST corners on the GUI
  static int snMaxPatchesPerFrame;    ///< Maximum number of patches to search for features per frame
  static int snMinPatchesPerFrame;    ///< Minimum number of patches that must be found, or it's lost
  static int snCoarseMin;              ///< Min number of large-scale features for coarse stage
  static int snCoarseMax;              ///< Max number of large-scale features for coarse stage
  static int snCoarseRange;            ///< Pixel search radius for coarse features
  static int snCoarseSubPixIts;        ///< Max sub-pixel iterations for coarse features
  static bool sbDisableCoarse;        ///< Set this to true to disable coarse stage (except after recovery)
  static double sdCoarseMinVelocity;   ///< Speed above which coarse stage is used
  static std::string sMEstimatorName;  ///< One of Tukey, Cauchy, Huber
  static double sdTrackingQualityGood; ///< If fraction of potentially visible points actually founds exceeds this, tracking quality is good
  static double sdTrackingQualityBad; ///< If fraction of potentially visible points actually founds is below this, tracking quality is bad
  static int snLostFrameThresh;       ///< Cap the number of frames where tracker is lost
  static bool sbCollectAllPoints;  ///< Should we project the whole map or do something smarter? (affects CollectNearestPoints)
  
  static double sdCrossCovDur;
  
protected:
  
  /** @brief Generate a new MultiKeyFrame with a given pose and its children KeyFrames with the fixed camera poses
   *  @param pose The pose of the MultiKeyFrame */
  void InitCurrentMKF(const TooN::SE3<>& pose);

  // Main tracking-related functions
  /** @brief Helper function that updates the current MultiKeyFrame and optionally draws the camera images
   * 
   *  This function exists separately from TrackFrame because Tracker and CalibratorTracker (which is a child of Tracker) need
   *  to perform the frame tracking somewhat differently, but they share common requirements encapsulated by this function.
   *  @param imFrames The input images
   *  @param bDraw Should I draw the images and corner features? */
  void TrackFrameSetup(ImageBWMap& imFrames, ros::Time timestamp, bool bDraw); 

  /// Calls the map maker's initialization routine with the current MultiKeyFrame
  void TrackForInitialMap();      

  /// Updates the pose of the current MultiKeyFrame by tracking MapPoints in the Map
  void TrackMap();          
  
  // Functions used by TrackMap    
  /** @brief Find the points "nearest" to the current keyframe
   * 
   *  How "nearest" is defined is very much up for interpretation, see code for some ways I've tried in the past
   *  @param kf The keyframe to look around
   *  @param [out] spNearestPoints Set of pointers to nearby map points */
  void CollectNearestPoints(KeyFrame& kf, std::set<MapPoint*>& spNearestPoints);
  
  /** @brief Finds the Potentialy Visible Set, ie TrackerDatas referencing MapPoints that should be seen from one camera
   *  @param cameraName The name of the camera (and the KeyFrame that's carrying its pose info) we're processing
   *  @param [out] vPVSLevels The potentially visible set, split into LEVEL pyramid levels */
  void FindPVS(std::string cameraName, TDVLevels& vPVSLevels);
  
  /** @brief Gather some TrackerDatas from coarse levels (ie high pyramid level number), try to find them and return the number found
   *  @param [in,out] vPVSLevels The potentially visible set, split into LEVEL pyramid levels, for one camera
   *  @param cameraName The name of the camera being processed
   *  @param nCoarseRange The pixel search radius for coarse features
   *  @param nCoarseMax The maximum number of coarse features to use
   *  @param nCoarseSubPixIts The maximum number of sub-pixel iterations when finding coarse features
   *  @param [out] vIterationSet The set of TrackerDatas that were used for coarse search 
   *  @return The number of coarse features found */
  int TestForCoarse(TDVLevels& vPVSLevels, std::string cameraName, unsigned int nCoarseRange, unsigned int nCoarseMax, int nCoarseSubPixIts, TrackerDataPtrVector& vIterationSet);
  
  /** @brief Try to find the points references by the TrackerDatas of the input in the current image
   *  @param [in,out] vTD Vector of TrackerDatas to find, NOT split into levels
   *  @param cameraName The name of the camera being processed
   *  @param nRange The search radius
   *  @param nSubPixIts The number of sub-pixel iterations to perform
   *  @return The number of points found */
  int SearchForPoints(TrackerDataPtrVector& vTD, std::string cameraName, int nRange, int nSubPixIts, bool bExhaustive=false);
  
  /** @brief Collect more TrackerDatas at fine levels (low pyramid number), try to find them, add them to the ones found by TestForCoarse
   *  @param vPVSLevels The potentially visible set, split into LEVEL pyramid levels, for one camera
   *  @param [in,out] vIterationSet The set of TrackerDatas that will be used for fine pose search, on input it is filled with those found by TestForCoarse and this function adds to it
   *  @param cameraName The name of the camera being processed
   *  @param bDidCoarse Was a coarse stage performed before this? Changes the search radius of features. */
  void SetupFineTracking(TDVLevels& vPVSLevels, TrackerDataPtrVector& vIterationSet, std::string cameraName, bool bDidCoarse);
  
  /** @brief Takes one nonlinear step in updating the pose of the current MultiKeyFrame, using the TrackerDatas collected by (potentially) TestForCoarse and SetupFineTracking
   *  @param vIterationSets The set of ALL TrackerDatas collected, for all cameras
   *  @param nIter The number of the current iteration, controls when to reproject and MEstimator severity (harsh after 5 iterations)
   *  @param dOverrideSigma Will be used to override the MEstimator sigma in CalcPoseUpdate after the 5th iteration
   *  @param bMarkOutliers Passed through to CalcPoseUpdate
   *  @return The update vector that was applied to the current pose */
  TooN::Vector<6> PoseUpdateStep(std::vector<TrackerDataPtrVector>& vIterationSets, int nIter, double dOverrideSigma, bool bMarkOutliers);
  
  /** @brief Takes one linear step in updating the pose of the current MultiKeyFrame, using the TrackerDatas collected by (potentially) TestForCoarse and SetupFineTracking
   *  @param vIterationSets The set of ALL TrackerDatas collected, for all cameras
   *  @param v6LastUpdate The update vector applied at the last step, will be used to update the projected location of a MapPoint in a TrackerData rather than reprojecting
   *  @param nIter The number of the current iteration, controls when to reproject and MEstimator severity (harsh after 5 iterations)
   *  @param dOverrideSigma Will be used to override the MEstimator sigma in CalcPoseUpdate after the 5th iteration
   *  @param bMarkOutliers Passed through to CalcPoseUpdate
   *  @return The update vector that was applied to the current pose */
  TooN::Vector<6> PoseUpdateStepLinear(std::vector<TrackerDataPtrVector>& vIterationSets, TooN::Vector<6>& v6LastUpdate, int nIter, double dOverrideSigma, bool bMarkOutliers);
  
  /** @brief Calculate a pose update 6-vector from a bunch of image measurements using weighted least squares.
   * 
   * User-selectable M-Estimator. Normally this robustly estimates a sigma-squared for all the measurements
   * to reduce outlier influence, but this can be overridden. 
   * @param vIterationSets The set of ALL TrackerDatas collected, for all cameras
   * @param dOverrideSigma If positive, this value is used instead of the MEstimator sigma to calculate weights
   * @param bMarkOutliers If true, MapPoints associated with TrackerDatas whose MEstimator weight is zero will be marked as an outlier, and as an inlier if the weight is positive
   * @return The update vector found */
  TooN::Vector<6> CalcPoseUpdate(std::vector<TrackerDataPtrVector>& vIterationSets, double dOverrideSigma = 0.0, bool bMarkOutliers=false); 
  
  /** @brief Update the scene depth variables of the current MultiKeyFrame and KeyFrames without recording any measurements
   * 
   *  The scene depth is used in the distance calculators, which are used to figure out if a new MultiKeyFrame is needed, so even if the 
   *  MultiKeyFrame isn't going to be added to the Map the scene depth needs to be calculated.
   *  @param vIterationSets The set of ALL TrackerDatas collected, for all cameras */
  void RefreshSceneDepth(std::vector<TrackerDataPtrVector>& vIterationSets);
  
  /** @brief Save a simplified copy of the measurements made during tracking. Can be retrieved with GetKeyFrameSimpleMeas,
   *  used only for visualization */
  void SaveSimpleMeasurements(std::vector<TrackerDataPtrVector>& vIterationSets);
  
  // Functions used when adding a MultiKeyFrame to the Map
  /** @brief Create measurements of the MapPoints referenced in mvIterationSets, add them to the current KeyFrames
   * 
   * This function is called only once we've decided to add the current MultiKeyFrame to the Map, and just before the
   * MapMaker function to do this is called */
  void RecordMeasurements();
  
  /** @brief Count the number of measurements we could make based on the size of mvIterationSets
   *  @return The number of measurements */
  int CountMeasurements();
  
  /// Gives the current MultiKeyFrame to the map maker. The current MultiKeyFrame needs to be regenerated.
  void AddNewKeyFrame();       
  
  /// Clears mvIterationSets, which causes the "using" count of MapPoints to decrement as the boost intrusive pointers destruct   
  void ReleasePointLock();
  
  /// Copy masks from the given MKF to the current MKF, used when map maker takes our MKF and we need to regenerate our own masks
  void CopyMasks(MultiKeyFrame& mkf);
  
  /// Copy the scene depth from the given MKF, used when the map maker take our MKF and we want to regenerate our scene depths
  void CopySceneDepths(MultiKeyFrame& mkf);
  
  
  // Functions called by TrackFrame
  /** @brief If tracking is lost, try to find the current pose through another means
   * 
   * Uses the Relocalizer to find the best guess at the current pose.
   * @return Was the recovery attempt successful? */
  bool AttemptRecovery(); 
  
  // Tracking quality functions
  /// Find the tracking quality of the system by taking the top quality out of all the cameras
  void AssessOverallTrackingQuality();
  
  /** @brief Assess the tracking quality of a specific camera
   * 
   *  Uses heuristics to choose between BAD, DODGY, and GOOD
   *  @param cameraName The camera whose tracking quality is assessed */
  TrackingQuality AssessTrackingQuality(std::string cameraName);   
  
  // Updating values
  /** @brief A decaying velocity motion model is applied prior to calling TrackMap
   * 
   * Calculate the assumed motion based on the last known velocity and the last time step 
   * and add it to the current pose to get an updated pose */
  void ApplyMotionModel();        
  
  /// The motion model is updated after TrackMap has found a new pose
  void UpdateMotionModel();       
  
  /// Update the camera-from-world poses of the current KeyFrames, based on the current MultiKeyFrame's pose and the fixed relative transforms
  void UpdateCamsFromWorld();
  
  /// Calculate the difference between the last pose and the current pose by comparing SmallBlurryImages
  /** @return The pose difference as a 6-vector */
  bool CalcSBIRotation(TooN::Vector<3>& v3SBIRot);
  //void CalcSBIRotation(TooN::SE3<>& se3RelativePose, TooN::Matrix<6>& m6Covariance);
  
  /// Find average rotation of arguments using L2 norm
  /** @return The average rotation */
  TooN::Vector<3> FindAverageRotation(std::vector<TooN::Vector<3> >& vRots);
  
  // Drawing functions
  /// Draws the reference grid at z=0 as an overlay
  void RenderGrid(); 
  
  void InitTrial();
  
  double CalcMAD();
  
  TooN::Matrix<6> CalcCovariance(std::vector<TrackerDataPtrVector>& vIterationSets, bool bFull, std::string fileName=std::string());
         
  
  MultiKeyFrame* mpCurrentMKF;       ///< The current processing space as a MultiKeyFrame
  
  // The major components to which the tracker needs access:
  Map &mMap;                                ///< The Map, consisting of points, multikeyframes and keyframes
  MapMakerClientBase &mMapMaker;              ///< The class which maintains the map
  TaylorCameraMap mmCameraModels;             ///< Camera projection models
  TaylorCameraMap mmCameraModelsSBI;          ///< Camera projection models again, their image sizes will be resized by SmallBlurryImage
  Relocaliser mRelocaliser;                   ///< Relocalisation module
  
  TooN::Matrix<6>	mm6PoseCovariance;		///< covariance of current pose estimate
  TooN::Matrix<6>	mm6PoseCovarianceExperimental;		///< covariance of current pose estimate
  TooN::Matrix<6>	mm6PoseCovarianceOld;
  ros::Duration mCovDur;
  int mnMeasNum;
  
  TooN::SE3<> mse3StartPose;             ///< The pose of the system at the start of processing a new set of images
  TooN::Vector<6> mv6BaseVelocity;    ///< The 6-vector representation of pose differences between the current and previous poses, divided by the time step
  double mdMSDScaledVelocityMagnitude;  ///< Velocity magnitude of base
  bool mbDidCoarse;               ///< Did tracking use the coarse tracking stage?
  
  ImageRefMap mmSizes;            ///< Image sizes of all cameras, used for drawing and TrackerData initialization
  ImageRefMap mmDrawOffsets;      ///< Offsets for each camera for drawing images to window
  bool mbDraw;                    ///< Should the tracker draw anything to OpenGL?
  
  int mnFrame;                    ///< Frames processed since last reset
  int mnLastMultiKeyFrameDropped;      ///< At which frame number was the last MultiKeyFrame inserted into the Map?
  ros::Time mtLastMultiKeyFrameDropped;   ///< At what time was the last MultiKeyFrame inserted into the Map?
  
  // Tracking quality control:
  IntLevelsMap mmMeasAttemptedLevels;   ///< The number of measurements attempted, at each level, for each camera
  IntLevelsMap mmMeasFoundLevels;       ///< The number of measurements found, at each level, for each camera
  TrackingQuality mOverallTrackingQuality;     ///< The overall tracking quality
  std::map<std::string, TrackingQuality> mmTrackingQuality;
  int mnLostFrames;                     ///< The number of frames where we are lost, reset to zero upon recovery
  bool mbJustRecoveredSoUseCoarse;      ///< Always use coarse tracking after recovery!

  std::vector<TrackerDataPtrVector> mvIterationSets;   ///< Vector of TrackerData pointers, one for each camera, filled in TrackMap and used to calculate a new pose
  V2LevelsMap mmSimpleMeas;                        ///< %Map of camera names to 2-vectors at LEVELS levels, holds the last measurements in simplified form (used for visualization only)

  // Frame-to-frame motion init:
  std::map<std::string, SmallBlurryImage*> mmpSBILastFrame;   ///< SmallBlurryImage of the last image processed, space for all the cameras but only first of each group used
  std::map<std::string, SmallBlurryImage*> mmpSBIThisFrame;   ///< SmallBlurryImage of the current image being processed, space for all the cameras but only first of each group used
  
  std::ostringstream mMessageForUser;   ///< Stream to collect messages for GUI
  
  ros::Time mLastProcessTime;           ///< Time that the previous image processing step began
  ros::Duration mLastProcessDur;        ///< Time since the previous image processing step start
  
  int mnTotalFound;                 ///< Number of features found by the tracker in the current frame
  int mnTotalAttempted;             ///< Number of features attempted to find by the tracker
  int mnNumInliers;                 ///< Number of measurement inliers for the tracking
  
  ImageBWMap mmMasks;               ///< Map of all of the camera image masks
  
  std::vector<std::string> mvAllCamNames;   ///< Names of all the cameras in the system
  std::vector<std::string> mvCurrCamNames;  ///< Names of the cameras that provided the most recent batch of images
  
  SE3Map mmFixedPoses;             ///< The fixed poses of the cameras relative to a base frame
  ros::NodeHandle mNodeHandle;     ///< ROS global node handle
  ros::NodeHandle mNodeHandlePrivate; ///< ROS private node handle
  
  bool mbInitRequested;  ///< Is initialization requested?
  bool mbPutPlaneAtOrigin;   ///< During initialization, should the dominant plane be detected and placed at the origin?
  
  GLWindow2* mpGLWindow;    ///< Pointer to the GUI window
  
  ros::Publisher maskPub;   ///< Publisher for the camera image masks
  ros::Publisher timingPub;     ///< Publisher for the Tracker timing messages
  mcptam::TrackerTiming timingMsg;  ///< Message for the various timing sections of the Tracker
  
  //testing
  bool mbAddNext;   ///< Add the next MKF now
  
  RunMode mRunMode;
  int mnTrialNumber;
  int mnMaxTrials;
  TooN::SE3<> mse3SavedPose;
  bool mbTrialAdding;
  
  TrackerCovariance mTrackerCovariance;
  ros::ServiceClient mPauseClient;
  bool mbDoAnalysis;
  
};

#endif

