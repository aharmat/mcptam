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
 * \file MapMakerClientBase.h
 * \brief Declaration of MapMakerClientBase interface
 *
 * Large parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * The MapMaker of PTAM has been broken up into several different pieces, in order to allow
 * the construction of both a standalone MapMaker as well as a client/server MapMaker with
 * a minimum amount of code duplication that would promote confusion and inconsistency. The 
 * inheritance tree looks as follows:
 * 
 *                             MapMakerBase
 *                            /          \
 *             MapMakerClientBase      MapMakerServerBase
 *              /              \        /              \
 *    MapMakerClient            MapMaker            MapMakerServer
 * 
 * The "client-side" interfaces with the tracker, whereas the "server-side"
 * actually does the optimizations. Both sides can delete map points.
 * 
 * MapMakerClientBase is an abstract base class that specifies the interface
 * that the Tracker interacts with, ie accepting new keyframes and checking
 * to see if a new keyframe is needed.
 *
 ****************************************************************************************/

#ifndef __MAP_MAKER_CLIENT_BASE_H
#define __MAP_MAKER_CLIENT_BASE_H

#include <mcptam/MapMakerBase.h>
#include <mcptam/ChainBundleIncrementalCovariance.h>
#include <mcptam/Tracker.h>  //just for data types
#include <mcptam/TrackerData.h> //just for data types
#include <deque>
#include <unordered_set>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <cvd/image.h>
#include <TooN/se3.h>

struct CovHelper
{
  CovHelper() : mpCovBundle(NULL) { }
  ~CovHelper(){ 
    if(mpCovBundle)
      delete mpCovBundle;
  }
  
  ChainBundleIncrementalCovariance* mpCovBundle;
  std::map<MapPoint*, int> mmPoint_BundleID;      ///< %Map FROM MapPoint* TO point id
  std::map<std::string, int> mmCamName_BundleID;  ///< %Map FROM camera name TO camera id
  int mnTracker_BundleID;
};


/** @brief An abstract base class that specifies the interface used by the Tracker to interact with
 *         any map maker.
 * 
 * The inheritance of MapMakerClientBase from MapMakerBase is virtual so that only one copy of
 * MapMakerBase is included in MapMaker. See the "dreaded diamond" pattern evident in the inheritence
 * hierarcy of MapMaker. */
class MapMakerClientBase : public virtual MapMakerBase
{
public:

  /** @brief Need to call constructor with Map as argument
   *  @param map The Map being worked on */
  MapMakerClientBase(Map &map, TaylorCameraMap &cameras);
  
  /// Destructor
  virtual ~MapMakerClientBase(){ };
  
  /// The number of MultiKeyFrames sitting in the queue from the Tracker
  /** @return The queue size */
  int TrackerQueueSize() 
  { 
    boost::mutex::scoped_lock lock(mQueueMutex);
    return mqpMultiKeyFramesFromTracker.size();
  }
  
  /// Checks to see if the given MultiKeyFrame is a candidate to be added to the Map
  /** @param mkf The MultiKeyFrame to check
   *  @return Should this MultiKeyFrame be added? */
  bool NeedNewMultiKeyFrame(MultiKeyFrame &mkf); 
  
  /** @brief Checks to see if the given MultiKeyFrame is a candidate to be added to the Map
   *
   *  This version of the function uses the number of measurements the MKF is capable of making in deciding if it's time to
   *  add new data. If the number of measurements is below a threshold computed from the average number of measurements of the
   *  MKF's nearest neighbors, then it's time to add. In order to save time, the tracker doesn't make measurement structures for
   *  potential measurements, which is what we need to check here, so this number is passed in as a separate parameter.
   *  @param mkf The MultiKeyFrame to check
   *  @param nNumMeas The number of measurements the MKF we are checking could make
   *  @return Should this MultiKeyFrame be added? */
  bool NeedNewMultiKeyFrame(MultiKeyFrame &mkf, int nNumMeas);
  
  // TESTING
  bool NeedNewMultiKeyFrame(MultiKeyFrame &mkf, TooN::Matrix<6> m6Cov);
  
  // testing
  bool NeedNewMultiKeyFrame(MultiKeyFrame &mkf, double dMAD);
  
  // testing
  TooN::Matrix<6> GetTrackerCov(TooN::SE3<> se3BaseFromWorld, std::vector<std::string>& vCamNames, std::vector<TrackerDataPtrVector>& vIterationSets);
  TooN::Matrix<6> GetTrackerCovFull(TooN::SE3<> se3BaseFromWorld, std::vector<std::string>& vCamNames, std::vector<TrackerDataPtrVector>& vIterationSets);
  
  // testing
  void UpdateCrossCovariances(std::unordered_set<MapPoint*>& spParticipatingPoints, ros::Duration allowedDur, double dPriorityThresh=0);
  //void UpdateCrossCovariances2(std::unordered_set<MapPoint*> spParticipatingPoints, ros::Duration allowedDur, double dPriorityThresh=0);
  void SavePointCovMatrix(std::string fileName);
  void SavePointCovMatrix(std::string fileName, std::unordered_set<MapPoint*> spParticipatingPoints);
  
  /// Checks to see if the given KeyFrame is a candidate to be added to the Map
  /** @param kf The KeyFrame to check
   *  @return Should this KeyFrame be added? */           
  bool NeedNewKeyFrame(KeyFrame &kf, bool bSameCamName = false);
  
  /// Checks if the nearest MKF is too far away
  /** @param mkf The MultiKeyFrame to check
   *  @return Is the given MKF far away from the nearest other MKF (i.e. maybe lost?) */
  bool IsDistanceToNearestMultiKeyFrameExcessive(MultiKeyFrame &mkf); 
   
  /// Checks if the nearest KF is too far away
  /** @param kf The KeyFrame to check
   *  @return Is the KF far away from the nearest other KF (i.e. maybe lost?) */
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kf);
  
  //---------------------------------------- Override me! -----------------------------------------------
  /// Override this to implement adding a MultiKeyFrame to the internal queue. Called by the Tracker.
  /** @param pMKF_Incoming Pointer to the MultiKeyFrame being added */
  virtual void AddMultiKeyFrame(MultiKeyFrame*& pMKF_Incoming) = 0;  
  
  //---------------------------------------- Override me! -----------------------------------------------
  /** @brief Override this to implement initializing the Map from a MultiKeyFrame. Called by the Tracker.
   * 
   *  This initialization method assumes that the given MultiKeyFrame contains at least two KeyFrames.
   *  @param pMKF_Incoming Pointer to the MultiKeyFrame being used to initialize Map */
  virtual bool Init(MultiKeyFrame*& pMKF_Incoming, bool bPutPlaneAtOrigin) = 0;
  
  //---------------------------------------- Override me! -----------------------------------------------
  /// Override this to implement signaling a reset. Called by the Tracker.
  virtual void RequestReset() = 0;
  
  // Static members
  static int snMinOutliers;           ///< Minimum number of times a point needs to be seen as an outlier to be considered for removal
  static double sdOutlierMultiplier;  ///< A point needs to be seen as an outlier this many more times than an inlier to be considered for removal
  static double sdMaxScaledMKFDist;   ///< Maximum permissible depth-scaled distance between MultiKeyFrames before a new one should be added
  static double sdMaxScaledKFDist;    ///< Maximum permissible depth-scaled distance between KeyFrames before a new one should be added
  
protected:
  
  //---------------------------------------- Override me! -----------------------------------------------
  /// Override this to implement adding a MultiKeyFrame from the internal queue to the Map.
  /// Called by the run() function of derived classes.
  virtual void AddMultiKeyFrameFromTopOfQueue() = 0;  
  
  /// Empty the internal MultiKeyFrame queue, derived classes calling this should also call MapMakerBase's Reset function
  void Reset();
  
  /// Points that are considered outliers by the Tracker are marked with a bad flag
  void MarkOutliersAsBad();
  
  /// Points that have fewer than two measurements are marked as bad. If the point is fixed, one measurement is enough.
  void MarkDanglersAsBad();
  
  /** @brief Child KeyFrames that don't have the mbActive flag set are deleted. KeyFrame's images are made unique/
   * 
   *  The Tracker might not have filled out all child KeyFrames if it did not receive images
   *  from all of them. Those that weren't filled out have their mbActive flag set to false.
   *  @param mkf The MultiKeyFrame to process */
  void ProcessIncomingKeyFrames(MultiKeyFrame &mkf);
  
  /** @brief Find the closest MKF in the internal queue to the supplied one
   *
   * @param mkf The MultiKeyFrame to find the closest MKF in the queue to */
  MultiKeyFrame* ClosestMultiKeyFrameInQueue(MultiKeyFrame &mkf);
  
  /// Called when initialization phase has finished, clears incoming queue
  void ClearIncomingQueue();
  
  void LoadCovBundle();
  
  void ComputeSelectedPointsCrossCov();
   
  std::deque<MultiKeyFrame*> mqpMultiKeyFramesFromTracker;  ///< Queue of MultiKeyFrames from the tracker waiting to be processed
  boost::mutex mQueueMutex;     ///< Mutex to protect the MKF queue
  
  int mnCurrCov;
  std::vector<CovHelper> mvCovHelpers;
  boost::mutex mCovMutex;
};

#endif

