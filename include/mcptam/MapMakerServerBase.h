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
 * \file MapMakerServerBase.h
 * \brief Declaration of MapMakerServerBase interface
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
 * MapMakerServerBase is an abstract base class that contains most functions required
 * for building and maintaining the map, as well as functions that need to be overriden
 * to implement handling bad points and checking the incoming queue size.
 *
 ****************************************************************************************/

#ifndef __MAP_MAKER_SERVER_BASE_H
#define __MAP_MAKER_SERVER_BASE_H

#include <mcptam/MapMakerBase.h>
#include <mcptam/Types.h>
#include <TooN/TooN.h>

class MapPoint;
class BundleAdjusterBase;

/** @brief An abstract base class that contains most functions required for building and maintaining the map.
 * 
 * The inheritance of MapMakerServerBase from MapMakerBase is virtual so that only one copy of
 * MapMakerBase is included in MapMaker. See the "dreaded diamond" pattern evident in the inheritence
 * hierarcy of MapMaker. */
class MapMakerServerBase : public virtual MapMakerBase
{
public:

  /** @brief Need to call constructor with arguments to ensure valid object
   *  @param map The Map being worked on
   *  @param cameras The camera models
   *  @param bundleAdjuster Some derived class of BundleAdjusterBase that will be used to optimize the map */
  MapMakerServerBase(Map &map, TaylorCameraMap &cameras, BundleAdjusterBase& bundleAdjuster);
  
  /// Destructor  
  virtual ~MapMakerServerBase();
  
  // Static members
  static int snMinMapPoints;              ///< Minimum number of points to make a map with
  static int snMaxConsecutiveFailedBA;    ///< Number of times BA can fail before a reset is requested
  static int snMaxTriangulationKFs;       ///< Max number of keyframes to use for epipolar point adding
  static int snMaxInitPointsLevelZero;    ///< Max number of points to add at pyramid level zero (scaled by 1/2 for each further level)
  static double sdInitDepth;              ///< Initial radial depth to initialize non-triangulated points to
  static std::string ssInitPointMode;             ///< Point initialization mode -- 0: stereo only, 1: idp only, 2+: both
  static double sdInitCovThresh;          ///< Feature position covariance threshold to end initialization phase
  static bool sbLargePointTest;
  
protected:

  /** @brief Called when the map maker decides the map has gone bad, signals the System's reset service
   * 
   *  The System then call's the Tracker's Reset function, which requests the map maker to stop.
   *  This circuitous route was chosen so that reset requests propagate only in one direction
   *  (away from the Tracker). Otherwise we'd have to figure out who called reset on us, and make
   *  sure not to call reset on them. Also, the map maker would have to know about the Tracker,
   *  which would then also need a thread-safe reset request mechanism. */
  void RequestResetInternal();

  /// Empty the internal queues, derived classes calling this should also call MapMakerBase's Reset function
  void Reset();
  
  // Functions for starting the map from scratch:
  /** @brief Build an initial map using epipolar matches between KeyFrames of a MultiKeyFrame
   * 
   *  This function is to be called from the function overriding MapMakerClientBase's Init(), after
   *  it has taken possession of the MKF. The map maker thread needs to have posession of the MKF because this function
   *  will simply push the MKF pointer to the map.
   *  @param pMKF Pointer to the MultiKeyFrame used for initialization
   *  @param bPutPlaneAtOrigin Should we compute a ground plane and transform the map to put it at z=0?
   *  @return Did the initialization succeed? If not, map maker should be reset */
  bool InitFromMultiKeyFrame(MultiKeyFrame *pMKF, bool bPutPlaneAtOrigin);
  
  /** @brief Compute the best fit plane to the current set of map points
   *  @return The transform that would put the center of the computed plane at the origin, with the z axis normal to the plane */
  TooN::SE3<> CalcPlaneAligner();
  
  /** @brief Apply a transform to all the MapPoints and MultiKeyFrames of the map
   *  @param se3NewFromOld The transform to apply */
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);
  
  /** @brief Apply a scaling to all the MapPoints and MultiKeyFrames of the map
   *  @param dScale The scale to apply */
  void ApplyGlobalScaleToMap(double dScale);
  
  // Map expansion functions:
  /** @brief Thins out a KeyFrame's candidate list
   * 
   *  Candidates are those salient corners where the MapMaker will attempt 
   *  to make a new map point by epipolar search. We don't want to make new points
   *  where there are already existing map points, this routine erases such candidates.
   *  Needs to be called every time we want to make new points from a KeyFrame, since 
   *  MapPoints might have been added since last time that invalidate some of the Candidates.
   *  Operates on a single level of a keyframe.
   *  @param kf The KeyFrame to operate on
   *  @param nLevel The image pyramid level to operate on */
  void ThinCandidates(KeyFrame &kf, int nLevel);
  
  /** @brief Adds map points by epipolar search to the last-added key-frame, at a single specified pyramid level.
   * 
   *  Finds the closest KeyFrame to use for triangulation by calling MapMakerBase's ClosestKeyFramesWithinDist()
   *  @param mkfSrc the source MKF
   *  @param nLevel The image pyramid level to work on
   *  @param nLimit The maximum number of points to make from each target KF at the given level
   *  @param dDistThresh The distance threshold for finding neighboring KFs
   *  @param region The region where the closest KeyFrame will be searched */
  void AddStereoMapPoints(MultiKeyFrame& mkfSrc, int nLevel, int nLimit, double dDistThresh, KeyFrameRegion region);
  
  /** @brief Adds map points at a given initial depth in a specified pyramid level
   *  @param mkfSrc the source MKF
   *  @param nLevel The image pyramid level to work on
   *  @param nLimit The maximum number of points to make at the given level
   *  @param dInitDepth The depth to set the points at */
  void AddInitDepthMapPoints(MultiKeyFrame& mkfSrc, int nLevel, int nLimit, double dInitDepth);
  
  /** @brief Tries to create a new MapPoint out of a Candidate in the source KeyFrame by patch matching it in the target KeyFrame
   * 
   *  The 3D position of the point is found from triangulation.
   *  @param kfSrc The source KeyFrame which has the Candidate being considered
   *  @param kfTarget The target KeyFrame where a match is sought
   *  @param nLevel The image pyramid level being worked on
   *  @param nCandidate The vector index of the Candidate in the source KeyFrame
   *  @return Was match found and a new MapPoint created? */
  bool AddPointEpipolar(KeyFrame &kfSrc, KeyFrame &kfTarget, int nLevel, int nCandidate);
  
  /** @brief Finds the 3D position of a point (in reference frame B) given the inputs
   * 
   *  Uses the method of linear triangulation, an example of which is given in Hartley & Zisserman's
   *  Multiple View Geometry book section 12.2. This is not an optimal estimate for the position of
   *  the point based on error minimization, but that's what bundle adjustment is for. Note that instead
   *  of dealing with 2D points in homogeneous form, here we are dealing with unit 3-vectors that work
   *  exactly the same in the equations. In MVG book, the matrix A becomes: 
   *  \f[ A = \left[ \begin{array}{cc} x\mathbf{p}^{3T} - z\mathbf{p}^{1T} \\ y\mathbf{p}^{3T} - z\mathbf{p}^{2T} \\
   *                                   x'\mathbf{p'}^{3T} - z'\mathbf{p'}^{1T} \\ y'\mathbf{p'}^{3T} - z'\mathbf{p'}^{2T} \end{array} \right] \f]
   *  @param se3AfromB The pose of reference frame A in reference frame B
   *  @param v3A The unit vector in frame A defining the half-line along which the point should lie
   *  @param v3B The unit vector in frame B defining the half-line along which the point should lie
   *  @return The position of the point */
  TooN::Vector<3> ReprojectPoint(TooN::SE3<> se3AfromB, const TooN::Vector<3> &v3A, const TooN::Vector<3> &v3B);
  
  /// Common code for adding a MultiKeyFrame to a map and generating new MapPoints through epipolar search
  /** @param pMKF Point to the MultiKeyFrame being added 
   *  @return Was the addition successful? */
  bool AddMultiKeyFrameAndCreatePoints(MultiKeyFrame *pMKF);
  
  /// Add a new MultikeyFrame to the map, erase the one at the end of the map.
  void AddMultiKeyFrameAndMarkLastDeleted(MultiKeyFrame *pMKF, bool bMakeRest);
  
  /// Mark the MKF which is furthest from the given one as bad and erase it
  void MarkFurthestMultiKeyFrameAsBad(MultiKeyFrame& mkf);

  // Data association functions:
  /** @brief Tries to find points in a KeyFrame that were never searched by the Tracker. Called when KeyFrame is first added to the Map.
   * 
   *  @param kf The KeyFrame to search
   *  @return The number of new points found */
  int ReFindInSingleKeyFrame(KeyFrame &kf);
  
  /// Measurements that were considered outliers get a second chance
  void ReFindFromFailureQueue();
  
  /// Tries to make measurements of new MapPoints in KeyFrames other than the pair they were generated from
  void ReFindNewlyMade();
  
  /** @brief Code common to all RefindXYZ functions, actually does the point finding
   * 
   *  This operates much like the tracker, so most of the code looks just like in TrackerData.h
   *  @param kf The KeyFrame to search in
   *  @param point The MapPoint to project
   *  @return Was the point found (and hence a measurement made) successfully? */
  bool ReFind_Common(KeyFrame &kf, MapPoint &point);
  
  //---------------------------------------- Override me! -----------------------------------------------
  /** @brief Override this to return the size of the queue considered to be the source of all data for the derived class. 
   *  
   *  For the standalone MapMaker, this is the queue of MultiKeyFrames coming from the tracker. For the 
   *  MapMakerServer, this is the incoming queue of the NetworkManager handling the connection to the client.
   *  This function is used to check if the ReFind functions should stop execution because there's more
   *  important things to worry about.
   *  @return The size of the queue */
  virtual int IncomingQueueSize() = 0;         
  
  // Dealing with bad points/measurements
  //---------------------------------------- Override me! -----------------------------------------------
  /// Override this to implement handling of points flagged as bad.
  virtual void HandleBadEntities() = 0;  
            
  /** @brief Marks points as bad or puts them in the failure queue for a second chance depending on where the point came from
   *  @param vOutliers The outlier measurements encoded as KeyFrame,MapPoint pairs */
  void HandleOutliers(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers);
  
  /** @brief Points marked bad are removed from the internal queues. Call this function before emptying the Map trash to ensure no pointers to MapPoints are left hanging around. */
  void EraseBadEntitiesFromQueues();
  
  std::list<std::pair<KeyFrame*, MapPoint*> > mlFailureQueue; ///< Queue of failed observations to re-find, implemented as a list
  std::list<MapPoint*> mlpNewQueue;   ///< Queue of newly-made map points to re-find in other KeyFrames, implemented as a list
  
  TaylorCameraMap mmCameraModels;          ///< The camera models used for projecting points
  BundleAdjusterBase& mBundleAdjuster;     ///< The bundle adjuster used for map optimization
  
  int mnNumConsecutiveFailedBA;    ///< Number of consecutive failed bundle adjustments, if exceeds threshold map is considered corrupt and is reset
  
  ros::ServiceClient mResetSystemClient;    ///< A service client to call the System's reset service, used by RequestResetInternal()
};

#endif


