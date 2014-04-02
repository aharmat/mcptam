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
 * \file BundleAdjusterBase.h
 * \brief Declaration of BundleAdjusterBase class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the MapMaker class in the original PTAM, which are
 * Copyright 2008 Isis Innovation Limited
 *
 * The bundle adjustment code has been separated from the MapMaker so that different 
 * bundle adjusters can be used depending on need (ie live tracking or calibration).
 * BundleAdjusterBase is an abstract base class that other classes have to derive from
 * and implement the BundleAdjust(...) function to perform the actual computations.
 *
 ****************************************************************************************/

#ifndef __BUNDLE_ADJUSTER_BASE_H
#define __BUNDLE_ADJUSTER_BASE_H

#include <mcptam/Types.h>
#include <set>

class Map;
class KeyFrame;
class MultiKeyFrame;
class MapPoint;

/** @brief An abstract base class that specifies the interface of BundleAdjusterXYZ classes
 * 
 * This class allows the removal of virtually all bundle adjustment code from the map maker, which are replaced by a few function calls.
 * This allows different bundle adjuster types to be used by the map maker without knowing what's going on under the hood.
 * To derive from this class, override the protected member function BundleAdjust(...), which performs the actual bundle adjustment.
 */
class BundleAdjusterBase
{
public:
  /// Only a parameterized constructor is available to ensure requirements are met
  /** @param map The map being adjusted
   *  @param cameras The camera models being used */
  BundleAdjusterBase(Map &map, TaylorCameraMap& cameras);
  virtual ~BundleAdjusterBase(){ };
  
  /// Set internal flags to indicate problem has not converged
  void SetNotConverged(){ mbBundleConverged_Recent = false; mbBundleConverged_Full = false; }
  /// Set internal flags to the default state
  void Reset();
  /// Set internal abort flag to true, which will trigger the underlying bundle adjustment class to quit computation
  void RequestAbort(){ mbBundleAbortRequested = true; }
  /// Is the adjuster running and working on all KeyFrames?
  /** @return Bundle adjuster running state (working on all KeyFrames) */
  bool Running(){ return mbBundleRunning; }
  /// Is the adjuster running and working only on recently added KeyFrames?
  /** @return Bundle adjuster running state (working on recent KeyFrames) */
  bool RunningIsRecent(){ return mbBundleRunningIsRecent; }
  /// Has the map converged while working only on recently added KeyFrames?
  /** @return Bundle adjuster convergence state (working on recent KeyFrames) */
  bool ConvergedRecent(){ return mbBundleConverged_Recent; }
  /// Has the map converged while working on all KeyFrames?
  /** @return Bundle adjuster convergence state (working on all KeyFrames) */
  bool ConvergedFull(){ return mbBundleConverged_Full; }
  /// The maximum point covariance from the last run
  double GetMaxCov(){ return mdMaxCov; }
  /// The mean of weighted errors squared from end of last run
  double GetMeanChiSquared(){ return mdMeanChiSquared; }
  /// The sigma squared value of the robust kernel
  double GetSigmaSquared(){ return mdSigmaSquared; }
  /// Tell the adjuster whether to use Tukey test on measurements
  void UseTukey(bool bUse){ mbUseTukey = bUse; }
  /// Tell the adjuster whether to do a couple of iterations, update the map, then continue or run to convergence first
  void UseTwoStep(bool bUse){ mbUseTwoStep = bUse; }
  /// Tell the adjuster whether to use robustification on measurements
  void UseRobust(bool bUse){ mbUseRobust = bUse; }

  /** @brief Adjust all the MapPoints and MultiKeyFrames in the Map
   *  @param [out] vOutliers The vector of outlier measurements found, encoded as KeyFrame,MapPoints pairs
   *  @return The number of good iterations of the underlying bundle adjuster, negative indicates error */
  int BundleAdjustAll(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers);
  
  /** @brief Adjusts the most recent MultiKeyFrame and a small number of its neighbors, as well as the MapPoints seen from them
   * 
   *  The number of neighbors used depends on the static variable snRecentNum, which can be set by a ROS param (see LoadStaticParams.h)
   *  @param [out] vOutliers The vector of outlier measurements found, encoded as KeyFrame,MapPoints pairs
   *  @return The number of good iterations of the underlying bundle adjuster, negative indicates error */
  int BundleAdjustRecent(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers);
  
  // Static variables
  static int snRecentMinSize;  ///< Number of MultiKeyFrames necessary in the map before BundleAdjustRecent(...) will run
  static int snRecentNum;      ///< Number of neighbor MultiKeyFrames that will be used in addition to the most recently added on in BundleAdjustRecent(...)
  static int snMinMapPoints;   ///< Number of good MapPoints needed to do bundle adjustment
  
protected:

  /// Override this to implement specific bundle adjustment method, 
  /// see code of BundleAdjustAll(...) and BundleAdjustRecent(...) to figure out how it's called
  virtual int BundleAdjust(std::set<MultiKeyFrame*> sAdjustSet, std::set<MultiKeyFrame*> sFixedSet, std::set<MapPoint*> sMapPoints, std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers, bool bRecent) = 0;
  
  /** @brief Get a certain number of neighbors of a given KeyFrame
   *  @param kfSrc The KeyFrame we're finding neighbors for
   *  @param N The number of neighbors to find
   *  @return A vector of KeyFrame pointers to the found neighbors */
  std::vector<KeyFrame*> NClosestKeyFrames(KeyFrame &kfSrc, unsigned int N);
  /** @brief Get a certain number of neighbors of a given MultiKeyFrame
   *  @param mkfSrc The MultiKeyFrame we're finding neighbors for
   *  @param N The number of neighbors to find
   *  @return A vector of MultiKeyFrame pointers to the found neighbors */
  std::vector<MultiKeyFrame*> NClosestMultiKeyFrames(MultiKeyFrame &mkfSrc, unsigned int N);

  Map& mMap;  ///< Reference to the map being worked on

  bool mbBundleRunning;            ///< Is the adjuster running on the whole map?
  bool mbBundleRunningIsRecent;    ///< Is the adjuster running on the most recent MultiKeyFrame and its neighbors?
  bool mbBundleAbortRequested;     ///< Has an abort been requested?
  bool mbBundleConverged_Recent;   ///< Has the map around the most recent MultiKeyFrame converged
  bool mbBundleConverged_Full;     ///< Has the whole map converged?
  
  double mdMaxCov;               ///< The maximum point feature position covariance in the map
  double mdSigmaSquared;         ///< Sigma squared value for the robust kernel
  double mdMeanChiSquared;       ///< Mean of errors squared for the optimization, weighted by information and robust kernel
  
  bool mbUseTukey;                ///< Use the Tukey test to eliminate outliers
  bool mbUseTwoStep;              ///< Stop optimization early to initialize new points quickly, then continue
  bool mbUseRobust;               ///< Use robustification on measurements
  
  TaylorCameraMap mmCameraModels;  ///< The camera models
  
  std::map<MapPoint*, int> mmPoint_BundleID;      ///< %Map FROM MapPoint* TO point id
  std::map<int, MapPoint*> mmBundleID_Point;      ///< %Map FROM point id TO MapPoint*
  std::map<MultiKeyFrame*, int> mmBase_BundleID;  ///< %Map FROM MultiKeyFrame* TO mkf id
  std::map<int, MultiKeyFrame*> mmBundleID_Base;  ///< %Map FROM mkf id TO MultiKeyFrame*
  std::map<std::string, int> mmCamName_BundleID;  ///< %Map FROM camera name TO camera id
  std::map<int, std::string> mmBundleID_CamName;  ///< %Map FROM camera id TO camera name
  
};

#endif

