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
 * \file BundleAdjusterMulti.h
 * \brief Declaration of BundleAdjusterMulti class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the MapMaker class in the original PTAM, which are
 * Copyright 2008 Isis Innovation Limited
 *
 * The bundle adjustment code has been separated from the MapMaker so that different 
 * bundle adjusters can be used depending on need (ie live tracking or calibration).
 * 
 * BundleAdjusterMulti uses the ChainBundle class to perform bundle adjustment of 
 * MultiKeyFrames where measurements of MapPoints are made through KeyFrames. 
 * Unlike in BundleAdjusterSingle, the pose between KeyFrames and their parent 
 * MultiKeyFrame is kept fixed, only the MultiKeyFrame pose is changed.
 *
 ****************************************************************************************/


#ifndef __BUNDLE_ADJUSTER_MULTI_H
#define __BUNDLE_ADJUSTER_MULTI_H

#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/ChainBundle.h>
#include <boost/function.hpp>

/** @brief Uses ChainBundle to perform bundle adjustment with MultiKeyFrames
 * 
 * The pose between KeyFrames and their parent MultiKeyFrame is kept fixed, only the
 * MultiKeyFrame is moved. 
 */
class BundleAdjusterMulti : public BundleAdjusterBase
{
public:

  /// Used to call an external update function after a round of bundle adjustment, used only when running on server side
  typedef boost::function<void (std::set<MultiKeyFrame*>, std::set<MapPoint*>)> UpdateCallbackType;

  /// Only a parameterized constructor is available to ensure requirements are met
  /** @param map The map being adjusted
   *  @param cameras The camera models being used 
   *  @param bApplyUpdates Apply updates to map? */
  BundleAdjusterMulti(Map &map, TaylorCameraMap& cameras, bool bApplyUpdates = true, bool bVerbose = false);
  
  /// Destructor
  virtual ~BundleAdjusterMulti(){ };
  
  /// Set the function that will be called after a successful bundle adjustment. The reason this exists is that
  /// the bundle adjuster will be first run with a small number of iterations to get large errors to disappear, and then
  /// till convergence. In case we are running this in MapMakerServer, we want to send updates to the client after
  /// every update, but the BundleAdjust(...) function won't return until convergence or the max iterations are up.
  /// Therefore, pass the update sending function as a callback to SetUpdateCallback(...), and then everything will
  /// work nicely.
  void SetUpdateCallback(UpdateCallbackType up){ mUpdateCallback = up; }

protected:
  
  /** @brief Loads the given MultiKeyFrames and MapPoints into the underlying ChainBundle, runs computation, extracts results and outliers
   *  @param [out] spAdjustSet The set of MultiKeyFrames that were moved during adjustment
   *  @param [out] spFixedSet The set of MultiKeyFrames that were held fixed during adjustment (only the first one)
   *  @param [out] spMapPoints The set of MapPoints that were adjusted (all of them)
   *  @param [out] vOutliers The vector of outlier measurements found, encoded as KeyFrame,MapPoints pairs
   *  @param bRecent Is the bundle adjustment using only the most recent MultiKeyFrame and its neighbors?
   *  @return The number of good iterations of the underlying ChainBundle, negative indicates error */
  virtual int BundleAdjust(std::set<MultiKeyFrame*> spAdjustSet, std::set<MultiKeyFrame*> spFixedSet, std::set<MapPoint*> spMapPoints, std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers, bool bRecent);
    
  /** @brief Run the Bundle Adjustment and update the map with the result
   *  @param multiBundle Reference to the ChainBundle object
   *  @param spAdjustSet The set of adjusted MultiKeyFrames, passed in so it can be sent to the update callback if necessary
   *  @param spMapPoints The set of adjusted MapPoints, passed in so it can be sent to the update callback if necessary
   *  @param nIterations The number of iterations to attempt to run 
   *  @return The number of good iterations of the ChainBundle, negative indicates error */
  int AdjustAndUpdate(ChainBundle& multiBundle, std::set<MultiKeyFrame*> spAdjustSet, std::set<MapPoint*> spMapPoints, int nIterations=-1);
    
  bool mbApplyUpdates;              ///< Will the Map be updated with adjusted positions? False only for debugging.
  bool mbVerbose;                   ///< Output extra debugging information from the ChainBundle calculations
  UpdateCallbackType mUpdateCallback;  ///< The callback function handle
};

#endif

