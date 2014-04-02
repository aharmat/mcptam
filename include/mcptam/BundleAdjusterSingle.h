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
 * \file BundleAdjusterSingle.h
 * \brief Declaration of BundleAdjusterSingle class
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
 * BundleAdjusterSingle uses the ChainBundle class to perform bundle adjustment of 
 * MultiKeyFrames where measurements of MapPoints are made through KeyFrames. Unlike
 * BundleAdjusterMulti, the KeyFrames are adjusted individually, ignoring the pose between 
 * KeyFrames and their parent MultiKeyFrame (ie almost the same as the Bundle class of PTAM). 
 * After adjustment, the parent MultiKeyFrame's pose is set to the pose of the first
 * child KeyFrame, and then the KeyFrames' CamFromBase transforms are updated.
 *
 ****************************************************************************************/

#ifndef __BUNDLE_ADJUSTER_SINGLE_H
#define __BUNDLE_ADJUSTER_SINGLE_H

#include <mcptam/BundleAdjusterBase.h>

/** @brief Uses ChainBundle to perform bundle adjustment with MultiKeyFrames
 * 
 * The KeyFrames are adjusted individually, ignoring the pose between KeyFrames and
 * their parent MultiKeyFrame (ie almost the same as the Bundle class of PTAM). 
 * After adjustment, the parent MultiKeyFrame's pose is set to the pose of the first
 * child KeyFrame, and then the KeyFrames' CamFromBase transforms are updated.
 */
class BundleAdjusterSingle : public BundleAdjusterBase
{
public:

  /// Only a parameterized constructor is available to ensure requirements are met
  /** @param map The map being adjusted
   *  @param cameras The camera models being used 
   *  @param bApplyUpdates Apply updates to map? */
  BundleAdjusterSingle(Map &map, TaylorCameraMap& cameras, bool bApplyUpdates = true);
  
  /// Destructor
  virtual ~BundleAdjusterSingle(){ };

protected:
  
  /** @brief Loads the given MultiKeyFrames and MapPoints into the underlying ChainBundleG2O, runs computation, extracts results and outliers
   *  @param [out] spAdjustSet The set of MultiKeyFrames that were moved during adjustment
   *  @param [out] spFixedSet The set of MultiKeyFrames that were held fixed during adjustment (only the first one)
   *  @param [out] spMapPoints The set of MapPoints that were adjusted (all of them)
   *  @param bRecent Is the bundle adjustment using only the most recent MultiKeyFrame and its neighbors?
   *  @param [out] vOutliers The vector of outlier measurements found, encoded as KeyFrame,MapPoints pairs
   *  @return The number of good iterations of the underlying ChainBundleG2O, negative indicates error */
  virtual int BundleAdjust(std::set<MultiKeyFrame*> spAdjustSet, std::set<MultiKeyFrame*> spFixedSet, std::set<MapPoint*> spMapPoints, std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers, bool bRecent);
    
  bool mbApplyUpdates;             ///< Will the Map be updated with adjusted positions? False only for debugging.
  
  std::map<KeyFrame*, int> mmKF_BundleID; ///< Map FROM Keyframe* TO keyframe id
  std::map<int, KeyFrame*> mmBundleID_KF; ///< Map FROM keyframe id TO Keyframe*
};

#endif

