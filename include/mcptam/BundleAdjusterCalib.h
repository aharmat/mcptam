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
 * \file BundleAdjusterCalib.h
 * \brief Declaration of BundleAdjusterCalib class
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
 * BundleAdjusterCalib uses the ChainBundle class to perform bundle adjustment of 
 * MultiKeyFrames where measurements of MapPoints are made through KeyFrames. The problem
 * is set up such that there is only a single transform representing the relative pose
 * between two cameras, and the pose of the first camera within a MultiKeyFrame is identity.
 * For example, in a system with two cameras (camera1 & camera2, represented in KeyFrame form
 * as KF1 and KF2) and two MultiKeyFrames, we have the following pose graph:
 *
 *       world1
 *             \
 *              KF1------KF2
 *             /
 *       world2
 *
 * This way, when calibrating the extrinsic properties of the cameras, we are only solving
 * for a single transform between any two cameras, rather than taking some kind of average
 * of a bunch of different poses. 
 *
 ****************************************************************************************/

#ifndef __BUNDLE_ADJUSTER_CALIB_H
#define __BUNDLE_ADJUSTER_CALIB_H

#include <mcptam/BundleAdjusterBase.h>

class ChainBundle;

/** @brief Uses ChainBundle to perform bundle adjustment for extrinsic camera calibration
 * 
 * The problem is set up such that there is only a single transform representing the relative pose
 * between two cameras, and the pose of the first camera within a MultiKeyFrame is identity. This class can be used
 * with a call to BundleAdjustAll(...). Do not use it with BundleAdjustRecent(...), it will throw an error, as the
 * whole map is needed for calibration.
 */
class BundleAdjusterCalib : public BundleAdjusterBase
{
public:

  /// Only a parameterized constructor is available to ensure requirements are met
  /** @param map The map being adjusted
   *  @param cameras The camera models being used 
   *  @param bApplyUpdates Apply updates to map? */
  BundleAdjusterCalib(Map &map, TaylorCameraMap& cameras, bool bApplyUpdates = true);
  
  /// Destructor
  virtual ~BundleAdjusterCalib(){ };
  
  SE3Map mmFinalPoses;     ///< Camera name => pose map of final relative transforms (after adjustment)

protected:
  
  /** @brief Loads the given MultiKeyFrames and MapPoints into the underlying ChainBundle, runs computation, extracts results and outliers
   *  @param [out] spAdjustSet The set of MultiKeyFrames that were moved during adjustment
   *  @param [out] spFixedSet The set of MultiKeyFrames that were held fixed during adjustment (only the first one)
   *  @param [out] spMapPoints The set of MapPoints that were adjusted
   *  @param [out] vOutliers The vector of outlier measurements found, encoded as KeyFrame,MapPoints pairs
   *  @param bRecent Is the bundle adjustment using only the most recent MultiKeyFrame and its neighbors?
   *  @return The number of good iterations of the underlying ChainBundleG20, negative indicates error */
  virtual int BundleAdjust(std::set<MultiKeyFrame*> spAdjustSet, std::set<MultiKeyFrame*> spFixedSet, std::set<MapPoint*> spMapPoints, std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers, bool bRecent);
    
  /** @brief Extracts a relative pose map from a set of MKFs. The assumption is that all KFs' mse3CamFromBase
    * pose has been properly set. 
    * @param spMKFs Set of MKFs to look at
    * @return The relative pose map */
  SE3Map ExtractRelativePoses(std::set<MultiKeyFrame*> spMKFs);
    
  bool mbApplyUpdates;          ///< Will the Map be updated with adjusted positions? False only for debugging.
};

#endif

