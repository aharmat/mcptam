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
 * \file Map.h
 * \brief Declaration of Map class
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *   This header declares the Map class.
 *   This is pretty light-weight: All it contains is
 *   a vector of MapPoints and a vector of KeyFrames.
 *   
 *   N.b. since I don't do proper thread safety,
 *   everything is stored as lists of pointers,
 *   and map points are not erased if they are bad:
 *   they are moved to the trash list. That way
 *   old pointers which other threads are using are not 
 *   invalidated!
 *
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *   The above comments no longer apply exactly as stated. First, the MapPoints and
 *   KeyFrames are now in lists to allow deletion without reordering vectors. Thread
 *   safety is implemented using a mutex that prevents MapPoints from being moved
 *   to the trash when the Tracker is acquiring its potentially visible set (PVS). Also, a 
 *   reference counting scheme embedded in the MapPoint and MultiKeyFrame classes allows the Tracker
 *   to release the mutex lock as soon as the PVS is acquired because MapPoints and 
 *   MultiKeyFrames with a positive "using" count won't be deleted from the trash.
 *
 ****************************************************************************************/


#ifndef __MAP_H
#define __MAP_H

#include <mcptam/Types.h>
#include <set>
#include <boost/thread/mutex.hpp>

class MapPoint;

/** @brief Contains the lists of MultiKeyFrames and MapPoints, and methods to manipulate them.
 * 
 *  The Map also maintains a list of MapPoints and MultiKeyFrames that are considered to be in the trash, ie invalid
 *  but not yet deleted. These entities will sit in the trash until their "using" count drops to 
 *  zero, after which they will be delted during the next call to EmptyTrash. Users of Map outside
 *  the MapMaker thread need to lock mMutex before interating through the MapPoints or MultiKeyFrames to ensure
 *  the iterator is not invalidated by a simultaneous move to the trash. MapPoints or MultiKeyFrames that need to be
 *  valid even after the iteration (ie in Tracker the set of points being worked on and their source KF/MKFs) should have their
 *  "using" counter incremented when acquired, which ensures they will not be delted from the trash.
 *  The counter needs to be decremented after usage is finished, otherwise the object will never
 *  be deleted. The best way to do this is using an itrusive pointer that takes care of the increment/decrement
 *  automatically. See TrackerData for details.
 */
class Map
{
public:

  /// Default constructor
  Map();
  
  /// Destructor
  ~Map();
  
  /// Clears all lists and deletes their contents, sets mbGood flag to false
  void Reset();
  
  /** @brief Points marked bad are moved to the trash. 
   * 
   *  Before each move to the trash, mMutex is locked. Therefore, if someone else has locked the map
   *  this function could be slightly delayed until the lock is freed.
   *  @return The set of map points that were moved to the trash */
  std::set<MapPoint*> MoveBadPointsToTrash();
  
  /** @brief MultiKeyFrames marked bad are moved to the trash
   * 
   *  The mutex is locked before moving each MultiKeyFrame.
   *  @return The set of MultiKeyFrames that were moved to the trash.  */
  std::set<MultiKeyFrame*> MoveBadMultiKeyFramesToTrash();
  
  /** @brief Points marked deleted are moved to the trash. 
   * 
   *  Similar to MoveBadPointsToTrash, except it's points that are marked deleted that are moved to the trash.
   *  Mutex locking is identical. The "bad" flag is also set on points move to the trash. This function
   *  is used in the client/server mode to allow incoming bad points to be deleted immediately without
   *  mixing them up with other points that have been marked bad. */
  void MoveDeletedPointsToTrash();
  
  /** @brief MultiKeyFrames marked "deleted" are moved to the trash */
  void MoveDeletedMultiKeyFramesToTrash();
  
  /** @brief Deletes points that are in the trash.
   * 
   * If an object's "using" counter is at zero, it is deleted
   * and removed from the trash. */
  void EmptyTrash();
  
  void MakeSnapshot();

  void Restore();
  
  void SaveToFolder(std::string folder, bool bKeepExistingImages, std::string postfix="");
  
  void LoadFromFolder(std::string folder, SE3Map mPoses, TaylorCameraMap mCameraModels, bool bFix);
  
  bool Contains(MultiKeyFrame* pMKF);
  
  MapPointPtrList mlpPoints;  ///< List of MapPoint pointers that are in the live map
  MapPointPtrList mlpPointsTrash;  ///< List of MapPoint pointers that are in the trash
  MultiKeyFramePtrList mlpMultiKeyFrames;  ///<List of MultiKeyFrame pointers that are in the live map
  MultiKeyFramePtrList mlpMultiKeyFramesTrash;  ///< List of MultiKeyFrame pointers that are in the trash
  boost::mutex mMutex;   ///< To allow multi-threaded operation safely
  bool mbGood;  ///< Is the map initialized and operational?
  
  // For making a snapshot
  MapPointPtrList mlpPointsSnapshot; ///< List of MapPoint pointers that are in the live map
  MultiKeyFramePtrList mlpMultiKeyFramesSnapshot; ///<List of MultiKeyFrame pointers that are in the live map
  bool mbGoodSnapshot;
};




#endif

