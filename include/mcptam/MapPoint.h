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
 * \file MapPoint.h
 * \brief Declaration of MapPoint class
 *
 * Large parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * The map is made up of a bunch of MapPoints.
 * Each one is just a 3D point in the world;
 * it also includes information on where and in which KeyFrame the point was
 * originally made from, so that pixels from that KeyFrame can be used
 * to search for that point.
 * Also stores stuff like inlier/outlier counts, and private information for 
 * both Tracker and MapMaker.
 *
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * MapPoint has been modified to allow the use of multiple cameras by changing the single
 * TrackerData structure to a map of camera name => TrackerData pointers. There is also
 * an atomic integer counter that allows a TrackerData object to indicate that it is using
 * the MapPoint, which prevents its deletion by the MapMaker.
 *
 ****************************************************************************************/

#ifndef __MAP_POINT_H
#define __MAP_POINT_H

#include <mcptam/KeyFrame.h>
#include <TooN/TooN.h>
#include <cvd/image_ref.h>
#include <set>
#include <map>
#include <unordered_map>

#include <ros/ros.h>
#include <atomic>

class TrackerData;
class MapPoint;
class PointCrossCov;

typedef std::unordered_map<MapPoint*, PointCrossCov*> CrossCovPtrMap;

/// Stores information on keyframe status relative to owner MapPoint
struct MapMakerData
{
  std::set<KeyFrame*> spMeasurementKFs;   ///< Which keyFrames has this map point got measurements in?
  std::set<KeyFrame*> spNeverRetryKFs;    ///< Which keyFrames have measurements failed enough so I should never retry?
  
  /// Convenience function to get the number of measuring keyframes
  /** @return The number of measuring keyframes */
  inline int GoodMeasCount()            
  {  
    int nGoodCount = 0;
    for(std::set<KeyFrame*>::iterator kf_it = spMeasurementKFs.begin(); kf_it != spMeasurementKFs.end(); ++kf_it)
      nGoodCount += (int)!((*kf_it)->mpParent->mbBad);
    
    return nGoodCount; 
  }
};


/// Contains all map point related data, such as location relative to the world, source KeyFrame etc.
class MapPoint
{
public:
  /// Constructor inserts sensible defaults and zeros pointers.
  MapPoint()
  {
    mbFixed = false;
    mbBad = false;
    mbDeleted = false;
    mnUsing = 0;
    mbOptimized = false;
    mnMEstimatorOutlierCount = 0;
    mnMEstimatorInlierCount = 1;
    //mtCreationTime = ros::Time::now();
    mpPatchSourceKF = NULL;
    mm3WorldCov = TooN::Identity * 1e10;
  };
  
  /// Delete owned TrackerData pointers
  ~MapPoint();
  
  // Operators
  /// Deleted to disallow copy construction
  MapPoint(const MapPoint& other) = delete;
  /// Deleted to disallow assignment operation
  MapPoint& operator= (const MapPoint & other) = delete;
  
  // Utility functions
  /// Calls the EraseMeasurementOfPoint() function of all keyframes that hold measurements of this point 
  void EraseAllMeasurements();
  
  /// Refreshes the cached vectors used for patch finding
  void RefreshPixelVectors();      
  
  void EraseAllCrossCov();
  
  bool CrossCov(MapPoint* pOther, TooN::Matrix<3>& m3CrossCov);
  
  void UpdateCrossCovPriorities(double dVal);
  
  void AddCrossCov(MapPoint* pOther, PointCrossCov* pCrossCov);
  
  TooN::Vector<3> mv3WorldPos; ///< Current position relative to the world
  TooN::Matrix<3> mm3WorldCov; ///< Current covariance in world frame
  
  bool mbBad;  ///< Is it a dud? In that case it'll be moved to the trash soon.
  bool mbDeleted; ///< Similar to mbBad, but used only in client/server code to allow immediate deletion of received points
  std::atomic<unsigned int> mnUsing;  ///< Atomic counter that indicates the number of TrackerData structures in Tracker currently referencing this point
  bool mbFixed;  ///< Is the point fixed in the world frame?
  bool mbOptimized; ///< This point has been optimized and is in a valid position to track against
  
  // What pixels should be used to search for this point?
  KeyFrame* mpPatchSourceKF; ///< mnUsingThe KeyFrame the point was originally made in
  int mnSourceLevel;         ///< Pyramid level in source KeyFrame
  CVD::ImageRef mirCenter;   ///< The center of the patch, in level-coords in the source pyramid level
  
  // What follows next is a bunch of intermediate vectors - they all lead up
  // to being able to calculate v3Pixel{Down,Right}_W, which the PatchFinder
  // needs for patch warping!
  
  TooN::Vector<3> mv3Center_NC;             ///< Unit vector in Source-KF coords pointing at the patch center
  TooN::Vector<3> mv3OneDownFromCenter_NC;  ///< Unit vector in Source-KF coords pointing towards one pixel down of the patch center
  TooN::Vector<3> mv3OneRightFromCenter_NC; ///< Unit vector in Source-KF coords pointing towards one pixel right of the patch center
  TooN::Vector<3> mv3Normal_NC;             ///< Unit vector in Source-KF coords indicating patch normal
  
  TooN::Vector<3> mv3PixelDown_W;           ///< 3-Vector in World coords corresponding to a one-pixel move down the source image
  TooN::Vector<3> mv3PixelRight_W;          ///< 3-Vector in World coords corresponding to a one-pixel move right the source image
  
  MapMakerData mMMData; ///< Info for the Mapmaker (not to be trashed by the tracker)
  std::map<std::string, TrackerData*> mmpTData;  ///< Info for the Tracker (not to be trashed by the MapMaker)
  
  CrossCovPtrMap mmpCrossCov;
  boost::mutex mCrossCovMutex;
  
  // Info provided by the tracker for the mapmaker:
  int mnMEstimatorOutlierCount;  ///< Number of times this point was seen as an inlier while tracking
  int mnMEstimatorInlierCount;   ///< Number of times this point was seen as an outlier while tracking
  
  // Random junk (e.g. for visualisation)
  //ros::Time mtCreationTime;   ///< Time of creation
  int mnID;     ///< Used when dumping map to file
  
};


class PointCrossCov
{
public:
  PointCrossCov(MapPoint& point1, MapPoint& point2);
  
  // DEBUG
  ~PointCrossCov(){ ROS_BREAK(); }
  
  // Row: point1, Col: point2
  void SetCrossCov(const TooN::Matrix<3>& m3CrossCov);

  /// point: the querying point
  TooN::Matrix<3> GetCrossCov(MapPoint* pPoint);
  
  void EraseLinks();
  
  MapPoint& mPoint1;
  MapPoint& mPoint2;
  
  double mdPriority;

protected:
  
  TooN::Matrix<3> mm3CrossCov;
  
};

#endif

