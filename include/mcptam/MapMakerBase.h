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
 * \file MapMakerBase.h
 * \brief Declaration of MapMakerBase interface
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
 * MapMakerBase contains everything that is common to both client-side and
 * server-side operations, such as a reference to an underlying map, resetting 
 * the map, and some KeyFrame operations.
 *
 ****************************************************************************************/

#ifndef __MAP_MAKER_BASE_H
#define __MAP_MAKER_BASE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Map;
class KeyFrame;
class MultiKeyFrame;

/** @brief Contains components that are common to both the "client-side" and the "server-side"
 *         of the map-making task. 
 * 
 * Client-side deals with interfacing with the tracker, and server-side deals with optimizing the map.
 * This is not an abstract base class, but why would you want to create an object of this class? It'd be
 * pretty useless. */
class MapMakerBase
{
public:

  /** @brief Used for indicating the region to search for closest KeyFrames
   * 
   * KF_ONLY_SELF: Search only those KeyFrames that share a parent \n
   * KF_ONLY_OTHER: Search only those KeyFrames that DON'T share a parent \n
   * KF_ALL: Search all KeyFrames */
  enum KeyFrameRegion{KF_ONLY_SELF, KF_ONLY_OTHER, KF_ALL};
  
  /** @brief Used to indicate the status of the map maker
   *
   * INITIALIZING: Initialization phase
   * JUST_FINISHED_INIT: Just finished initialization
   * RUNNING: Normal running phase */   
  enum State{MM_INITIALIZING, MM_JUST_FINISHED_INIT, MM_RUNNING} mState;

  /** @brief Need to call constructor with Map as argument
   *  @param map The Map being worked on 
   *  @param bAdvertise Should we advertise various info topics right away? Useful if we're actually going to publish on them, confusing if we're not */
  MapMakerBase(Map &map, bool bAdvertise);
  
  /// Destructor
  virtual ~MapMakerBase(){ };
  
  /** @brief Has resetting finished?
   *  @return True if reset complete, false otherwise */
  bool ResetDone(){ return mbResetDone; }
  
  /// Publish information about the map
  void PublishMapInfo();
    
  /// Publish the map points, then the MKFs
  void PublishMapVisualization();
  
  /** @brief Finds the closest KeyFrame in a given region
   *  @param kf The KeyFrame to search around
   *  @param region Enum code indicating the region to search
   *  @param bSameCamName Only search for keyframes with the same camera name
   *  @return Pointer to the nearest matching KeyFrame */
  KeyFrame* ClosestKeyFrame(KeyFrame &kf, KeyFrameRegion region = KF_ALL, bool bSameCamName = false);
  
  /** @brief Finds nMaxNum closest KeyFrames, within a given distance, within a given region 
   *  @param kf The KeyFrame to search around
   *  @param dThreshDist The threshold distance
   *  @param nMaxNum The maximum number of keyframes to find
   *  @param region Enum code indicating the region to search
   *  @return Vector of pointers to the resulting keyframes */
  std::vector<KeyFrame*> ClosestKeyFramesWithinDist(KeyFrame &kf, double dThreshDist, unsigned int nMaxNum, KeyFrameRegion region = KF_ALL);
  
  /** @brief Finds N closest KeyFrames
   *  @param kfSrc The KeyFrame to search around
   *  @param N The maximum number of keyframes to find
   *  @return Array of pointers to the resulting keyframes */
  std::vector<KeyFrame*> NClosestKeyFrames(KeyFrame &kfSrc, unsigned int N);
  
  /** @brief Finds N closest MultiKeyFrames
   *  @param mkfSrc The MultiKeyFrame to search around
   *  @param N The maximum number of multikeyframes to find
   *  @return Array of pointers to the resulting multikeyframes */
  std::vector<MultiKeyFrame*> NClosestMultiKeyFrames(MultiKeyFrame &mkfSrc, unsigned int N);
  
  /** @brief Finds the closest MultiKeyFrame
   *  @param mkf The MultiKeyFrame to search around
   *  @return Pointer to the nearest MultiKeyFrame */
  MultiKeyFrame* ClosestMultiKeyFrame(MultiKeyFrame &mkf);
  
  /** @brief Finds the furthest MultiKeyFrame
   *  @param mkf The MultiKeyFrame to search around
   *  @return Pointer to the furthest MultiKeyFrame */
  MultiKeyFrame* FurthestMultiKeyFrame(MultiKeyFrame &mkf);
  
  /// Check if the MapMaker is in the initialization phase
  bool Initializing(){ return mState == MM_INITIALIZING; }
  
  /** @brief Gets the maximum covariance on the positions of the point features */
  double GetMaxCov(){ return mdMaxCov; }
  
protected:

  /** @brief Has a reset been requested?
   *  @return True if reset was requested, false otherwise */
  bool ResetRequested(){ return mbResetRequested; }
  
  /** @brief This will set an internal flag to indicate that the map should wipe clean.
   * 
   *  This is called by subclasses, who should implement their own RequestReset function
   *  that will first do any class-specific processing required when a reset request is received,
   *  and then call this function. */
  void RequestReset();  
  
  /** @brief Wipe the map, reset internal flags
   *  
   *  This is called by subclasses, who should implement their own Reset function
   *  that will first do any class-specific resetting (ie clearing variables), and 
   *  then call this function LAST. */
  void Reset();
  
  /// Publish map point PCL cloud
  void PublishMapPoints();
  
  /// Publish MKFs as a marker array
  void PublishMapMKFs();
  
  /// Converts an MKF to a visualization  message
  /** @param mkf The MKF to convert
   *  @param [out] marker The resulting visualization message */
  void MKFToMarker(MultiKeyFrame& mkf, visualization_msgs::Marker& marker);
  
  /// Dumps all map information to a file
  void DumpToFile(std::string filename);
  
  Map &mMap;   ///< Reference to the Map
  
  ros::NodeHandle mNodeHandle;   ///< ROS node handle referencing the global namespace
  ros::NodeHandle mNodeHandlePriv; ///< ROS node handle referencing the private namespace
  
  ros::Publisher mMapInfoPub;   ///< Publishes diagnostic messages about the map
  ros::Publisher mMapPointsPub;  ///< Publishes point markers
  ros::Publisher mMapMKFsPub;    ///< Publishes MKF visualization markers
  
  double mdMaxCov;              ///< Maximum covariance of the point feature positions
  
private:
  // Thread interaction signalling stuff, only want this accessed through appropriate functions
  // to keep our sanity
  bool mbResetRequested;   ///< A reset has been requested
  bool mbResetDone;        ///< The reset was done.
  
};

#endif

