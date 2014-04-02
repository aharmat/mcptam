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
 * \file MapMakerServer.h
 * \brief Declaration of MapMakerServer class
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
 * MapMakerServer is the server portion of the networked client/server map maker.
 * It defines MapMakerServerBase's virtual functions to pass requisite data
 * over the network to MapMakerClient.
 *
 ****************************************************************************************/

#ifndef __MAP_MAKER_SERVER_H
#define __MAP_MAKER_SERVER_H

#include <mcptam/MapMakerServerBase.h>
#include <mcptam/NetworkManager.h>
#include <cvd/thread.h>
#include <std_srvs/Empty.h>

/** @brief Implements the server portion of the networked client/sever map maker 
 * 
 * By inheriting from CVD::Thread, we gain access to some useful functions that allow this class
 * to run in its own thread */
class MapMakerServer : public MapMakerServerBase, protected CVD::Thread
{ 
public:

  /** @brief Need to call constructor with arguments to ensure valid object
   *  @param map The Map being worked on
   *  @param cameras The camera models
   *  @param bundleAdjuster Some derived class of BundleAdjusterBase that will be used to optimize the map */
  MapMakerServer(Map &map, TaylorCameraMap &cameras, BundleAdjusterBase &bundleAdjuster);
  
  /// Destructor
  virtual ~MapMakerServer();
  
  /// Overridden from CVD::Thread, this executes in its own thread
  virtual void run();

protected:

  /** @brief Calls Reset on parent classes */
  void Reset();
  
  /** @brief Sends points to the client as an "add" message
   *  @brief begin_it Iterator into the list of MapPoint pointers where sending will start
   *  @brief begin_it Iterator into the list of MapPoint pointers where sending will end 
   *                  (same rule as STL containers, operation will happen up to but not including end iterator) */
  void SendPoints(MapPointPtrList::iterator begin_it, MapPointPtrList::iterator end_it);
  
  /** @brief Send updates of certain MultiKeyFrames and MapPoints to the client
   *  @param spAdjustedFrames The MultiKeyFrames whose updated pose and scene depth will be sent to client
   *  @param spAdjustedPoints The MapPoints whose updated position and internal pixel vectors will be sent to client */
  void SendUpdate(std::set<MultiKeyFrame*>& spAdjustedFrames, std::set<MapPoint*>& spAdjustedPoints);

  /** @brief Overridden from MapMakerServerBase, deletes bad points, sends message to client, and removes pointers to
   * deleted points from internal queues. */
  virtual void HandleBadEntities();
  
  /** @brief Overridden from MapMakerServerBase, returns the size of the network manager's incoming queue as that is the
   * data source for us. In MapMaker, the data source would be the tracker queue. */
  virtual int IncomingQueueSize(){ return mNetworkManager.IncomingQueueSize(); } 
  
  /** @brief Function that mNetworkManager calls when an "init" message from the client is processed
   * 
   *  The network manager will have created the MultiKeyFrame object, so we treat it exactly the same as if it was given
   *  to us by the Tracker. After initialization, this function sends an update of the MultiKeyFrame pose and the new
   *  MapPoints that have been created.
   *  @param pMKF Pointer to the MultiKeyFrame we should use for initialization
   *  @return Did initialization succeed? */
  bool InitCallback(MultiKeyFrame* pMKF);
  
  /** @brief Function that mNetworkManager calls when an "add" message from the client is processed
   * 
   *  The network manager will have created the MultiKeyFrame object, we need to deal with it exactly the same way as if
   *  Tracker had given us a new MultiKeyFrame.
   *  @param spMultiKeyFrames Set of MultiKeyFrames to add, should only ever contain one object
   *  @param spPoints Set of MapPoints to add, should never be filled out by network manager because the client doesn't add points to the server. 
   *                  The only reason it's here is because the AddCallback function signature is used by the network manager whether it's on the client or server */
  void AddCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints);
  
   /** @brief Function that mNetworkManger calls when a "delete" message from the client is processed
   *  
   *  The mbDeleted flag of the points will be set to true, after which they can be moved to the trash without affecting the other "bad"
   *  points that we might currently be holding of which the client will be notified.
   *  @param spPoints The points that will get deleted */
  void DeleteCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints);
  
  /// Called by mNetworkManager immediately when a new "add" message arrives, requests the bundle adjuster to abort if it's running
  void NewAddCallback();
  
  /** @brief Function that mNetworkManager calls when a RESET message from the server is processed  */
  void ResetCallback();
  
  NetworkManager mNetworkManager;        ///< The network manager, handles all data transactions with the client
  bool mbInitialized;        ///< Has the map been initialized? Some operations shouldn't be carried out before map has fully initialized because client won't have the same data as server.
};

#endif

