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
 * \file MapMakerClient.h
 * \brief Declaration of MapMakerClient class
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
 * MapMakerClient is the client portion of the networked client/server map maker.
 * It defines MapMakerClientBase's virtual functions to pass requisite data
 * over the network to MapMakerSever.
 *
 ****************************************************************************************/

#ifndef __MAP_MAKER_CLIENT_H
#define __MAP_MAKER_CLIENT_H

#include <mcptam/MapMakerClientBase.h>
#include <mcptam/NetworkManager.h>
#include <cvd/thread.h>


/** @brief Implements the client portion of the networked client/sever map maker 
 * 
 * By inheriting from CVD::Thread, we gain access to some useful functions that allow this class
 * to run in its own thread */
class MapMakerClient : public MapMakerClientBase, protected CVD::Thread
{ 
public:

  /** @brief Need to call constructor with Map as argument
   *  @param map The Map being worked on */
  MapMakerClient(Map &map, TaylorCameraMap &cameras);
  virtual ~MapMakerClient();
  
  /// Overridden from CVD::Thread, this executes in its own thread
  virtual void run();
  
  /// Overridden from MapMakerClientBase, entry point for Tracker to add a MultiKeyFrame
  /** @param pMKF_Incoming Pointer to the MultiKeyFrame being added */
  virtual void AddMultiKeyFrame(MultiKeyFrame*& pMKF_Incoming);  
  
  /// Overridden from MapMakerClientBase, entry point for Tracker to initialize the map
  /** @param pMKF_Incoming Pointer to the MultiKeyFrame being used to initialize map */
  virtual bool Init(MultiKeyFrame*& pMKF_Incoming, bool bPutPlaneAtOrigin);
  
  /// Overridden from MapMakerClientBase, entry point for Tracker to request a reset
  virtual void RequestReset();
  
protected:

  /** @brief Calls Reset on parent classes and server */
  void Reset();

  /** @brief Overridden from MapMakerClientBase, called to add a MultiKeyFrame from the internal queue to the map
   * 
   *  This keeps the MultiKeyFrame in the client-side map but doesn't make any measurements. It also sends the 
   *  MultiKeyFrame to the server, where measurements are made.*/
  virtual void AddMultiKeyFrameFromTopOfQueue();
  
  /// Deletes points that the tracker considered outliers, sends an update to the server telling it to do likewise
  void HandleBadPoints();
  
  /** @brief Function that mNetworkManager calls when an "add" message from the server is processed
   *
   *  The network manager will have created the MultiKeyFrame or MapPoint objects based on the "add" message, but 
   *  it's our job to actually put the objets into the Map
   *  @param spMultiKeyFrames This should never be filled out by the network manager, since the server will never add a MultiKeyFrame to the client.
   *                          The only reason it's here is because the AddCallback function signature is used by the network manager whether it's on the client or server.
   *  @param spPoints The points that we need to add to the map */
  void AddCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints);
  
  /** @brief Function that mNetworkManger calls when a "delete" message from the server is processed
   *  
   *  The mbDeleted flag of the points will be set to true, after which they can be moved to the trash without affecting the other "bad"
   *  points that we might currently be holding. This behaviour is needed because the server needs to be told about bad points, and we don't
   *  want to double count points.
   *  @param spPoints The points that will get deleted */
  void DeleteCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints);
  
  void StateCallback(MapMakerBase::State state, double dMaxCov);
  
  NetworkManager mNetworkManager;   ///< The network manager, handles all data transactions with the server
};

#endif

