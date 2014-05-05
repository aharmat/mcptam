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
 * \file NetworkManager.h
 * \brief Declaration of NetworkManager class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * NetworkManager handles all communication between client and server. MapMakerClient
 * and MapMakerServer both contain a NetworkManager, and can send data to each other 
 * by passing pointers as arguments to one of NetworkManager's functions.
 *
 ****************************************************************************************/

#ifndef __NETWORK_MANAGER_H
#define __NETWORK_MANAGER_H

#include <mcptam/Dictionary.h>
#include <mcptam/ModifyMap.h>
#include <mcptam/MapMakerBase.h>  // just for State enum
#include <cvd/thread.h>
#include <queue>
#include <set>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>

class KeyFrame;
class MultiKeyFrame;
class MapPoint;
class Measurement;

/** @brief Handles all communication between client and server. 
 * 
 *  MapPoint and MultiKeyFrame data can be sent as "add", "update" or "delete" messages. Internally,
 *  there is an incoming and outgoing message queue which allows calling code to return immediately
 *  rather than waiting for network communication to take place. Network connection issues are handled
 *  by repeatedly trying to send failed messages with a small wait time between each attempt. Upon 
 *  creation, NetworkManager runs in its own thread. Synchronization issues are avoided by processing
 *  messages in the incoming queue only through an explicit function call, which would happen within
 *  the calling code's thread of execution. */
class NetworkManager : protected CVD::Thread
{ 
public:
  typedef boost::function<bool(MultiKeyFrame*)> InitCallbackType;  
  typedef boost::function<void (std::set<MultiKeyFrame*>, std::set<MapPoint*>)> AddCallbackType;
  typedef boost::function<void (std::set<MultiKeyFrame*>, std::set<MapPoint*>)> DeleteCallbackType;
  typedef boost::function<void()> ResetCallbackType;
  typedef boost::function<void (MapMakerBase::State, double)> StateCallbackType;

  // Two constructors because the client will not receive an INIT or RESET request, so it
  // shouldn't have to provide callback functions for those
  
  /** @brief The version called by the server, since only the server will receive an INIT request
   * 
   *  Calls the Initialize() function to connect to the other NetworkManager
   *  @param init A boost function that will be called for INIT messages
   *  @param add A boost function that will be called for ADD messages
   *  @param del A boost function that will be called for DELETE messages
   *  @param role A name for the calling code, used for debugging NetworkDictionary problems */
  NetworkManager(InitCallbackType init, AddCallbackType add, DeleteCallbackType del, ResetCallbackType res, std::string role);
  
  /** @brief The version called by the client, without the INIT callback argument, but with a STATE callback argument
   * 
   * Calls the Initialize() function to connect to the other NetworkManager
   *  @param add A boost function that will be called for ADD messages
   *  @param del A boost function that will be called for DELETE messages
   *  @param st  A boost function that will be called for STATE messages
   *  @param role A name for the calling code, used for debugging NetworkDictionary problems */
  NetworkManager(AddCallbackType add, DeleteCallbackType del, StateCallbackType st, std::string role);
  
  /// Stops the running thread
  ~NetworkManager();
  
  /** @brief Call the map reset method on the other NetworkManager
   * 
   *  This function executes in the caller's thread and blocks until a response is received.
   *  Also calls Reset() */
  void CallReset();
  
  /** @brief Get the number of messages sitting in incoming queue
   *  @return The queue size */
  int IncomingQueueSize(){ return mIncomingQueue.size(); } 
  
  /// Get the next message from the incoming queue, perform any message-to-object conversions, and call callback function
  void HandleNextIncoming();
  
  /// Set a function that will be called the instant a new ADD message comes in
  /** @param f The boost function that will be called */
  void SetNewAddCallback(boost::function<void()> f){ mNewAddCallbackWrapper = f; }
  
  // Functions for sending data 
  /** @brief Call the map initialization method on the other NetworkManager
   * 
   *  This function executes in the caller's thread and blocks until a response is received
   *  @param pMKF Pointer to the MultiKeyFrame to send
   *  @return Did the initialization succeed? */
  bool CallInit(MultiKeyFrame* pMKF);
  
  /** @brief Send a MultiKeyFrame as an ADD message, which will make the recipient add it to its own map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param pMKF Pointer to the MultiKeyFrame to send */
  void SendAdd(MultiKeyFrame* pMKF);
  
  /** @brief Send a set of MapPoints as an ADD message, which will make the recipient add them to its own map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param spPoints Pointers to the MapPoints to send*/
  void SendAdd(std::set<MapPoint*> spPoints);
  
  /** @brief Send a set of MapPoints as a DELETE message, which will make the recipient delete them from its own map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param spPoints Pointers to the MapPoints to send*/
  void SendDelete(std::set<MapPoint*> spPoints);
  
  /** @brief Send a single MultiKeyFrame as a DELETE message, which will make the recipient delete it from its own map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param pMKF Pointer to the MultiKeyFrame to send */
  void SendDelete(MultiKeyFrame* pMKF);
  
  /** @brief Send a set of MultiKeyFrames and MapPoints as a DELETE message, which will make the recipient delete them from its own map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param pMKF Pointer to the MultiKeyFrame to send */
  void SendDelete(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints);
  
  /** @brief Send a single MultiKeyFrame as an UPDATE message, which will make the recipient update the data already existing in their map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param pMKF Pointer to the MultiKeyFrame to send */
  void SendUpdate(MultiKeyFrame* pMKF);
  
  /** @brief Send a set of MapPoints and MultiKeyFrames as an UPDATE message, which will make the recipient update the data already existing in their map
   * 
   *  This function pushes a message to the outgoing queue and thus does not block
   *  @param spMKFs Pointers to the MultiKeyFrames to send
   *  @param spPoints Pointers to the MapPoints to send */
  void SendUpdate(std::set<MultiKeyFrame*> spMKFs, std::set<MapPoint*> spPoints);
  
  void SendOutliers(std::vector<std::pair<KeyFrame*, MapPoint*> >& vOutliers);
  
  void SendState(MapMakerBase::State state, double dMaxCov);
  
  void ClearIncomingQueue();
  
  //void RemoveFromDictionary(MultiKeyFrame* pMKF);
  
  
protected:
  
  /// Connect to the other NetworkManager, start own thread of execution
  void Initialize();
  
  /// Clears the incoming and outgoing message queues, clears the NetworkDictionary
  void Reset();
  
  /// Calls callbackQueue's available callbacks, launched in a separate thread by run()
  /** @param rate The rate at which callbacks are called */
  void Spinning(ros::Rate rate);
  
  /// The function that is run in its own thread, main loop lives heres
  virtual void run();
  
  /** @brief The callback called by the ROS service server when a new message is received
   * 
   *  If the message is of type INIT, calls the appropriate callback immediately, otherwise
   *  pushes message to the incoming queue. Calls the "NewAddCallback" function immediately if
   *  the message type was ADD and the callback was set with SetNewAddCallback 
   *  @param request The ROS service request, contains incoming data
   *  @param response The ROS service response, currently unused
   *  @return Did the callback succeed? */
  bool ModifyMapCallback(mcptam::ModifyMap::Request &request, mcptam::ModifyMap::Response &response);
  
  /// Pops a message off the outgoing queue, and tries to send it until it succeeds
  void HandleNextOutgoing();
  
  // Conversions to and from a MultiKeyFrame
  /** @brief Converts a MultiKeyFrame to an ADD message, encodes all data
   *  @param mkf The MultiKeyFrame to convert
   *  @param mkf_msg The message */
  void MultiKeyFrame_To_AddMsg(MultiKeyFrame &mkf, mcptam::NetworkMultiKeyFrame &mkf_msg);
  
  /** @brief Converts a MultiKeyFrame to an UPDATE message, encodes all data
   *  @param mkf The MultiKeyFrame to convert
   *  @param mkf_msg The message */
  void MultiKeyFrame_To_UpdateMsg(MultiKeyFrame &mkf, mcptam::NetworkMultiKeyFrame &mkf_msg);
  
  /** @brief Converts a MultiKeyFrame to a DELETE message, encodes only the id 
   *  @param mkf The MultiKeyFrame to convert
   *  @param mkf_msg The message */
  void MultiKeyFrame_To_DeleteMsg(MultiKeyFrame &mkf, mcptam::NetworkMultiKeyFrame &mkf_msg);
  
  /** @brief Converts an ADD message to a MultiKeyFrame, reconstructs all data
   *  @param mkf_msg The message
   *  @return Pointer to the reconstructed MultiKeyFrame */
  MultiKeyFrame* AddMsg_To_MultiKeyFrame(mcptam::NetworkMultiKeyFrame &mkf_msg);
  
  /** @brief Applies an UPDATE message to the MultiKeyFrame it encodes for
   *  @param mkf_msg The message */
  void UpdateMsg_ApplyTo_MultiKeyFrame(mcptam::NetworkMultiKeyFrame &mkf_msg);
  
  /** @brief Converts a DELETE message to a MultiKeyFrame, reconstructs all data
   *  @param mkf_msg The message
   *  @return Pointer to the reconstructed MultiKeyFrame */
  MultiKeyFrame* DeleteMsg_To_MultiKeyFrame(mcptam::NetworkMultiKeyFrame &mkf_msg);
  
  // Conversions to and from a KeyFrame
  /** @brief Converts a KeyFrame to an ADD message, encodes all measurements and data needed by MakeKeyFrame_Lite
   *  @param kf The KeyFrame to convert
   *  @param kf_msg The message */
  void KeyFrame_To_AddMsg(KeyFrame &kf, mcptam::NetworkKeyFrame &kf_msg);
  
  /** @brief Converts a KeyFrame to an UPDATE message, encodes only poses and scene depth
   *  @param kf The KeyFrame to convert
   *  @param kf_msg The message */
  void KeyFrame_To_UpdateMsg(KeyFrame &kf, mcptam::NetworkKeyFrame &kf_msg);
  
  /** @brief Applies an ADD message to a blank KeyFrame, reconstructs all data
   *  @param kf_msg The message
   *  @param kf The KeyFrame that will be modified with the ADD message data. Note that KeyFrames don't get managed by the NetworkDictionary
   *            like MapPoints and MultiKeyFrames, so this function can't generate a new KeyFrame on its own. */
  void AddMsg_To_KeyFrame(mcptam::NetworkKeyFrame &kf_msg, KeyFrame &kf);
  
  /** @brief Applies an UPDATE message to the KeyFrame it encodes for
   *  @param kf_msg The message */
  void UpdateMsg_ApplyTo_KeyFrame(mcptam::NetworkKeyFrame &kf_msg);
  
  // Conversion to and from a MapPoint
  /** @brief Converts a MapPoint to an ADD message, encodes all data except for RefreshPixelVector intermediaries
   *  @param point The MapPoint to convert
   *  @param point_msg The message */
  void MapPoint_To_AddMsg(MapPoint& point, mcptam::NetworkMapPoint &point_msg);
  
  /** @brief Converts a MapPoint to an UPDATE message, encodes all position and pixel vector data
   *  @param point The MapPoint to convert
   *  @param point_msg The message */
  void MapPoint_To_UpdateMsg(MapPoint& point, mcptam::NetworkMapPoint &point_msg);
  
  /** @brief Converts a MapPoint to a DELETE message, encodes only the id 
   *  @param point The MapPoint to convert
   *  @param point_msg The message */
  void MapPoint_To_DeleteMsg(MapPoint& point, mcptam::NetworkMapPoint &point_msg);
  
  /** @brief Converts a an ADD message to a MapPoint, reconstructs all data
   *  @param point_msg The message 
   *  @return Pointer to the new MapPoint object */
  MapPoint* AddMsg_To_MapPoint(mcptam::NetworkMapPoint &point_msg);
  
  /** @brief Applies an UPDATE message to the MapPoint it encodes for
   *  @param point_msg The message */
  void UpdateMsg_ApplyTo_MapPoint(mcptam::NetworkMapPoint &point_msg);
  
  /** @brief Converts a DELETE message to the MapPoint it represents
   *  @param point_msg The message 
   *  @return Pointer the MapPoint that should be deleted */
  MapPoint* DeleteMsg_To_MapPoint(mcptam::NetworkMapPoint &point_msg);
  
  void OutliersMsg_ApplyTo_Affected(mcptam::NetworkOutlier& outlier_msg);
  
  void Outlier_To_OutlierMsg(std::pair<KeyFrame*, MapPoint*>& outlier, mcptam::NetworkOutlier& outlier_msg);

  //NetworkDictionary mDict;   ///< The NetworkDictionary
  Dictionary<MapPoint> mMapPointDict;            ///< The MapPoint Dictionary
  Dictionary<MultiKeyFrame> mMultiKeyFrameDict;  ///< The MultiKeyFrame Dictionary
  //Dictionary<Measurement> mMeasurementDict;      ///< The Measurement Dictionary, used only to keep track of which measurements have been sent
  std::map<std::tuple<KeyFrame*, std::string, int>, std::tuple<int, Measurement*> > mmMeasDebugSentAdd;
  std::map<std::tuple<KeyFrame*, std::string, int>, std::tuple<int, Measurement*> > mmMeasDebugReceivedAdd;
  std::map<std::tuple<KeyFrame*, std::string, int>, std::tuple<int, Measurement*> > mmMeasDebugSentUpdate;
  std::map<std::tuple<KeyFrame*, std::string, int>, std::tuple<int, Measurement*> > mmMeasDebugReceivedUpdate;
  std::set<std::tuple<KeyFrame*, std::string, int> > msMeasDebugUnaccounted;
  std::map<std::tuple<KeyFrame*, std::string, int>, int > mmMeasDebugAlreadyDeleted;
  long int mnSeqSend, mnSeqReceived;
  
  std::queue<mcptam::ModifyMap::Request*> mIncomingQueue;  ///< Incoming queue of messages
  std::queue<mcptam::ModifyMap::Request*> mOutgoingQueue;  ///< Outgoing queue of messages
  
  boost::function<void()> mNewAddCallbackWrapper;  ///< Boost function to call immediately when new ADD message arrives
  InitCallbackType mInitCallbackWrapper;           ///< Boost function to call when processing INIT message
  AddCallbackType mAddCallbackWrapper;             ///< Boost function to call when processing ADD message
  DeleteCallbackType mDeleteCallbackWrapper;       ///< Boost function to call when processing DELETE message
  ResetCallbackType mResetCallbackWrapper;
  StateCallbackType mStateCallbackWrapper;
  
  ros::NodeHandle mNodeHandle;             ///< Global node handle
  ros::NodeHandle mNodeHandlePriv;         ///< Private namespace node handle
  ros::ServiceClient mModifyMapClient;     ///< ROS Client that calls other NetworkManager
  ros::ServiceServer mModifyMapServer;     ///< ROS Server that accepts calls from other NetworkManager
  ros::CallbackQueue mCallbackQueue;         ///< Custom callback queue so we can spin just for our own callbacks instead of a node-wide spin
  
  std::string mRole;    ///< Role of the code (client or server), used for debugging NetworkDictionary
};

#endif

