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

//=========================================================================================
//
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/NetworkManager.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/MapPoint.h>
#include <mcptam/Utility.h>
#include <boost/thread/thread.hpp>

void printNetworkOutlier(mcptam::NetworkOutlier& outlier_msg, KeyFrame* pKF, MapPoint* pPoint, int nSeq,
                         std::string action = "")
{
  std::cerr << "(" << nSeq << ") " << outlier_msg.mMKFId << "::" << outlier_msg.mCamName << " [" << pKF << "] <==> "
            << outlier_msg.mapPointId << " [" << pPoint << "]  " << action << std::endl;
}

void printNetworkMeasurement(mcptam::NetworkMeasurement& meas_msg, std::string cameraName, std::string mkfParentId,
                             int nSeq)
{
  std::cerr << "(" << nSeq << ") " << mkfParentId << "::" << cameraName << " <==> " << meas_msg.mapPointId
            << " level: " << (int)meas_msg.nLevel;
  std::cerr << " v2RootPos: [" << meas_msg.v2RootPos[0] << ", " << meas_msg.v2RootPos[1]
            << "] source: " << (int)meas_msg.eSource << std::endl;
}

void printNetworkMeasurement(mcptam::NetworkMeasurement& meas_msg, KeyFrame* pKF, MapPoint* pPoint, int nSeq)
{
  std::cerr << "(" << nSeq << ") " << pKF << " <==> " << pPoint << " level: " << (int)meas_msg.nLevel;
  std::cerr << " v2RootPos: [" << meas_msg.v2RootPos[0] << ", " << meas_msg.v2RootPos[1]
            << "] source: " << (int)meas_msg.eSource << std::endl;
}

void printNetworkMeasurement(mcptam::NetworkMeasurement& meas_msg, std::string mkfParentId, KeyFrame* pKF,
                             MapPoint* pPoint, int nSeq)
{
  std::cerr << "(" << nSeq << ") " << mkfParentId << "::" << pKF->mCamName << " [" << pKF << "] <==> "
            << meas_msg.mapPointId << " [" << pPoint << "] level: " << (int)meas_msg.nLevel;
  std::cerr << " v2RootPos: [" << meas_msg.v2RootPos[0] << ", " << meas_msg.v2RootPos[1]
            << "] source: " << (int)meas_msg.eSource << std::endl;
}

void printNetworkMapPoint(mcptam::NetworkMapPoint& point_msg, MapPoint* pPoint, int nSeq, std::string action = "")
{
  std::cerr << "(" << nSeq << ") " << point_msg.mId << " [" << pPoint << "] " << action << std::endl;
}

NetworkManager::NetworkManager(InitCallbackType init, AddCallbackType add, DeleteCallbackType del,
                               ResetCallbackType res, std::string role)
  : mMapPointDict("MP")
  , mMultiKeyFrameDict("MKF")
  //, mMeasurementDict("MEAS")
  , mInitCallbackWrapper(init)
  , mAddCallbackWrapper(add)
  , mDeleteCallbackWrapper(del)
  , mResetCallbackWrapper(res)
  , mNodeHandlePriv("~")
{
  mRole = role;
  Initialize();
}

NetworkManager::NetworkManager(AddCallbackType add, DeleteCallbackType del, StateCallbackType st, std::string role)
  : mMapPointDict("MP")
  , mMultiKeyFrameDict("MKF")
  //, mMeasurementDict("MEAS")
  , mAddCallbackWrapper(add)
  , mDeleteCallbackWrapper(del)
  , mStateCallbackWrapper(st)
  , mNodeHandlePriv("~")
{
  mRole = role;
  Initialize();
}

// Connect to the other NetworkManager, start own thread of execution
void NetworkManager::Initialize()
{
  // Make sure we are using our own callback queue
  mNodeHandle.setCallbackQueue(&mCallbackQueue);
  mNodeHandlePriv.setCallbackQueue(&mCallbackQueue);

  mModifyMapClient = mNodeHandle.serviceClient<mcptam::ModifyMap>(
      "modify_map", true);  // This will need to be remapped in a launch file
  mModifyMapServer = mNodeHandlePriv.advertiseService("modify_map", &NetworkManager::ModifyMapCallback, this);

  while (!mModifyMapClient.waitForExistence(ros::Duration(5)) && ros::ok())
  {
    ROS_WARN_STREAM("NetworkManager: Waiting for modify_map service to be advertised...");
  }

  mnSeqSend = 1;

  start();
}

NetworkManager::~NetworkManager()
{
  stop();  // CVD::Thread function, makes shouldStop() return true
  ROS_DEBUG("NetworkManager: Waiting for run thread to die");
  join();  // CVD::Thread function
  ROS_DEBUG("NetworkManager: Run thread has died.");

  mModifyMapServer.shutdown();
  mModifyMapClient.shutdown();
}

void NetworkManager::Spinning(ros::Rate rate)
{
  while (!shouldStop() && ros::ok())
  {
    mCallbackQueue.callAvailable();
    rate.sleep();
  }
}

// The function that is run in its own thread, main loop lives heres
void NetworkManager::run()
{
  ros::Rate rate(20);  // run loop at 20 Hz

  // Separate spinning thread is needed because every message sent depends on a service call,
  // which will not complete without this
  boost::thread spinThread(boost::bind(&NetworkManager::Spinning, this, rate));
  while (!shouldStop() && ros::ok())
  {
    HandleNextOutgoing();
    rate.sleep();
  }

  spinThread.join();
}

// Call the map reset method on the other NetworkManager
// Clears the incoming and outgoing message queues, clears the NetworkDictionary
void NetworkManager::CallReset()
{
  mcptam::ModifyMap reset;
  reset.request.action = reset.request.RESET;

  while (!mModifyMapClient.call(reset) && ros::ok())
  {
    ROS_ERROR_STREAM("NetworkManager (" << mRole << "): Failed calling reset! Trying again...");
    mModifyMapClient = mNodeHandle.serviceClient<mcptam::ModifyMap>("modify_map", true);
    while (!mModifyMapClient.waitForExistence(ros::Duration(1)) && ros::ok())
    {
      ROS_WARN_STREAM("NetworkManager: Waiting for modify_map service to reconnect...");
    }
  }

  ROS_ASSERT(reset.response.success);

  // Resetting ourselves needs to be done once the other network manager has been reset
  // otherwise it might send us a new message in the interval between clearing
  // ourselves and requeseting a reset, causing all kinds of problems
  Reset();
}

void NetworkManager::Reset()
{
  while (!mIncomingQueue.empty())
  {
    delete mIncomingQueue.front();
    mIncomingQueue.pop();
  }

  while (!mOutgoingQueue.empty())
  {
    delete mOutgoingQueue.front();
    mOutgoingQueue.pop();
  }

  mMultiKeyFrameDict.Clear();
  mMapPointDict.Clear();
}

void NetworkManager::ClearIncomingQueue()
{
  while (!mIncomingQueue.empty())
  {
    delete mIncomingQueue.front();
    mIncomingQueue.pop();
  }
}

// The callback called by the ROS service server when a new message is received
bool NetworkManager::ModifyMapCallback(mcptam::ModifyMap::Request& request, mcptam::ModifyMap::Response& response)
{
  if (request.action == request.INIT)  // Don't push to incoming queue, call callback immediately
  {
    mnSeqReceived = request.header.seq;

    MultiKeyFrame* pMKF = AddMsg_To_MultiKeyFrame(request.mvMultiKeyFrames[0]);

    if (mInitCallbackWrapper.empty())  // No init callback given!
    {
      ROS_FATAL("NetworkManager: Got an INIT request but no Init callback function specified");
      ros::shutdown();
      response.success = false;
    }
    else
    {
      response.success = mInitCallbackWrapper(pMKF);
    }
  }
  else if (request.action == request.RESET)  // Don't push to incoming queue, call callback immediately
  {
    if (mResetCallbackWrapper.empty())  // No reset callback given!
    {
      ROS_FATAL("NetworkManager: Got a RESET request but no Reset callback function specified");
      ros::shutdown();
      response.success = false;
    }
    else
    {
      mResetCallbackWrapper();  // call the reset callback first (this will wait for map maker to reset)
      Reset();                  // Then reset ourselves
      response.success = true;
    }
  }
  else
  {
    // Do we have to clone the request or can we just save the address of the request??
    // Depends on how memory is managed behind the scenes, but this is safe and guaranteed to work
    mcptam::ModifyMap::Request* pReq = new mcptam::ModifyMap::Request(request);

    // Save the request
    mIncomingQueue.push(pReq);

    if (pReq->action == pReq->ADD)
      ROS_INFO("NetworkManager: Just got ADD message!");

    // If there is a callback for the reception of a new "ADD" event, call it
    if (pReq->action == pReq->ADD && !mNewAddCallbackWrapper.empty())
      mNewAddCallbackWrapper();

    response.success = true;
  }

  return true;
}

// Pops a message off the outgoing queue, and tries to send it until it succeeds
void NetworkManager::HandleNextOutgoing()
{
  if (mOutgoingQueue.size() == 0)
    return;

  mcptam::ModifyMap::Response res;
  mcptam::ModifyMap::Request* pReq = mOutgoingQueue.front();
  mOutgoingQueue.pop();

  ROS_INFO_STREAM("Calling request with stamp: " << pReq->header.stamp << " seq: " << pReq->header.seq);

  if (pReq->action == pReq->DELETE)
  {
    ROS_INFO("NetworkManager: sending DELETE message");
    std::cout << "Sending " << pReq->mvMultiKeyFrames.size() << " MKFs to delete" << std::endl;
    std::cout << "Sending " << pReq->mvPoints.size() << " Points to delete" << std::endl;
  }

  // Very simple method for dealing with a failed service call: keep trying
  // until it succeeds
  while (!mModifyMapClient.call(*pReq, res) && ros::ok())
  {
    ROS_WARN("NetworkManager: Couldn't send request, trying again...");
    mModifyMapClient = mNodeHandle.serviceClient<mcptam::ModifyMap>("modify_map", true);
    while (!mModifyMapClient.waitForExistence(ros::Duration(1)) && ros::ok())
    {
      ROS_WARN_STREAM("NetworkManager: Waiting for modify_map service to reconnect...");
    }
  }

  ROS_ASSERT(res.success);

  if (pReq->action == pReq->ADD)
    ROS_INFO("NetworkManager: Just sent ADD message");

  delete pReq;
}

// Get the next message from the incoming queue, perform any message-to-object conversions, and call callback function
void NetworkManager::HandleNextIncoming()
{
  if (mIncomingQueue.size() == 0)
    return;

  mcptam::ModifyMap::Request* pReq = mIncomingQueue.front();
  mIncomingQueue.pop();

  mnSeqReceived = pReq->header.seq;

  ROS_INFO_STREAM("Got request with stamp: " << pReq->header.stamp << " seq: " << pReq->header.seq);

  // Don't have to check for INIT since they don't get pushed to the queue

  if (pReq->action == pReq->ADD)
  {
    ROS_DEBUG("NetworkManager: Processing ADD message");

    std::set<MultiKeyFrame*> spMKFs;
    std::set<MapPoint*> spPoints;

    for (unsigned i = 0; i < pReq->mvMultiKeyFrames.size(); ++i)
      spMKFs.insert(AddMsg_To_MultiKeyFrame(pReq->mvMultiKeyFrames[i]));

    std::cerr << "---- RECEIVING ADD MESSAGE ----" << std::endl;
    for (unsigned i = 0; i < pReq->mvPoints.size(); ++i)
      spPoints.insert(AddMsg_To_MapPoint(pReq->mvPoints[i]));

    mAddCallbackWrapper(spMKFs, spPoints);
  }
  else if (pReq->action == pReq->DELETE)
  {
    ROS_INFO("NetworkManager: Processing DELETE message");

    std::set<MultiKeyFrame*> spMultiKeyFrames;
    for (unsigned i = 0; i < pReq->mvMultiKeyFrames.size(); ++i)
    {
      MultiKeyFrame* pMKF = DeleteMsg_To_MultiKeyFrame(pReq->mvMultiKeyFrames[i]);
      ROS_ASSERT(pMKF);  // only one side can remove MKFs so it needs to be a valid pointer
      spMultiKeyFrames.insert(pMKF);
    }

    std::cout << "Got " << spMultiKeyFrames.size() << " MKFs to delete" << std::endl;

    std::set<MapPoint*> spPoints;
    for (unsigned i = 0; i < pReq->mvPoints.size(); ++i)
    {
      MapPoint* pPoint = DeleteMsg_To_MapPoint(pReq->mvPoints[i]);
      if (pPoint)
        spPoints.insert(pPoint);
    }

    std::cout << "Got " << spPoints.size() << " Points to delete" << std::endl;

    mDeleteCallbackWrapper(spMultiKeyFrames, spPoints);
  }
  else if (pReq->action == pReq->UPDATE)
  {
    ROS_DEBUG("NetworkManager: Processing UPDATE message");
    std::cerr << "===== RECEIVING UPDATE MSG =====" << std::endl;
    for (unsigned i = 0; i < pReq->mvPoints.size(); ++i)
      UpdateMsg_ApplyTo_MapPoint(pReq->mvPoints[i]);

    for (unsigned i = 0; i < pReq->mvMultiKeyFrames.size(); ++i)
      UpdateMsg_ApplyTo_MultiKeyFrame(pReq->mvMultiKeyFrames[i]);
  }
  else if (pReq->action == pReq->OUTLIERS)
  {
    ROS_DEBUG("NetworkManager: Processing OUTLIERS message");
    std::cerr << "===== RECEIVING OUTLIER MSG =====" << std::endl;
    for (unsigned i = 0; i < pReq->mvOutliers.size(); ++i)
      OutliersMsg_ApplyTo_Affected(pReq->mvOutliers[i]);
  }
  else if (pReq->action == pReq->STATE)
  {
    ROS_DEBUG("NetworkManager: Processing STATE message");
    mStateCallbackWrapper(static_cast<MapMakerBase::State>(pReq->mState), pReq->mdMaxCov);
  }
  else
  {
    ROS_FATAL("NetworkManager: Invalid action specified in request!");
    ros::shutdown();
  }

  delete pReq;
}

// Call the map initialization method on the other NetworkManager, this function blocks!
bool NetworkManager::CallInit(MultiKeyFrame* pMKF)
{
  mcptam::ModifyMap init;
  init.request.action = init.request.INIT;
  init.request.mvMultiKeyFrames.resize(1);

  MultiKeyFrame_To_AddMsg(*pMKF, init.request.mvMultiKeyFrames[0]);

  ROS_DEBUG_STREAM("NetworkManager: In CallInit, sending MKF with id: " << init.request.mvMultiKeyFrames[0].mId);

  while (!mModifyMapClient.call(init) && ros::ok())
  {
    ROS_WARN("NetworkManager: Couldn't call init, trying again...");
    mModifyMapClient = mNodeHandle.serviceClient<mcptam::ModifyMap>("modify_map", true);
    while (!mModifyMapClient.waitForExistence(ros::Duration(1)) && ros::ok())
    {
      ROS_WARN_STREAM("NetworkManager: Waiting for modify_map service to reconnect...");
    }
  }

  return init.response.success;
}

// Send a MultiKeyFrame as an ADD message
void NetworkManager::SendAdd(MultiKeyFrame* pMKF)
{
  mcptam::ModifyMap::Request* pAdd = new mcptam::ModifyMap::Request;
  pAdd->action = pAdd->ADD;
  pAdd->mvMultiKeyFrames.resize(1);

  MultiKeyFrame_To_AddMsg(*pMKF, pAdd->mvMultiKeyFrames[0]);

  pAdd->header.stamp = ros::Time::now();
  pAdd->header.seq = mnSeqSend;
  mnSeqSend++;

  mOutgoingQueue.push(pAdd);
}

//  Send a set of MapPoints as an ADD message
void NetworkManager::SendAdd(std::set<MapPoint*> spPoints)
{
  mcptam::ModifyMap::Request* pAdd = new mcptam::ModifyMap::Request;
  pAdd->action = pAdd->ADD;
  pAdd->mvPoints.resize(spPoints.size());

  unsigned i = 0;
  for (std::set<MapPoint*>::iterator it = spPoints.begin(); it != spPoints.end(); it++, i++)
    MapPoint_To_AddMsg(*(*it), pAdd->mvPoints[i]);

  pAdd->header.stamp = ros::Time::now();
  pAdd->header.seq = mnSeqSend;
  mnSeqSend++;

  mOutgoingQueue.push(pAdd);
}

void NetworkManager::SendDelete(std::set<MapPoint*> spPoints)
{
  SendDelete(std::set<MultiKeyFrame*>(), spPoints);
}

void NetworkManager::SendDelete(MultiKeyFrame* pMKF)
{
  std::set<MultiKeyFrame*> spTemp;
  spTemp.insert(pMKF);
  SendDelete(spTemp, std::set<MapPoint*>());
}

// Send a set of MapPoints as a DELETE message
void NetworkManager::SendDelete(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints)
{
  mcptam::ModifyMap::Request* pDel = new mcptam::ModifyMap::Request;
  pDel->action = pDel->DELETE;
  pDel->mvMultiKeyFrames.resize(spMultiKeyFrames.size());
  pDel->mvPoints.resize(spPoints.size());

  unsigned i;

  i = 0;
  for (std::set<MultiKeyFrame*>::iterator mkf_it = spMultiKeyFrames.begin(); mkf_it != spMultiKeyFrames.end();
       mkf_it++, i++)
    MultiKeyFrame_To_DeleteMsg(*(*mkf_it), pDel->mvMultiKeyFrames[i]);

  std::cerr << "---- SENDING DELETE MESSAGE: " << spPoints.size() << " points -----" << std::endl;
  i = 0;
  for (std::set<MapPoint*>::iterator point_it = spPoints.begin(); point_it != spPoints.end(); point_it++, i++)
    MapPoint_To_DeleteMsg(*(*point_it), pDel->mvPoints[i]);

  pDel->header.stamp = ros::Time::now();
  pDel->header.seq = mnSeqSend;
  mnSeqSend++;

  mOutgoingQueue.push(pDel);
}

void NetworkManager::SendOutliers(std::vector<std::pair<KeyFrame*, MapPoint*>>& vOutliers)
{
  mcptam::ModifyMap::Request* pOutliers = new mcptam::ModifyMap::Request;
  pOutliers->action = pOutliers->OUTLIERS;
  pOutliers->mvOutliers.resize(vOutliers.size());

  std::cerr << "===== SENDING OUTLIER MSG =====" << std::endl;
  for (unsigned i = 0; i < vOutliers.size(); ++i)
  {
    Outlier_To_OutlierMsg(vOutliers[i], pOutliers->mvOutliers[i]);
  }

  pOutliers->header.stamp = ros::Time::now();
  pOutliers->header.seq = mnSeqSend;
  mnSeqSend++;

  mOutgoingQueue.push(pOutliers);
}

// Send a MultiKeyFrame as an UPDATE message
void NetworkManager::SendUpdate(MultiKeyFrame* pMKF)
{
  std::set<MultiKeyFrame*> tempSet;
  tempSet.insert(pMKF);

  SendUpdate(tempSet, std::set<MapPoint*>());
}

// Send a set of MultiKeyFrames and MapPoints as an UPDATE message
void NetworkManager::SendUpdate(std::set<MultiKeyFrame*> spMKFs, std::set<MapPoint*> spPoints)
{
  mcptam::ModifyMap::Request* pUpdate = new mcptam::ModifyMap::Request;
  pUpdate->action = pUpdate->UPDATE;
  pUpdate->mvMultiKeyFrames.resize(spMKFs.size());
  pUpdate->mvPoints.resize(spPoints.size());

  unsigned i = 0;
  for (std::set<MultiKeyFrame*>::iterator it = spMKFs.begin(); it != spMKFs.end(); it++, i++)
    MultiKeyFrame_To_UpdateMsg(*(*it), pUpdate->mvMultiKeyFrames[i]);

  i = 0;
  for (std::set<MapPoint*>::iterator it = spPoints.begin(); it != spPoints.end(); it++, i++)
    MapPoint_To_UpdateMsg(*(*it), pUpdate->mvPoints[i]);

  pUpdate->header.stamp = ros::Time::now();
  pUpdate->header.seq = mnSeqSend;
  mnSeqSend++;

  mOutgoingQueue.push(pUpdate);
}

void NetworkManager::SendState(MapMakerBase::State state, double dMaxCov)
{
  mcptam::ModifyMap::Request* pState = new mcptam::ModifyMap::Request;
  pState->action = pState->STATE;
  pState->mState = state;
  pState->mdMaxCov = dMaxCov;

  pState->header.stamp = ros::Time::now();
  pState->header.seq = mnSeqSend;
  mnSeqSend++;

  mOutgoingQueue.push(pState);
}

// Converts an ADD message to a MultiKeyFrame, reconstructs all data
MultiKeyFrame* NetworkManager::AddMsg_To_MultiKeyFrame(mcptam::NetworkMultiKeyFrame& mkf_msg)
{
  ROS_DEBUG_STREAM("NetworkManager: In AddMsg_To_MultiKeyFrame, incoming mkf id: " << mkf_msg.mId);

  MultiKeyFrame* pMKF = mMultiKeyFrameDict.IdToPtr(mkf_msg.mId, true);
  ROS_ASSERT(pMKF);

  std::stringstream ss;
  ss << mkf_msg.mse3BaseFromWorld;
  ss >> pMKF->mse3BaseFromWorld;

  pMKF->mbFixed = mkf_msg.mbFixed;
  pMKF->mdTotalDepthMean = mkf_msg.mdTotalDepthMean;

  std::cerr << "==== RECEIVING ADD MESSAGE ====" << std::endl;
  for (unsigned i = 0; i < mkf_msg.mvKeyFrames.size(); ++i)
  {
    std::string camName = mkf_msg.mvKeyFrames[i].mCamName;

    // Create new KeyFrame and give it to the MultiKeyFrame
    KeyFrame* pKF = new KeyFrame(pMKF, camName);
    pMKF->mmpKeyFrames[camName] = pKF;

    AddMsg_To_KeyFrame(mkf_msg.mvKeyFrames[i], *pKF);
  }

  if (pMKF->NoImages())
  {
    ROS_INFO_STREAM("Received MKF has no images, removing from dictionary");
    mMultiKeyFrameDict.Remove(pMKF, mRole);
  }

  return pMKF;
}

// Converts a MultiKeyFrame to an ADD message, encodes all data
void NetworkManager::MultiKeyFrame_To_AddMsg(MultiKeyFrame& mkf, mcptam::NetworkMultiKeyFrame& mkf_msg)
{
  mkf_msg.mId = mMultiKeyFrameDict.PtrToId(&mkf, true);

  ROS_DEBUG_STREAM("NetworkManager: In MultiKeyFrame_To_AddMsg, outgoing mkf id: " << mkf_msg.mId);

  std::stringstream ss;
  ss << mkf.mse3BaseFromWorld;
  mkf_msg.mse3BaseFromWorld = ss.str();

  mkf_msg.mbFixed = mkf.mbFixed;
  mkf_msg.mdTotalDepthMean = mkf.mdTotalDepthMean;

  std::cerr << "===== SENDING ADD MSG =====" << std::endl;
  for (KeyFramePtrMap::iterator it = mkf.mmpKeyFrames.begin(); it != mkf.mmpKeyFrames.end(); it++)
  {
    mkf_msg.mvKeyFrames.push_back(mcptam::NetworkKeyFrame());
    KeyFrame_To_AddMsg(*it->second, mkf_msg.mvKeyFrames.back());
  }

  if (mkf.NoImages())
  {
    ROS_INFO_STREAM("Sent MKF has no images, removing from dictionary");
    mMultiKeyFrameDict.Remove(&mkf, mRole);
  }
}

// Converts a MultiKeyFrame to an UPDATE message, encodes all data
void NetworkManager::MultiKeyFrame_To_UpdateMsg(MultiKeyFrame& mkf, mcptam::NetworkMultiKeyFrame& mkf_msg)
{
  mkf_msg.mId = mMultiKeyFrameDict.PtrToId(&mkf, false);

  ROS_DEBUG_STREAM("NetworkManager: In MultiKeyFrame_To_UpdateMsg, outgoing mkf id: " << mkf_msg.mId);

  std::stringstream ss;
  ss << mkf.mse3BaseFromWorld;
  mkf_msg.mse3BaseFromWorld = ss.str();

  mkf_msg.mbFixed = mkf.mbFixed;
  mkf_msg.mdTotalDepthMean = mkf.mdTotalDepthMean;

  std::cerr << "==== SENDING UPDATE MESSAGE =======" << std::endl;
  for (KeyFramePtrMap::iterator it = mkf.mmpKeyFrames.begin(); it != mkf.mmpKeyFrames.end(); it++)
  {
    mcptam::NetworkKeyFrame kf_msg;
    KeyFrame_To_UpdateMsg(*it->second, kf_msg);
    mkf_msg.mvKeyFrames.push_back(kf_msg);
  }
}

// Converts a MultiKeyFrame to a DELETE message, encodes only the id
void NetworkManager::MultiKeyFrame_To_DeleteMsg(MultiKeyFrame& mkf, mcptam::NetworkMultiKeyFrame& mkf_msg)
{
  mkf_msg.mId = mMultiKeyFrameDict.PtrToId(&mkf, false);
  mMultiKeyFrameDict.Remove(&mkf, mRole);
}

// Applies an UPDATE message to the MultiKeyFrame it encodes for
void NetworkManager::UpdateMsg_ApplyTo_MultiKeyFrame(mcptam::NetworkMultiKeyFrame& mkf_msg)
{
  ROS_DEBUG_STREAM("NetworkManager: In UpdateMsg_ApplyTo_MultiKeyFrame, incoming mkf id: " << mkf_msg.mId);

  MultiKeyFrame* pMKF = mMultiKeyFrameDict.IdToPtr(mkf_msg.mId, false);
  if (!pMKF)
  {
    ROS_ERROR_STREAM("Couldn't convert MKF id to pointer, removed by: " << mMultiKeyFrameDict.WhoRemoved(mkf_msg.mId));
    ROS_BREAK();
  }

  std::stringstream ss;
  ss << mkf_msg.mse3BaseFromWorld;
  ss >> pMKF->mse3BaseFromWorld;

  pMKF->mbFixed = mkf_msg.mbFixed;
  pMKF->mdTotalDepthMean = mkf_msg.mdTotalDepthMean;

  std::cerr << "===== RECEIVING UPDATE MESSAGE ========" << std::endl;
  for (unsigned j = 0; j < mkf_msg.mvKeyFrames.size(); ++j)
  {
    std::string camName = mkf_msg.mvKeyFrames[j].mCamName;
    ROS_ASSERT(pMKF->mmpKeyFrames.count(camName) != 0);  // camName exists in mkf map of keyframes
    UpdateMsg_ApplyTo_KeyFrame(mkf_msg.mvKeyFrames[j]);
  }
}

// Converts a DELETE message to the MultiKeyFrame it represents
MultiKeyFrame* NetworkManager::DeleteMsg_To_MultiKeyFrame(mcptam::NetworkMultiKeyFrame& mkf_msg)
{
  MultiKeyFrame* pMKF = mMultiKeyFrameDict.IdToPtr(mkf_msg.mId, false);

  if (pMKF)
  {
    // Need to find sender's role
    std::string remover;
    if (mRole == "Server")
      remover = "Client";
    else
      remover = "Server";

    mMultiKeyFrameDict.Remove(pMKF, remover);
  }

  // Don't perform assert check, callback should deal with NULL pointer if present

  return pMKF;
}

// Applies an ADD message to a blank KeyFrame, reconstructs all data
void NetworkManager::AddMsg_To_KeyFrame(mcptam::NetworkKeyFrame& kf_msg, KeyFrame& kf)
{
  std::stringstream ss;
  ss << kf_msg.mse3CamFromBase << kf_msg.mse3CamFromWorld;
  ss >> kf.mse3CamFromBase >> kf.mse3CamFromWorld;

  kf.mbActive = true;
  kf.mdSceneDepthMean = kf_msg.mdSceneDepthMean;
  kf.mdSceneDepthSigma = kf_msg.mdSceneDepthSigma;

  for (unsigned i = 0; i < kf_msg.mvMeasurements.size(); ++i)
  {
    mcptam::NetworkMeasurement& meas_msg = kf_msg.mvMeasurements[i];
    MapPoint* pPoint = mMapPointDict.IdToPtr(meas_msg.mapPointId, false);

    if (pPoint)  // if NULL, then map point was deleted while the KeyFrame message was in transit, so don't build a
                 // measurement for it
    {
      // printNetworkMeasurement(meas_msg, kf_msg.mParentId, &kf, pPoint, mnSeqReceived);

      Measurement* pMeas = new Measurement;

      // make sure we haven't already received this
      /*
      ROS_ASSERT(mmMeasDebugReceivedAdd.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)) == 0);

      mmMeasDebugReceivedAdd[std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)] =
      std::make_tuple(mnSeqReceived, pMeas);
      */

      pMeas->bTransferred = true;
      pMeas->eSource = static_cast<Measurement::Src>(meas_msg.eSource);
      pMeas->v2RootPos[0] = meas_msg.v2RootPos[0];
      pMeas->v2RootPos[1] = meas_msg.v2RootPos[1];
      pMeas->nLevel = meas_msg.nLevel;
      pMeas->bSubPix = meas_msg.bSubPix;

      kf.AddMeasurement(pPoint, pMeas);
      // kf.mmpMeasurements[pPoint] = pMeas;
      // pPoint->mMMData.spMeasurementKFs.insert(&kf);
    }
  }

  CVD::Image<CVD::byte> image, mask;
  util::MsgToImage(kf_msg.image, image);
  util::MsgToImage(kf_msg.mask, mask);

  // Set mask before creating rest of keyframe internals
  if (mask.totalsize() > 0)
    kf.SetMask(mask);

  // Don't do deep copy of image since we just created it and nobody else will use it
  // Also don't do glare masking since this would have been done on client side
  if (image.totalsize() > 0)
    kf.MakeKeyFrame_Lite(image, false, false);
}

// Converts a KeyFrame to an ADD message, encodes all measurements and data needed by MakeKeyFrame_Lite
void NetworkManager::KeyFrame_To_AddMsg(KeyFrame& kf, mcptam::NetworkKeyFrame& kf_msg)
{
  kf_msg.mParentId = mMultiKeyFrameDict.PtrToId(kf.mpParent, false);  // parent MKF should already have an id
  kf_msg.mCamName = kf.mCamName;

  std::stringstream ss;
  ss << kf.mse3CamFromBase;
  kf_msg.mse3CamFromBase = ss.str();

  ss.str("");
  ss << kf.mse3CamFromWorld;
  kf_msg.mse3CamFromWorld = ss.str();

  kf_msg.mdSceneDepthMean = kf.mdSceneDepthMean;
  kf_msg.mdSceneDepthSigma = kf.mdSceneDepthSigma;

  kf_msg.mvMeasurements.resize(kf.mmpMeasurements.size());

  unsigned i = 0;
  for (MeasPtrMap::iterator it = kf.mmpMeasurements.begin(); it != kf.mmpMeasurements.end(); it++, i++)
  {
    Measurement& meas = *(it->second);
    MapPoint& point = *(it->first);

    kf_msg.mvMeasurements[i].nLevel = meas.nLevel;
    kf_msg.mvMeasurements[i].bSubPix = meas.bSubPix;
    kf_msg.mvMeasurements[i].eSource = meas.eSource;
    kf_msg.mvMeasurements[i].v2RootPos[0] = meas.v2RootPos[0];
    kf_msg.mvMeasurements[i].v2RootPos[1] = meas.v2RootPos[1];
    kf_msg.mvMeasurements[i].mapPointId =
        mMapPointDict.PtrToId(&point, false);  // map point should have id from when it was received

    // printNetworkMeasurement(kf_msg.mvMeasurements[i], kf_msg.mParentId, &kf, &point, mnSeqSend);

    // Add message should only be generated for a new MKF addition, so none of the measurements should
    // have been transferred yet
    ROS_ASSERT(!meas.bTransferred);

    meas.bTransferred = true;  // mark as transferred
    /*
    if(mmMeasDebugSentAdd.count(std::make_tuple(&kf, kf_msg.mvMeasurements[i].mapPointId, meas.nLevel)))
    {
      ROS_ERROR_STREAM("Already sent this measurement with an ADD, seq:
    "<<std::get<0>(mmMeasDebugSentAdd[std::make_tuple(&kf, kf_msg.mvMeasurements[i].mapPointId, meas.nLevel)])<<"
    current send seq: "<<mnSeqSend);
      ROS_BREAK();
    }

    mmMeasDebugSentAdd[std::make_tuple(&kf, kf_msg.mvMeasurements[i].mapPointId, meas.nLevel)] =
    std::make_tuple(mnSeqSend, &meas);
    */
  }

  util::ImageToMsg(kf.maLevels[0].image, 90, kf_msg.image);
  util::ImageToMsg(kf.maLevels[0].mask, 90, kf_msg.mask);
}

// Converts a KeyFrame to an UPDATE message, encodes, poses, scene depth and measurements to add/delete
void NetworkManager::KeyFrame_To_UpdateMsg(KeyFrame& kf, mcptam::NetworkKeyFrame& kf_msg)
{
  kf_msg.mParentId =
      mMultiKeyFrameDict.PtrToId(kf.mpParent, false);  // parent MKF should already have been assigned an id
  kf_msg.mCamName = kf.mCamName;

  std::stringstream ss;
  ss << kf.mse3CamFromBase;
  kf_msg.mse3CamFromBase = ss.str();

  ss.str("");
  ss << kf.mse3CamFromWorld;
  kf_msg.mse3CamFromWorld = ss.str();

  kf_msg.mdSceneDepthMean = kf.mdSceneDepthMean;
  kf_msg.mdSceneDepthSigma = kf.mdSceneDepthSigma;

  for (MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
  {
    Measurement& meas = *(meas_it->second);
    MapPoint& point = *(meas_it->first);

    if (meas.bTransferred)  // we already sent this one
      continue;

    meas.bTransferred = true;  // mark as sent

    mcptam::NetworkMeasurement meas_msg;
    meas_msg.mapPointId =
        mMapPointDict.PtrToId(&point, false);  // map point should have id from when it was first sent out
    meas_msg.nLevel = meas.nLevel;
    meas_msg.bSubPix = meas.bSubPix;
    meas_msg.eSource = meas.eSource;
    meas_msg.v2RootPos[0] = meas.v2RootPos[0];
    meas_msg.v2RootPos[1] = meas.v2RootPos[1];

    // printNetworkMeasurement(meas_msg, kf_msg.mParentId, &kf, &point, mnSeqSend);
    /*
      if(mmMeasDebugReceivedAdd.count(std::make_tuple(&kf, meas_msg.mapPointId, meas.nLevel)))
      {
        ROS_ERROR_STREAM("We received this meas in seq: "<<std::get<0>(mmMeasDebugReceivedAdd[std::make_tuple(&kf,
      meas_msg.mapPointId, meas.nLevel)])<<" (current seq: "<<mnSeqReceived<<")");
        ROS_ERROR_STREAM("Saved pointer: "<<std::get<1>(mmMeasDebugReceivedAdd[std::make_tuple(&kf, meas_msg.mapPointId,
      meas.nLevel)])<<" current pointer: "<<&meas);
        ROS_BREAK();
      }

      for(int i=0; i < LEVELS; ++i)
      {
        if(i == meas.nLevel)
          continue;

        if(mmMeasDebugReceivedAdd.count(std::make_tuple(&kf, meas_msg.mapPointId, i)))
        {
          ROS_ERROR_STREAM("Already received different level measurement in seq:
      "<<std::get<0>(mmMeasDebugReceivedAdd[std::make_tuple(&kf, meas_msg.mapPointId, i)])<<" (current seq:
      "<<mnSeqSend<<")");
          ROS_ERROR_STREAM("Saved pointer: "<<std::get<1>(mmMeasDebugReceivedAdd[std::make_tuple(&kf,
      meas_msg.mapPointId, i)])<<" current pointer: "<<&meas);
          ROS_ERROR_STREAM("Saved level: "<<i<<" current level: "<<meas.nLevel);
          ROS_BREAK();
        }
      }


      if(mmMeasDebugSentUpdate.count(std::make_tuple(&kf, meas_msg.mapPointId, meas.nLevel)))
      {
        ROS_ERROR_STREAM("Already sent measurement in seq: "<<std::get<0>(mmMeasDebugSentUpdate[std::make_tuple(&kf,
      meas_msg.mapPointId, meas.nLevel)])<<" (current seq: "<<mnSeqSend<<")");
        ROS_ERROR_STREAM("Saved pointer: "<<std::get<1>(mmMeasDebugSentUpdate[std::make_tuple(&kf, meas_msg.mapPointId,
      meas.nLevel)])<<" current pointer: "<<&meas);
        ROS_BREAK();
      }

      for(int i=0; i < LEVELS; ++i)
      {
        if(i == meas.nLevel)
          continue;

        if(mmMeasDebugSentUpdate.count(std::make_tuple(&kf, meas_msg.mapPointId, i)))
        {
          ROS_ERROR_STREAM("Already sent different level measurement in seq:
      "<<std::get<0>(mmMeasDebugSentUpdate[std::make_tuple(&kf, meas_msg.mapPointId, i)])<<" (current seq:
      "<<mnSeqSend<<")");
          ROS_ERROR_STREAM("Saved pointer: "<<std::get<1>(mmMeasDebugSentUpdate[std::make_tuple(&kf,
      meas_msg.mapPointId, i)])<<" current pointer: "<<&meas);
          ROS_ERROR_STREAM("Saved level: "<<i<<" current level: "<<meas.nLevel);
          ROS_BREAK();
        }
      }

      mmMeasDebugSentUpdate[std::make_tuple(&kf, meas_msg.mapPointId, meas.nLevel)] = std::make_tuple(mnSeqSend, &meas);
       */
    kf_msg.mvMeasurements.push_back(meas_msg);
  }
}

// Applies an UPDATE message to the KeyFrame it encodes for
void NetworkManager::UpdateMsg_ApplyTo_KeyFrame(mcptam::NetworkKeyFrame& kf_msg)
{
  MultiKeyFrame* pParent = mMultiKeyFrameDict.IdToPtr(kf_msg.mParentId, false);
  ROS_ASSERT(pParent);
  ROS_ASSERT(pParent->mmpKeyFrames.count(kf_msg.mCamName) != 0);

  KeyFrame& kf = *(pParent->mmpKeyFrames[kf_msg.mCamName]);

  std::stringstream ss;
  ss << kf_msg.mse3CamFromBase << kf_msg.mse3CamFromWorld;
  ss >> kf.mse3CamFromBase >> kf.mse3CamFromWorld;

  kf.mdSceneDepthMean = kf_msg.mdSceneDepthMean;
  kf.mdSceneDepthSigma = kf_msg.mdSceneDepthSigma;

  // ROS_INFO_STREAM(">>>>>> Update KF, "<<kf_msg.mvMeasurements.size()<<" meas to add and
  // "<<kf_msg.mvDeletedMeas.size()<<" meas to delete");
  // ROS_INFO_STREAM(">>>>>> Received seq: "<<mnSeqReceived);

  for (unsigned i = 0; i < kf_msg.mvMeasurements.size(); ++i)
  {
    mcptam::NetworkMeasurement& meas_msg = kf_msg.mvMeasurements[i];
    MapPoint* pPoint = mMapPointDict.IdToPtr(meas_msg.mapPointId, false);

    if (pPoint)  // if NULL, then map point was deleted while the KeyFrame message was in transit, so don't build a
                 // measurement for it
    {
      // printNetworkMeasurement(meas_msg, kf_msg.mParentId, &kf, pPoint, mnSeqReceived);
      /*
      if(msMeasDebugUnaccounted.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)))
      {
        ROS_ERROR("We needed this measurement before for deletion and didn't have it, means it arrived out of order!");
        ROS_BREAK();
      }

      if(mmMeasDebugSentAdd.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)))
      {
        ROS_ERROR_STREAM("Received meas, but we sent this measurement ourself in an ADD with seq:
      "<<std::get<0>(mmMeasDebugSentAdd[std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)])<<", current send
      seq: "<<mnSeqSend);
        ROS_BREAK();
      }

      for(int i=0; i < LEVELS; ++i)
      {
        if(i == meas_msg.nLevel)
          continue;

        if(mmMeasDebugSentAdd.count(std::make_tuple(&kf, meas_msg.mapPointId, i)))
        {
          ROS_ERROR_STREAM("We sent different level measurement in an ADD with seq:
      "<<std::get<0>(mmMeasDebugSentAdd[std::make_tuple(&kf, meas_msg.mapPointId, i)])<<" (current send seq:
      "<<mnSeqSend<<")");
          ROS_ERROR_STREAM("Saved level: "<<i<<" current level: "<<(int)meas_msg.nLevel);
          ROS_BREAK();
        }
      }

      if(mmMeasDebugReceivedUpdate.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)))
      {
        ROS_ERROR_STREAM("We already received this measurement in an UPDATE with seq:
      "<<std::get<0>(mmMeasDebugReceivedUpdate[std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)])<<", current
      receive seq: "<<mnSeqReceived);
        ROS_BREAK();
      }

      for(int i=0; i < LEVELS; ++i)
      {
        if(i == meas_msg.nLevel)
          continue;

        if(mmMeasDebugReceivedUpdate.count(std::make_tuple(&kf, meas_msg.mapPointId, i)))
        {
          ROS_ERROR_STREAM("We received a different level measurement in an UPDATE with seq:
      "<<std::get<0>(mmMeasDebugReceivedUpdate[std::make_tuple(&kf, meas_msg.mapPointId, i)])<<" (current receive seq:
      "<<mnSeqReceived<<")");
          ROS_ERROR_STREAM("Saved level: "<<i<<" current level: "<<(int)meas_msg.nLevel);
          ROS_BREAK();
        }
      }
      */
      Measurement* pMeas = new Measurement;

      // mmMeasDebugReceivedUpdate[std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)] =
      // std::make_tuple(mnSeqReceived, pMeas);

      pMeas->bTransferred = true;
      pMeas->eSource = static_cast<Measurement::Src>(meas_msg.eSource);
      pMeas->v2RootPos[0] = meas_msg.v2RootPos[0];
      pMeas->v2RootPos[1] = meas_msg.v2RootPos[1];
      pMeas->nLevel = meas_msg.nLevel;
      pMeas->bSubPix = meas_msg.bSubPix;

      kf.AddMeasurement(pPoint, pMeas);
      // kf.mmpMeasurements[pPoint] = pMeas;
      // pPoint->mMMData.spMeasurementKFs.insert(&kf);
    }
  }
}

// Converts a MapPoint to an ADD message, encodes all data except for RefreshPixelVector intermediaries
void NetworkManager::MapPoint_To_AddMsg(MapPoint& point, mcptam::NetworkMapPoint& point_msg)
{
  point_msg.mId = mMapPointDict.PtrToId(&point, true);

  // printNetworkMapPoint(point_msg, &point, mnSeqSend, "SENDING ADD");

  point_msg.mbFixed = point.mbFixed;
  point_msg.mbOptimized = point.mbOptimized;

  std::stringstream ss;
  ss << point.mv3WorldPos;
  point_msg.mv3WorldPos = ss.str();

  ss.str("");
  ss << point.mv3PixelRight_W;
  point_msg.mv3PixelRight_W = ss.str();

  ss.str("");
  ss << point.mv3PixelDown_W;
  point_msg.mv3PixelDown_W = ss.str();

  point_msg.mnSourceLevel = point.mnSourceLevel;
  point_msg.mirCenter[0] = point.mirCenter[0];
  point_msg.mirCenter[1] = point.mirCenter[1];

  point_msg.mSourceId = mMultiKeyFrameDict.PtrToId(point.mpPatchSourceKF->mpParent,
                                                   false);  // MKF should have had id assigned when it was received
  point_msg.mSourceCamName = point.mpPatchSourceKF->mCamName;
}

// Converts a MapPoint to an UPDATE message, encodes all position and pixel vector data
void NetworkManager::MapPoint_To_UpdateMsg(MapPoint& point, mcptam::NetworkMapPoint& point_msg)
{
  point_msg.mId = mMapPointDict.PtrToId(&point, false);
  point_msg.mbFixed = point.mbFixed;
  point_msg.mbOptimized = point.mbOptimized;

  std::stringstream ss;
  ss << point.mv3WorldPos;
  point_msg.mv3WorldPos = ss.str();

  ss.str("");
  ss << point.mv3PixelRight_W;
  point_msg.mv3PixelRight_W = ss.str();

  ss.str("");
  ss << point.mv3PixelDown_W;
  point_msg.mv3PixelDown_W = ss.str();
}

// Converts a MapPoint to a DELETE message, encodes only the id
void NetworkManager::MapPoint_To_DeleteMsg(MapPoint& point, mcptam::NetworkMapPoint& point_msg)
{
  point_msg.mId = mMapPointDict.PtrToId(&point, false);

  // printNetworkMapPoint(point_msg, &point, mnSeqSend, "SENDING DELETE");

  mMapPointDict.Remove(&point, mRole);
}

// Converts a an ADD message to a MapPoint, reconstructs all data
MapPoint* NetworkManager::AddMsg_To_MapPoint(mcptam::NetworkMapPoint& point_msg)
{
  MapPoint* pPoint = mMapPointDict.IdToPtr(point_msg.mId, true);

  // printNetworkMapPoint(point_msg, pPoint, mnSeqReceived, "RECEIVING ADD");

  ROS_ASSERT(pPoint);
  pPoint->mbFixed = point_msg.mbFixed;
  pPoint->mbOptimized = point_msg.mbOptimized;

  std::stringstream ss;
  ss << point_msg.mv3WorldPos << point_msg.mv3PixelRight_W << point_msg.mv3PixelDown_W;
  ss >> pPoint->mv3WorldPos >> pPoint->mv3PixelRight_W >> pPoint->mv3PixelDown_W;

  pPoint->mnSourceLevel = point_msg.mnSourceLevel;
  pPoint->mirCenter = CVD::ImageRef(point_msg.mirCenter[0], point_msg.mirCenter[1]);

  MultiKeyFrame* pMKF = mMultiKeyFrameDict.IdToPtr(point_msg.mSourceId, false);
  ROS_ASSERT(pMKF);

  ROS_ASSERT(pMKF->mmpKeyFrames.count(point_msg.mSourceCamName) != 0);

  pPoint->mpPatchSourceKF = pMKF->mmpKeyFrames[point_msg.mSourceCamName];

  return pPoint;
}

// Applies an UPDATE message to the MapPoint it encodes for
void NetworkManager::UpdateMsg_ApplyTo_MapPoint(mcptam::NetworkMapPoint& point_msg)
{
  MapPoint* pPoint = mMapPointDict.IdToPtr(point_msg.mId, false);

  if (pPoint)  // if the point exists
  {
    pPoint->mbFixed = point_msg.mbFixed;
    pPoint->mbOptimized = point_msg.mbOptimized;

    std::stringstream ss;
    ss << point_msg.mv3WorldPos << point_msg.mv3PixelRight_W << point_msg.mv3PixelDown_W;
    ss >> pPoint->mv3WorldPos >> pPoint->mv3PixelRight_W >> pPoint->mv3PixelDown_W;
  }

  // Otherwise the point has already been deleted so don't do anything
}

// Converts a DELETE message to the MapPoint it represents
MapPoint* NetworkManager::DeleteMsg_To_MapPoint(mcptam::NetworkMapPoint& point_msg)
{
  MapPoint* pPoint = mMapPointDict.IdToPtr(point_msg.mId, false);

  if (pPoint)
  {
    // printNetworkMapPoint(point_msg, pPoint, mnSeqReceived, "RECEIVING DELETE");

    // Need to find sender's role
    std::string remover;
    if (mRole == "Server")
      remover = "Client";
    else
      remover = "Server";

    mMapPointDict.Remove(pPoint, remover);
  }

  // Don't perform assert check, if pPoint is NULL then map point has already
  // been deleted and callback should ignore this pointer

  return pPoint;
}

void NetworkManager::Outlier_To_OutlierMsg(std::pair<KeyFrame*, MapPoint*>& outlier,
                                           mcptam::NetworkOutlier& outlier_msg)
{
  KeyFrame* pKF = outlier.first;
  MapPoint* pPoint = outlier.second;

  outlier_msg.mMKFId =
      mMultiKeyFrameDict.PtrToId(pKF->mpParent, false);  // parent MKF should already have been assigned an id
  outlier_msg.mCamName = pKF->mCamName;
  outlier_msg.mapPointId = mMapPointDict.PtrToId(pPoint, false);

  // printNetworkOutlier(outlier_msg, pKF, pPoint, mnSeqSend, "SENDING OUTLIER");
}

void NetworkManager::OutliersMsg_ApplyTo_Affected(mcptam::NetworkOutlier& outlier_msg)
{
  MultiKeyFrame* pMKF = mMultiKeyFrameDict.IdToPtr(outlier_msg.mMKFId, false);

  if (!pMKF)  // MultiKeyFrame already deleted, get out
    return;

  ROS_ASSERT(pMKF->mmpKeyFrames.count(outlier_msg.mCamName));
  KeyFrame& kf = *(pMKF->mmpKeyFrames[outlier_msg.mCamName]);

  MapPoint* pPoint = mMapPointDict.IdToPtr(outlier_msg.mapPointId, false);

  if (!pPoint)  // MapPoint already deleted, get out
    return;

  if (pPoint->mbBad)  // MapPoint labeled bad on client side, no point doing anything more since it'll be deleted soon
    return;

  // printNetworkOutlier(outlier_msg, &kf, pPoint, mnSeqReceived, "RECEIVED OUTLIER");

  if (!kf.mmpMeasurements.count(pPoint))
  {
    ROS_ERROR("Couldn't find measurement of point");

    if (pPoint->mbBad)
    {
      ROS_ERROR("Point is bad, about to be deleted along with all measurements");
    }

    if (pPoint->mbDeleted)
    {
      ROS_ERROR("Point is deleted");
    }

    if (mmMeasDebugSentAdd.count(std::make_tuple(&kf, outlier_msg.mapPointId, -1)))
    {
      ROS_ERROR_STREAM("KF and point appear in debug set, we sent this measurement ourself with an ADD message");
    }

    if (mmMeasDebugReceivedUpdate.count(std::make_tuple(&kf, outlier_msg.mapPointId, -1)))
    {
      ROS_ERROR_STREAM("KF and point appear in debug set, got this measurement with an UPDATE message");
    }

    if (mmMeasDebugAlreadyDeleted.count(std::make_tuple(&kf, outlier_msg.mapPointId, -1)))
    {
      ROS_ERROR_STREAM("We have already deleted this measurement here before, seq: "
                       << mmMeasDebugAlreadyDeleted[std::make_tuple(&kf, outlier_msg.mapPointId, -1)]);
    }

    msMeasDebugUnaccounted.insert(std::make_tuple(&kf, outlier_msg.mapPointId, -1));
    ROS_ERROR_STREAM("Unaccounted measurements: " << msMeasDebugUnaccounted.size());
    if (msMeasDebugUnaccounted.size() > 1)
      ROS_BREAK();
  }

  mmMeasDebugAlreadyDeleted[std::make_tuple(&kf, outlier_msg.mapPointId, -1)] = mnSeqReceived;

  kf.EraseMeasurementOfPoint(pPoint);
  int nErased = pPoint->mMMData.spMeasurementKFs.erase(&kf);
  ROS_ASSERT(nErased);
}

/*
 for(unsigned i=0; i < kf_msg.mvDeletedMeas.size(); ++i)
  {
    mcptam::NetworkMeasurement& meas_msg = kf_msg.mvDeletedMeas[i];
    MapPoint* pPoint = mMapPointDict.IdToPtr(meas_msg.mapPointId, false);

    if(pPoint) // if NULL, then map point was deleted while the KeyFrame message was in transit, so don't do anything
    {
      if(!kf.mmpMeasurements.count(pPoint))
      {
        ROS_ERROR("Couldn't find measurement of point");

        if(pPoint->mbBad)
        {
          ROS_ERROR("Point is bad, about to be deleted along with all measurements");
        }

        if(pPoint->mbDeleted)
        {
          ROS_ERROR("Point is deleted");
        }

        if(mmMeasDebugSentAdd.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)))
        {
          ROS_ERROR_STREAM("KF and point appear in debug set, we sent this measurement ourself with an ADD message");
        }

        if(mmMeasDebugReceivedUpdate.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)))
        {
          ROS_ERROR_STREAM("KF and point appear in debug set, got this measurement with an UPDATE message");
        }

        if(mmMeasDebugAlreadyDeleted.count(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)))
        {
          ROS_ERROR_STREAM("We have already deleted this measurement here before, seq:
 "<<mmMeasDebugAlreadyDeleted[std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)]);
        }

        msMeasDebugUnaccounted.insert(std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel));
        ROS_ERROR_STREAM("Unaccounted measurements: "<<msMeasDebugUnaccounted.size());
        if(msMeasDebugUnaccounted.size() > 1)
          ROS_BREAK();

        continue;
      }

      mmMeasDebugAlreadyDeleted[std::make_tuple(&kf, meas_msg.mapPointId, meas_msg.nLevel)] = mnSeqReceived;

      kf.EraseMeasurementOfPoint(pPoint);
      int nErased = pPoint->mMMData.spMeasurementKFs.erase(&kf);
      ROS_ASSERT(nErased);
    }
  }
*/
/*
void NetworkManager::RemoveFromDictionary(MultiKeyFrame* pMKF)
{
  ROS_ASSERT(!pMKF->mbFixed);
  mMultiKeyFrameDict.Remove(pMKF, mRole);
}
*/
