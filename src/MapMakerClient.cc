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
// Large parts of this code are from the original PTAM, which are
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/MapMakerClient.h>
#include <mcptam/MapPoint.h>
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/Map.h>
#include <mcptam/BundleAdjusterBase.h>
#include <std_srvs/Empty.h>

MapMakerClient::MapMakerClient(Map& map)
  : MapMakerBase(map, false)
  , MapMakerClientBase(map)
  // The network manager needs to be bound to the appropriate callback functions
  , mNetworkManager(boost::bind(&MapMakerClient::AddCallback, this, _1, _2),
                    boost::bind(&MapMakerClient::DeleteCallback, this, _1, _2),
                    boost::bind(&MapMakerClient::StateCallback, this, _1, _2), "Client")
{
  ROS_DEBUG("MapMakerClient: Starting constructor");
  ROS_DEBUG("MapMakerClient: Resetting");

  Reset();
  start();
}

MapMakerClient::~MapMakerClient()
{
  stop();  // CVD::Thread function, makes shouldStop() return true
  ROS_DEBUG("MapMakerClient: Waiting for run thread to die");
  join();  // CVD::Thread function
  ROS_DEBUG("MapMakerClient: Run thread has died.");
}

void MapMakerClient::RequestReset()
{
  // No prior processing necessary, just call the base class function to signal
  // a request
  MapMakerBase::RequestReset();
}

void MapMakerClient::Reset()
{
  ROS_DEBUG("MapMakerClient: Reset");

  ROS_DEBUG("MapMakerClient: calling reset on network manager");
  mNetworkManager.CallReset();
  ROS_DEBUG("MapMakerClient: After network manager reset");

  MapMakerClientBase::Reset();
  MapMakerBase::Reset();  // resets the actual map

  // not dealing with special initialization phase in client/server model right now
  // We have to set the state to running, otherwise the Tracker will try to feed us MKFs continuously
  // mState = MM_RUNNING;
}

// This executes in its own thread
void MapMakerClient::run()
{
  ros::Rate loopRate(500);
  ros::Rate publishRate(10);
  ros::Duration publishDur = publishRate.expectedCycleTime();
  ros::Time lastPublishTime = ros::Time::now();

  while (!shouldStop() &&
         ros::ok())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
  {
    if (ResetRequested())
    {
      Reset();
      continue;
    }

    while (mNetworkManager.IncomingQueueSize() > 0 && !ResetRequested())
      mNetworkManager.HandleNextIncoming();  // This will call the appropriate callback function

    if (!mMap.mbGood)  // Nothing to do if there is no map yet!
    {
      loopRate.sleep();
      continue;
    }

    if (ResetRequested())
    {
      Reset();
      continue;
    }

    HandleBadPoints();

    if (ResetRequested())
    {
      Reset();
      continue;
    }

    if (TrackerQueueSize() > 0)
      AddMultiKeyFrameFromTopOfQueue();  // Add MKF to map

    if (ResetRequested())
    {
      Reset();
      continue;
    }

    loopRate.sleep();
  }
}

// The tracker entry point for adding a new multikeyframe;
// the tracker thread doesn't want to hang about, so
// just dumps it on the top of the mapmaker's queue to
// be dealt with later, and return.
void MapMakerClient::AddMultiKeyFrame(MultiKeyFrame*& pMKF_Incoming)
{
  ROS_INFO_STREAM("MapMakerClient: Incoming MKF, mean depth: " << pMKF_Incoming->mdTotalDepthMean);

  MultiKeyFrame* pMKF = pMKF_Incoming;  // take posession
  pMKF_Incoming = NULL;                 // set original to null, tracker will have to make new MKF

  ProcessIncomingKeyFrames(*pMKF);

  for (KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);
    if (kf.mpSBI)
    {
      delete kf.mpSBI;  // Mapmaker uses a different SBI than the tracker, so will re-gen its own
      kf.mpSBI = NULL;
    }
  }

  if (mState == MM_INITIALIZING)
  {
    ROS_INFO("============== REMOVING IMAGES ===========");
    pMKF->RemoveImages();  // don't need images when in initializing state
  }

  boost::mutex::scoped_lock lock(mQueueMutex);
  mqpMultiKeyFramesFromTracker.push_back(pMKF);
  lock.unlock();
}

// Entry point for Tracker to initialize the map. Here, the tracker DOES
// hang around and wait for the function to finish, because what would it do
// otherwise with no map?
bool MapMakerClient::Init(MultiKeyFrame*& pMKF_Incoming, bool bPutPlaneAtOrigin)
{
  MultiKeyFrame* pMKF = pMKF_Incoming;  // take posession
  pMKF_Incoming = NULL;                 // set original to null, tracker will have to make new MKF

  ProcessIncomingKeyFrames(*pMKF);

  // Encode "bPutPlaneAtOrigin" into MKF's fixed flag to pass through network manager
  if (bPutPlaneAtOrigin)
    pMKF->mbFixed = false;
  else
    pMKF->mbFixed = true;

  if (!mNetworkManager.CallInit(pMKF))  // didn't intialize properly
  {
    ROS_WARN("MapMakerClient: Failed initializing map!");
    delete pMKF;
    return false;
  }

  for (KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
    it->second->MakeKeyFrame_Rest();

  mMap.mlpMultiKeyFrames.push_back(pMKF);

  while (mMap.mlpPoints.size() == 0 && ros::ok())
  {
    ROS_INFO("MapMakerClient: Waiting for initial points to show up...");
    ros::Duration(0.2).sleep();
  }

  ROS_INFO_STREAM("MapMakerClient: Got " << mMap.mlpPoints.size() << " initial points");

  mMap.mbGood = true;

  return true;
}

// Add a MultiKeyFrame from the internal queue to the map
void MapMakerClient::AddMultiKeyFrameFromTopOfQueue()
{
  boost::mutex::scoped_lock lock(mQueueMutex);

  if (mqpMultiKeyFramesFromTracker.size() == 0)
    return;

  MultiKeyFrame* pMKF = mqpMultiKeyFramesFromTracker.front();
  lock.unlock();

  for (KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);

    // In the interval between the Tracker recording the measurements and then releasing
    // the point locks, and here, points might have been made bad. Check for bad points,
    // remove measurements.
    for (MeasPtrMap::iterator iter = kf.mmpMeasurements.begin(); iter != kf.mmpMeasurements.end();)
    {
      MapPoint& point = *(iter->first);
      ++iter;  // advance iterator so if erasure needed it will not be invalidated

      if (point.mbBad)  // Point is bad, possibly sitting in trash, so remove measurement now
      {
        kf.EraseMeasurementOfPoint(&point);
        int nErased = point.mMMData.spMeasurementKFs.erase(&kf);
        ROS_ASSERT(nErased);
      }
    }

    if (!kf.NoImage())
      kf.MakeSBI();  // only needed for relocalizer
  }

  // When initializing, we don't need to send or keep images for the 2nd MKF,
  // so strip out images before sending/saving
  if (mState == MM_INITIALIZING)
  {
    if (mMap.mlpMultiKeyFrames.size() > 1)
    {
      // Get rid of MKF at back of map
      mMap.mlpMultiKeyFrames.back()->mbDeleted = true;
      mMap.MoveDeletedMultiKeyFramesToTrash();
    }
  }

  mNetworkManager.SendAdd(pMKF);  // Send the new multikeyframe to the server
  mMap.mlpMultiKeyFrames.push_back(pMKF);

  lock.lock();
  mqpMultiKeyFramesFromTracker.pop_front();
  lock.unlock();
}

// Deletes points that the tracker considered outliers, sends an update to the server telling it to do likewise
void MapMakerClient::HandleBadPoints()
{
  MarkOutliersAsBad();  // from MapMakerClientBase

  std::set<MapPoint*> spBadPoints = mMap.MoveBadPointsToTrash();  // the points that were trashed

  // ROS_INFO_STREAM("About to send "<<spBadPoints.size()<<" bad points to server");

  if (spBadPoints.size() > 0)
    mNetworkManager.SendDelete(spBadPoints);

  mMap.EmptyTrash();
}

// Function that mNetworkManager calls when an "add" message from the server is processed
void MapMakerClient::AddCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints)
{
  ROS_ASSERT(spMultiKeyFrames.size() == 0);  // server should never try to add MKF to client

  ROS_INFO_STREAM("MapMakerClient: Adding " << spPoints.size() << " points");

  for (std::set<MapPoint*>::iterator it = spPoints.begin(); it != spPoints.end(); it++)
  {
    MapPoint& point = *(*it);
    mMap.mlpPoints.push_back(&point);
  }
}

// Function that mNetworkManger calls when a "delete" message from the server is processed
void MapMakerClient::DeleteCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints)
{
  for (std::set<MultiKeyFrame*>::iterator it = spMultiKeyFrames.begin(); it != spMultiKeyFrames.end(); it++)
  {
    MultiKeyFrame* pMKF = *it;
    ROS_ASSERT(pMKF);  // Should not be null pointer, network manager should only have filled in valid pointers
    pMKF->mbDeleted = true;
  }

  for (std::set<MapPoint*>::iterator it = spPoints.begin(); it != spPoints.end(); it++)
  {
    MapPoint* pPoint = *it;
    ROS_ASSERT(pPoint);  // Should not be null pointer, network manager should only have filled in valid pointers
    pPoint->mbDeleted = true;
  }

  mMap.MoveDeletedMultiKeyFramesToTrash();
  mMap.MoveDeletedPointsToTrash();
}

// Function that mNetworkManger calls when a "state" message from the server is processed
void MapMakerClient::StateCallback(MapMakerBase::State state, double dMaxCov)
{
  // just switched to running state
  if (mState == MM_INITIALIZING && state == MM_RUNNING)
    ClearIncomingQueue();

  if (mState == MM_RUNNING && state == MM_INITIALIZING)
  {
    ROS_ERROR("MapMakerClient: We were RUNNING, and server is telling us we are INITIALIZING! Should be impossible");
    ROS_BREAK();
  }

  mState = state;
  mdMaxCov = dMaxCov;
}
