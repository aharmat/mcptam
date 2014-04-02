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

#include <mcptam/MapMakerServer.h>
#include <mcptam/MapPoint.h>
#include <mcptam/Map.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/BundleAdjusterMulti.h>

MapMakerServer::MapMakerServer(Map &map, TaylorCameraMap &cameras, BundleAdjusterBase &bundleAdjuster)
: MapMakerBase(map, true)
, MapMakerServerBase(map, cameras, bundleAdjuster)
 // The network manager needs to be bound to the appropriate callback functions
, mNetworkManager(boost::bind(&MapMakerServer::InitCallback, this, _1), boost::bind(&MapMakerServer::AddCallback, this, _1, _2), 
                  boost::bind(&MapMakerServer::DeleteCallback, this, _1, _2), boost::bind(&MapMakerServer::ResetCallback, this), "Server")
{
  mNetworkManager.SetNewAddCallback(boost::bind(&MapMakerServer::NewAddCallback, this));
  dynamic_cast<BundleAdjusterMulti*>(&mBundleAdjuster)->SetUpdateCallback(boost::bind(&MapMakerServer::SendUpdate, this, _1, _2));
  
  Reset();  // act like we got a reset request from client on startup
  start();
}

MapMakerServer::~MapMakerServer()
{
  stop(); // CVD::Thread function, makes shouldStop() return true
  ROS_DEBUG("MapMakerServer: Waiting for run thread to die");
  join(); // CVD::Thread function
  ROS_DEBUG("MapMakerServer: Run thread has died.");
}

void MapMakerServer::Reset()
{
  ROS_DEBUG("MapMakerServer: Reset");
  
  mbInitialized = false;
  
  MapMakerServerBase::Reset();
  MapMakerBase::Reset(); // resets the actual map
}

void MapMakerServer::ResetCallback()
{
  if(mBundleAdjuster.Running()) 
    mBundleAdjuster.RequestAbort();
    
  RequestReset();
  
  while(!ResetDone() && ros::ok())
  {
    ROS_INFO("MapMakerServer: Waiting for map to reset...");
    ros::Duration(1).sleep();
  }
  
}

// This executes in its own thread
void MapMakerServer::run()
{
  ros::Rate loopRate(50); // 50 Hz
  ros::Rate publishRate(10);
  ros::Duration publishDur = publishRate.expectedCycleTime();
  ros::Time lastPublishTime = ros::Time::now();
  
  while(!shouldStop() && ros::ok())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
  {
    if(ResetRequested()) {Reset(); continue;}
    
    if(!mbInitialized || !mMap.mbGood)  // Nothing to do if there is no map yet!
    {
      loopRate.sleep();
      continue;
    }
    
    if(ResetRequested()) {Reset(); continue;}
    
    if(ros::Time::now() - lastPublishTime > publishDur)
    {
      PublishMapInfo();
      lastPublishTime = ros::Time::now();
    }
    
    // From here on, mapmaker does various map-maintenance jobs in a certain priority
    // Hierarchy. For example, if there's a new key-frame to be added (IncomingQueueSize() is >0)
    // then that takes high priority.
    
    if(ResetRequested()) {Reset(); continue;}
    
    // Should we run local bundle adjustment?
    if(!mBundleAdjuster.ConvergedRecent() && IncomingQueueSize() == 0)  
    {
      ROS_DEBUG("MapMakerServer: Bundle adjusting recent");
      std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliers;
      
      mBundleAdjuster.UseTukey(mMap.mlpMultiKeyFrames.size() > 4);
      mBundleAdjuster.UseTwoStep(mMap.mlpMultiKeyFrames.size() > 4);
      
      int nAccepted = mBundleAdjuster.BundleAdjustRecent(vOutliers);  
      if(nAccepted < 0) // bad
      {
        mnNumConsecutiveFailedBA++;
        if(mnNumConsecutiveFailedBA > MapMakerServerBase::snMaxConsecutiveFailedBA)  // very bad
        {
          ROS_ERROR("MapMakerServer: Recent BA failed, requesting reset");
          RequestResetInternal();
        }
      }
      else
      {
        mnNumConsecutiveFailedBA = 0;          
        HandleOutliers(vOutliers);
        PublishMapVisualization();
      }
    }
    
    if(ResetRequested()) {Reset(); continue;}
    
    // Are there any newly-made map points which need more measurements from older key-frames?
    if(mBundleAdjuster.ConvergedRecent() && IncomingQueueSize() == 0)
    {
      ROS_DEBUG("MapMakerServer: ReFindNewlyMade");
      ReFindNewlyMade();  
    }
    
    if(ResetRequested()) {Reset(); continue;}
    
    // Run global bundle adjustment?
    if(mBundleAdjuster.ConvergedRecent() && !mBundleAdjuster.ConvergedFull() && IncomingQueueSize() == 0)
    {
      ROS_DEBUG("MapMakerServer: Bundle adjusting all");
      std::vector<std::pair<KeyFrame*, MapPoint*> > vOutliers;
      
      mBundleAdjuster.UseTukey(mMap.mlpMultiKeyFrames.size() > 4);
      mBundleAdjuster.UseTwoStep(mMap.mlpMultiKeyFrames.size() > 4);
      
      int nAccepted = mBundleAdjuster.BundleAdjustAll(vOutliers);
      if(nAccepted < 0) // bad
      {
        mnNumConsecutiveFailedBA++;
        if(mnNumConsecutiveFailedBA > MapMakerServerBase::snMaxConsecutiveFailedBA)  // very bad
        {
          ROS_ERROR("MapMakerServer: All BA failed, requesting reset");
          RequestResetInternal();
        }
      }
      else
      {
        mnNumConsecutiveFailedBA = 0;
        HandleOutliers(vOutliers);
        PublishMapVisualization();
      }
    }
    
    if(ResetRequested()) {Reset(); continue;}
    
    // Very low priorty: re-find measurements marked as vOutliers
    if(mBundleAdjuster.ConvergedRecent() && mBundleAdjuster.ConvergedFull() && rand()%20 == 0 && IncomingQueueSize() == 0)
    {
      ROS_DEBUG("MapMakerServer: ReFindFromFailureQueue");
      ReFindFromFailureQueue();
    }
    
    if(ResetRequested()) {Reset(); continue;}
    
    HandleBadEntities();
    
    if(ResetRequested()) {Reset(); continue;}
    
    // Any new key-frames to be added?
    while(IncomingQueueSize() > 0)
      mNetworkManager.HandleNextIncoming(); // Integrate into map data struct, and process
      
    if(ResetRequested()) {Reset(); continue;}
      
    loopRate.sleep();
  }
}

// Deletes bad points, sends message to client, and removes pointers to deleted points from internal queues.
void MapMakerServer::HandleBadEntities()
{
  std::set<MapPoint*> spBadPoints = mMap.MoveBadPointsToTrash();
  std::set<MultiKeyFrame*> spBadMKFs = mMap.MoveBadMultiKeyFramesToTrash();
  
  if(mbInitialized && (spBadPoints.size() > 0 || spBadMKFs.size() > 0))  // only do this if initialized because otherwise client might not have same data yet
  {
    ROS_DEBUG_STREAM("In HandleBadEntities, sending "<<spBadMKFs.size()<<" bad mkfs, "<<spBadPoints.size()<<" bad points");
    mNetworkManager.SendDelete(spBadMKFs, spBadPoints);
  }
  
  EraseBadEntitiesFromQueues();
  mMap.EmptyTrash(); 
}

// Send updates of certain MultiKeyFrames and MapPoints to the client
void MapMakerServer::SendUpdate(std::set<MultiKeyFrame*>& spAdjustedFrames, std::set<MapPoint*>& spAdjustedPoints)
{  
  if(mbInitialized) // only do this if initialized because otherwise client might not have same data yet
  {
    ROS_DEBUG_STREAM("MapMakerServer: sending updates of "<<spAdjustedFrames.size()<<" MKFs and "<<spAdjustedPoints.size()<<" points");
    mNetworkManager.SendUpdate(spAdjustedFrames, spAdjustedPoints);
  }
   
}

// Sends points to the client as an "add" message
void MapMakerServer::SendPoints(MapPointPtrList::iterator begin_it, MapPointPtrList::iterator end_it)
{
  std::set<MapPoint*> spNewSet;
  
  for(; begin_it != end_it; ++begin_it)
    spNewSet.insert(*begin_it);
    
  ROS_DEBUG_STREAM("MapMakerServer: Sending "<<spNewSet.size()<<" points, current map size: "<<mMap.mlpPoints.size());
  mNetworkManager.SendAdd(spNewSet);
}

// Function that mNetworkManager calls when an "init" message from the client is processed
bool MapMakerServer::InitCallback(MultiKeyFrame* pMKF)
{
  if(mbInitialized)
  {
    ROS_ERROR("MapMakerServer: Got initialize action but I'm already initialized, returning false");
    return false;
  }
  
  bool bPutPlaneAtOrigin = pMKF->mbFixed;  // this is how we passed the "put plane at origin" info through network manager
  bool success = InitFromMultiKeyFrame(pMKF, bPutPlaneAtOrigin);   // from MapMakerServerBase
  if(!success)
  {
    ROS_WARN("MapMakerServer: Initialization failed!");
    return false;
  }
  
  mNetworkManager.SendUpdate(mMap.mlpMultiKeyFrames.front());
  SendPoints(mMap.mlpPoints.begin(), mMap.mlpPoints.end());
  
  mbInitialized = true;

  return true;
}

// Function that mNetworkManager calls when an "add" message from the client is processed
void MapMakerServer::AddCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints)
{
  ROS_DEBUG("MapMakerServer: In AddCallback");
  ROS_ASSERT(spPoints.size() == 0); // client should never try to add points to server
  ROS_ASSERT(spMultiKeyFrames.size() == 1); // client should never try to add more than one MKF at a time
  
  MultiKeyFrame *pMKF = *spMultiKeyFrames.begin();
  
  // Save end iterator this way because if you save the end proper, it doesn't act as a normal
  // iterator would after more elements have been added
  std::list<MapPoint*>::iterator prevOneBeforeEnd_it = --mMap.mlpPoints.end();
  
  /*
  if(mMap.mlpMultiKeyFrames.size() >= 20)
  {
    std::cout<<">>>>>> Marking furthest MKF as bad"<<std::endl;
    MarkFurthestMultiKeyFrameAsBad(*pMKF);
    std::cout<<">>>>>> Marked furthest MKF as bad!"<<std::endl;
  }
  */
  
  bool bSuccess = AddMultiKeyFrameAndCreatePoints(pMKF);  // from MapMakerServerBase, this will add points to map
  
  if(!bSuccess)
  {
    pMKF->EraseBackLinksFromPoints();
    ROS_ERROR("Failed adding new points with mkf, sending back as delete");
    mNetworkManager.SendDelete(pMKF);
    delete pMKF;
  }
  else
  {
    mNetworkManager.SendUpdate(mMap.mlpMultiKeyFrames.back());
    SendPoints(++prevOneBeforeEnd_it, mMap.mlpPoints.end());
  }
}

// Function that mNetworkManger calls when a "delete" message from the client is processed
void MapMakerServer::DeleteCallback(std::set<MultiKeyFrame*> spMultiKeyFrames, std::set<MapPoint*> spPoints)
{
  ROS_ASSERT(spMultiKeyFrames.size() == 0);  // client shouldn't send a MKF delete message
  
  ROS_DEBUG("MapMakerServer: In DeleteCallback");
  for(std::set<MapPoint*>::iterator it = spPoints.begin(); it != spPoints.end(); it++)
  {
    (*it)->mbDeleted = true;
  }
  
  mMap.MoveDeletedPointsToTrash();
}

// Called by mNetworkManager immediately when a new "add" message arrives
void MapMakerServer::NewAddCallback()
{
  if(mBundleAdjuster.Running())   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mBundleAdjuster.RequestAbort();
}
