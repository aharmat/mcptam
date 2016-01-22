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

#include <mcptam/MapMaker.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/Map.h>
#include <mcptam/BundleAdjusterBase.h>
#include <mcptam/MapMakerTiming.h>

MapMaker::MapMaker(Map &map, TaylorCameraMap &cameras, BundleAdjusterBase &bundleAdjuster)
  : MapMakerBase(map, true), MapMakerClientBase(map), MapMakerServerBase(map, cameras, bundleAdjuster)
{
  mCreationTimingPub = mNodeHandlePriv.advertise<mcptam::MapMakerTiming>("timing_mkf_creation", 1, true);
  mLocalTimingPub = mNodeHandlePriv.advertise<mcptam::MapMakerTiming>("timing_local_ba", 1, true);
  mGlobalTimingPub = mNodeHandlePriv.advertise<mcptam::MapMakerTiming>("timing_global_ba", 1, true);

  Reset();
  start();
}

MapMaker::~MapMaker()
{
  stop();  // CVD::Thread function, makes shouldStop() return true
  ROS_DEBUG("MapMaker: Waiting for run thread to die");
  join();  // CVD::Thread function
  ROS_DEBUG("MapMaker: Run thread has died.");
  ;
}

// Requests an abort from the bundle adjuster and sets flags that signal a reset is waiting
void MapMaker::RequestReset()
{
  ROS_DEBUG("MapMaker: RequestReset");

  // Need to ask BA to stop before setting request flag in base class
  if (mBundleAdjuster.Running())
    mBundleAdjuster.RequestAbort();

  MapMakerBase::RequestReset();
}

// Rescale the map by the given scale factor
void MapMaker::RequestRescaling(double dScale)
{
  if (mBundleAdjuster.Running())
    mBundleAdjuster.RequestAbort();

  mbRescale = true;
  mdScale = dScale;
}

// Request the map parameters be written to the given file name
void MapMaker::RequestFileDump(std::string filename)
{
  if (mBundleAdjuster.Running())
    mBundleAdjuster.RequestAbort();

  mbFileDump = true;
  mDumpFileName = filename;
}

// Request that the initialization phase is completed early
void MapMaker::RequestStopInit()
{
  if (mBundleAdjuster.Running())
    mBundleAdjuster.RequestAbort();

  mbStopInit = true;
}

// Resets the parent classes
void MapMaker::Reset()
{
  ROS_DEBUG("MapMaker: Reset");

  MapMakerClientBase::Reset();
  MapMakerServerBase::Reset();
  MapMakerBase::Reset();  // reset the actual map

  // debug
  mbRescale = false;
  mdScale = 1.0;

  // debug
  mbFileDump = false;
  mDumpFileName = "map.dat";

  // testing
  mbStopInit = false;
}

void MapMaker::run()
{
  ros::WallRate loopRate(500);
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

    // Nothing to do if there is no map yet!
    // Also, map making can be "paused" by setting mMap.mbGood to FALSE, such as
    // is MapMakerCalib, so we need to check for this condition throughout the main loop
    if (!mMap.mbGood)
    {
      loopRate.sleep();
      continue;
    }

    // debug
    if (mbRescale)
    {
      ROS_INFO_STREAM("------------- Rescaling to " << mdScale);
      ApplyGlobalScaleToMap(mdScale);
      PublishMapVisualization();
      mbRescale = false;

      mBundleAdjuster.SetNotConverged();
    }

    // debug
    if (mbFileDump)
    {
      ROS_INFO_STREAM("------------- Dumping file to " << mDumpFileName);
      DumpToFile(mDumpFileName);
      mbFileDump = false;
    }

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    if (ros::Time::now() - lastPublishTime > publishDur)
    {
      PublishMapInfo();
      lastPublishTime = ros::Time::now();
    }

    // From here on, mapmaker does various map-maintenance jobs in a certain priority
    // Hierarchy. For example, if there's a new key-frame to be added (IncomingQueueSize() is >0)
    // then that takes high priority.

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    // Should we run local bundle adjustment?
    // Need to do this even if initializing just so the converged recent flag gets set to true,
    // even though no optimization will take place since the map is too small
    if (!mBundleAdjuster.ConvergedRecent() && IncomingQueueSize() == 0)
    {
      ROS_INFO("MapMaker: Bundle adjusting recent");
      std::vector<std::pair<KeyFrame *, MapPoint *>> vOutliers;

      mBundleAdjuster.UseTukey((mState == MM_RUNNING && mMap.mlpMultiKeyFrames.size() > 2));
      mBundleAdjuster.UseTwoStep((mState == MM_RUNNING && mMap.mlpMultiKeyFrames.size() > 2));

      mcptam::MapMakerTiming timingMsg;
      timingMsg.map_num_mkfs = mMap.mlpMultiKeyFrames.size();
      timingMsg.map_num_points = mMap.mlpPoints.size();

      ros::WallTime start = ros::WallTime::now();

      int nAccepted = mBundleAdjuster.BundleAdjustRecent(vOutliers);
      // mdMaxCov = mBundleAdjuster.GetMaxCov();

      timingMsg.elapsed = (ros::WallTime::now() - start).toSec();
      timingMsg.header.stamp = ros::Time::now();
      timingMsg.accepted = nAccepted;
      timingMsg.total = mBundleAdjuster.GetTotalIterations();
      mLocalTimingPub.publish(timingMsg);

      ROS_DEBUG_STREAM("Accepted iterations: " << nAccepted);
      ROS_DEBUG_STREAM("Number of outliers: " << vOutliers.size());
      // ROS_DEBUG_STREAM("Max cov: "<<mdMaxCov);

      if (nAccepted < 0)  // bad
      {
        mnNumConsecutiveFailedBA++;
        if (mnNumConsecutiveFailedBA > MapMakerServerBase::snMaxConsecutiveFailedBA)  // very bad
        {
          ROS_ERROR("MapMaker: Recent BA failed, requesting reset");
          RequestResetInternal();
        }
      }
      else if (nAccepted > 0)
      {
        mnNumConsecutiveFailedBA = 0;
        HandleOutliers(vOutliers);
        PublishMapVisualization();
      }
    }

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    // Are there any newly-made map points which need more measurements from older key-frames?
    if (mBundleAdjuster.ConvergedRecent() && IncomingQueueSize() == 0 && mState == MM_RUNNING)
      ReFindNewlyMade();

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    // Run global bundle adjustment?
    if (mBundleAdjuster.ConvergedRecent() && !mBundleAdjuster.ConvergedFull() && IncomingQueueSize() == 0)
    {
      ROS_INFO("MapMaker: Bundle adjusting all");
      ROS_INFO_STREAM("MapMaker: Number of map points: " << mMap.mlpPoints.size());
      std::vector<std::pair<KeyFrame *, MapPoint *>> vOutliers;

      mBundleAdjuster.UseTukey(mState == MM_RUNNING && mMap.mlpMultiKeyFrames.size() > 2);
      mBundleAdjuster.UseTwoStep((mState == MM_RUNNING && mMap.mlpMultiKeyFrames.size() > 2));

      mcptam::MapMakerTiming timingMsg;
      timingMsg.map_num_mkfs = mMap.mlpMultiKeyFrames.size();
      timingMsg.map_num_points = mMap.mlpPoints.size();

      ros::WallTime start = ros::WallTime::now();

      int nAccepted = mBundleAdjuster.BundleAdjustAll(vOutliers);

      timingMsg.elapsed = (ros::WallTime::now() - start).toSec();
      timingMsg.header.stamp = ros::Time::now();
      timingMsg.accepted = nAccepted;
      timingMsg.total = mBundleAdjuster.GetTotalIterations();
      mGlobalTimingPub.publish(timingMsg);

      mdMaxCov = mBundleAdjuster.GetMaxCov();

      ROS_DEBUG_STREAM("Accepted iterations: " << nAccepted);
      ROS_DEBUG_STREAM("Number of outliers: " << vOutliers.size());
      ROS_DEBUG_STREAM("Max cov: " << mdMaxCov);

      if (nAccepted < 0)  // bad
      {
        mnNumConsecutiveFailedBA++;
        if (mnNumConsecutiveFailedBA > MapMakerServerBase::snMaxConsecutiveFailedBA)  // very bad
        {
          ROS_ERROR("MapMaker: All BA failed, requesting reset");
          RequestResetInternal();
        }
      }
      else if (nAccepted > 0)
      {
        mnNumConsecutiveFailedBA = 0;
        HandleOutliers(vOutliers);
        PublishMapVisualization();

        if (mState == MM_INITIALIZING && (mdMaxCov < MapMakerServerBase::sdInitCovThresh || mbStopInit))
        {
          ROS_INFO_STREAM("INITIALIZING, Max cov " << mdMaxCov << " below threshold "
                                                   << MapMakerServerBase::sdInitCovThresh
                                                   << ", switching to MM_RUNNING");
          mState = MM_RUNNING;
          ClearIncomingQueue();
          mbStopInit = false;
        }
      }
    }

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    // Very low priorty: re-find measurements marked as vOutliers
    if (mBundleAdjuster.ConvergedRecent() && mBundleAdjuster.ConvergedFull() && rand() % 20 == 0 &&
        IncomingQueueSize() == 0 && mState == MM_RUNNING)
      ReFindFromFailureQueue();

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    HandleBadEntities();

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    // Any new key-frames to be added?
    if (IncomingQueueSize() > 0)
      AddMultiKeyFrameFromTopOfQueue();  // Integrate into map data struct, and process

    if (ResetRequested())
    {
      Reset();
      continue;
    }
    if (!mMap.mbGood)
    {
      continue;
    }

    loopRate.sleep();
  }
}

// The tracker entry point for adding a new multikeyframe;
// the tracker thread doesn't want to hang about, so
// just dumps it on the top of the mapmaker's queue to
// be dealt with later, and return.
void MapMaker::AddMultiKeyFrame(MultiKeyFrame *&pMKF_Incoming)
{
  ROS_INFO_STREAM("MapMaker: Incoming MKF, mean depth: " << pMKF_Incoming->mdTotalDepthMean);

  MultiKeyFrame *pMKF = pMKF_Incoming;  // take posession
  pMKF_Incoming = NULL;                 // set original to null, tracker will have to make new MKF

  ProcessIncomingKeyFrames(*pMKF);

  ROS_INFO("MKF contains: ");
  for (KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame &kf = *(it->second);
    ROS_INFO_STREAM(kf.mCamName);
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

  if (mBundleAdjuster.Running())  // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF
                                  // first.
    mBundleAdjuster.RequestAbort();
}

// Entry point for Tracker to initialize the map. Here, the tracker DOES
// hang around and wait for the function to finish, because what would it do
// otherwise with no map?
bool MapMaker::Init(MultiKeyFrame *&pMKF_Incoming, bool bPutPlaneAtOrigin)
{
  MultiKeyFrame *pMKF = pMKF_Incoming;  // take posession
  pMKF_Incoming = NULL;                 // set original to null, tracker will have to make new MKF

  ProcessIncomingKeyFrames(*pMKF);  // from MapMakerClientBase

  for (KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame &kf = *(it->second);
    if (kf.mpSBI)
    {
      delete kf.mpSBI;  // Mapmaker uses a different SBI than the tracker, so will re-gen its own
      kf.mpSBI = NULL;
    }
  }

  return InitFromMultiKeyFrame(pMKF, bPutPlaneAtOrigin);  // from MapMakerServerBase
}

// Add a MultiKeyFrame from the internal queue to the map
void MapMaker::AddMultiKeyFrameFromTopOfQueue()
{
  ROS_INFO("Adding MKF from top of queue");
  boost::mutex::scoped_lock lock(mQueueMutex);

  if (mqpMultiKeyFramesFromTracker.size() == 0)
    return;

  MultiKeyFrame *pMKF = mqpMultiKeyFramesFromTracker.front();
  lock.unlock();  // important!!

  for (KeyFramePtrMap::iterator it = pMKF->mmpKeyFrames.begin(); it != pMKF->mmpKeyFrames.end(); it++)
  {
    KeyFrame &kf = *(it->second);

    // In the interval between the Tracker recording the measurements and then releasing
    // the point locks, and here, points might have been made bad. Check for bad points,
    // remove measurements.
    for (MeasPtrMap::iterator iter = kf.mmpMeasurements.begin(); iter != kf.mmpMeasurements.end();)
    {
      MapPoint &point = *(iter->first);
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

  /*
    // Placing a limit on the number of MKFs in the map
    if(mMap.mlpMultiKeyFrames.size() > 20)
    {
      std::cout<<">>>>>> Marking furthest MKF as bad"<<std::endl;
      MarkFurthestMultiKeyFrameAsBad(*pMKF);
      std::cout<<">>>>>> Marked furthest MKF as bad!"<<std::endl;
    }
  */

  mcptam::MapMakerTiming timingMsg;
  timingMsg.map_num_mkfs = mMap.mlpMultiKeyFrames.size();
  timingMsg.map_num_points = mMap.mlpPoints.size();

  ros::WallTime start = ros::WallTime::now();

  if (mState == MM_RUNNING)
  {
    if (pMKF->NoImages())  // leftovers from Tracker, don't bother adding these
    {
      ROS_INFO("MM_RUNNING: Got an MKF with no image, ignoring");
      pMKF->EraseBackLinksFromPoints();
      delete pMKF;
    }
    else
    {
      ROS_INFO("MM_RUNNING: Trying to add MKF and create points");
      bool bSuccess = AddMultiKeyFrameAndCreatePoints(pMKF);  // from MapMakerServerBase
      if (!bSuccess)
      {
        pMKF->EraseBackLinksFromPoints();
        delete pMKF;
      }
    }
  }
  else  // INITIALIZING
  {
    ROS_INFO("MM_INITIALIZING: Trying to add MKF and mark last as bad");
    AddMultiKeyFrameAndMarkLastDeleted(pMKF, false);
    mMap.MoveDeletedMultiKeyFramesToTrash();
  }

  timingMsg.elapsed = (ros::WallTime::now() - start).toSec();
  timingMsg.header.stamp = ros::Time::now();
  mCreationTimingPub.publish(timingMsg);

  lock.lock();
  mqpMultiKeyFramesFromTracker.pop_front();
  lock.unlock();
}

// Deletes bad points and removes pointers to them from internal queues.
void MapMaker::HandleBadEntities()
{
  ROS_DEBUG_STREAM("HandleBadEntities: Before move to trash we have " << mMap.mlpPoints.size() << " map points, and "
                                                                      << mMap.mlpMultiKeyFrames.size() << " MKFs");
  mMap.MoveBadMultiKeyFramesToTrash();
  mMap.MoveDeletedMultiKeyFramesToTrash();

  MarkDanglersAsBad();
  MarkOutliersAsBad();

  mMap.MoveBadPointsToTrash();
  mMap.MoveDeletedPointsToTrash();
  ROS_DEBUG_STREAM("HandleBadEntities: After move to trash we have " << mMap.mlpPoints.size() << " map points, and "
                                                                     << mMap.mlpMultiKeyFrames.size() << " MKFs");

  EraseBadEntitiesFromQueues();
  mMap.EmptyTrash();
}
