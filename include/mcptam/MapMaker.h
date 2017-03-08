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
 * \file MapMaker.h
 * \brief Declaration of MapMaker class
 *
 * Large parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * The MapMaker of PTAM has been broken up into several different pieces, in
 *order to allow
 * the construction of both a standalone MapMaker as well as a client/server
 *MapMaker with
 * a minimum amount of code duplication that would promote confusion and
 *inconsistency. The
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
 * MapMaker is the standalone map maker that handles both interfacing with the
 * Tracker and optimizing the map. As it doesn't have a networking component, it
 * is pretty simple compared to the client/server versions.
 *
 ****************************************************************************************/

#ifndef MCPTAM_MAPMAKER_H
#define MCPTAM_MAPMAKER_H

#include <mcptam/MapMakerClientBase.h>
#include <mcptam/MapMakerServerBase.h>
#include <cvd/thread.h>
#include <string>

/** @brief Implements standalone map maker by inheriting from both client-side
 *and server-side code
 *
 * By inheriting from CVD::Thread, we gain access to some useful functions that
 *allow this class
 * to run in its own thread. */
class MapMaker : public MapMakerClientBase, public MapMakerServerBase, protected CVD::Thread
{
public:
  /** @brief Need to call constructor with arguments to ensure valid object
   *  @param map The Map being worked on
   *  @param cameras The camera models
   *  @param bundleAdjuster Some derived class of BundleAdjusterBase that will
   * be used to optimize the map */
  MapMaker(Map &map, TaylorCameraMap &cameras, BundleAdjusterBase &bundleAdjuster);

  /// Destructor
  virtual ~MapMaker();

  /// Overridden from CVD::Thread, this executes in its own thread
  virtual void run();

  /// Overridden from MapMakerClientBase, entry point for Tracker to add a
  /// MultiKeyFrame
  /** @param pMKF_Incoming Pointer to the MultiKeyFrame being added */
  virtual void AddMultiKeyFrame(MultiKeyFrame *&pMKF_Incoming);

  /// Overridden from MapMakerClientBase, entry point for Tracker to initialize
  /// the map with one MultiKeyFrame containing multiple KeyFrames
  /** @param pMKF_Incoming Pointer to the MultiKeyFrame being used to initialize
   * map
   *  @param bPutPlaneAtOrigin If false, origin will be at location of first
   * MKF, otherwise plane fitted to first set of points */
  virtual bool Init(MultiKeyFrame *&pMKF_Incoming, bool bPutPlaneAtOrigin);

  /// Overridden from MapMakerClientBase, requests an abort from the bundle
  /// adjuster and sets flags that signal a reset is waiting
  virtual void RequestReset();

  /// Rescale the map by the given scale factor
  void RequestRescaling(double dScale);

  /// Request the map parameters be written to the given file name
  void RequestFileDump(std::string filename);

  /// Request that the initialization phase is completed early
  void RequestStopInit();

protected:
  /// Resets the parent classes
  void Reset();

  /// Overridden from MapMakerClientBase, called to add a MultiKeyFrame from the
  /// internal queue to the map
  virtual void AddMultiKeyFrameFromTopOfQueue();

  /// Overridden from MapMakerServerBase, deletes bad points and removes
  /// pointers to them from internal queues
  virtual void HandleBadEntities();

  /// Overridden from MapMakerServerBase, returns the size of the tracker queue
  /// as that is the data source for us.
  /// In MapMakerServer, the data source would be the network manager.
  virtual int IncomingQueueSize()
  {
    return TrackerQueueSize();  // From MapMakerClient
  }

  // debug
  bool mbRescale;  ///< Rescaling is requested
  double mdScale;  ///< Amount to scale the map by

  // debug
  bool mbFileDump;            ///< Dump map params to file requested
  std::string mDumpFileName;  ///< File name to write map params to

  // testing
  bool mbStopInit;  ///< End of initialization phase requested

  ros::Publisher mCreationTimingPub;
  ros::Publisher mLocalTimingPub;
  ros::Publisher mGlobalTimingPub;

  unsigned int seed = 1;  ///< Seed for threadsafe rand_r
};

#endif  // MCPTAM_MAPMAKER_H
