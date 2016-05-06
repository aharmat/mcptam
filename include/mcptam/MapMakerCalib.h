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
 * \file MapMakerCalib.h
 * \brief Declaration of MapMakerCalib class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * MapMakerCalib derives from MapMaker and performs virtually the same function,
 * with two exceptions: the map is initialized from a CalibImageTaylor rather
 *than a
 * MultiKeyFrame, and it offers the capability of optimizing the map with
 *BundleAdjusterCalib
 * bundle adjuster, which optimizes the relative poses between KeyFrames rather
 *than only
 * their position in the world.
 *
 ****************************************************************************************/

#ifndef MCPTAM_MAPMAKERCALIB_H
#define MCPTAM_MAPMAKERCALIB_H

#include <mcptam/MapMaker.h>
#include <mcptam/BundleAdjusterCalib.h>
#include <cvd/thread.h>
#include <TooN/se3.h>
#include <ros/ros.h>
#include <string>
#include <set>

class CalibImageTaylor;

/** @brief Inherits from MapMaker, manages the map during camera pose
 *calibration
 *
 *  Two features that MapMakerCalib adds to MapMaker is the ability to
 *initialize the Map
 *  from a CalibImageTaylor rather than a MultiKeyFrame, and the capability of
 *optimizing the map
 *  with a BundleAdjusterCalib bundle adjuster, which optimizes the relative
 *poses between KeyFrames rather
 *  than only their position in the world */
class MapMakerCalib : public MapMaker
{
public:
  /** @brief Need to call constructor with arguments to ensure valid object
     *  @param map The Map being worked on
     *  @param cameras The camera models
     *  @param bundleAdjuster Some derived class of BundleAdjusterBase that will
   * be used to optimize the map (probably SingleBundle) */
  MapMakerCalib(Map &map, TaylorCameraMap &cameras, BundleAdjusterBase &bundleAdjuster);

  /// Destructor
  virtual ~MapMakerCalib();

  /** @brief Initialize the map
   *  @param calibImage The CalibImageTaylor used to initialize
   *  @param dSquareSize The real-world edge length of the checkerboard squares
   *  @param cameraName The name of the camera that provided the image
   *  @param [in,out] se3TrackerPose The pose of the tracker, is updated after
   * initialization performs bundle adjustment
   *  @return Did initialization succeed?  */
  bool InitFromCalibImage(CalibImageTaylor &calibImage, double dSquareSize, std::string cameraName,
                          TooN::SE3<> &se3TrackerPose);

  /** @brief Cleans up the map and gets it ready for calibration.
   *
   *  Removes MultiKeyFrames from the map that don't contain the first camera.
   *Computes average relative poses from
   *  the remaining MultiKeyFrames/KeyFrames. Updates the base pose of all
   *MultiKeyFrames so that the pose error,
   *  introduced by switching the KeyFrames' relative poses to the average
   *value, is minimized. This makes
   *  bundle adjustment happy.
   *  @return Did initialization succeed? */
  bool CalibInit();

  /** @brief Take one step with the BundleAdjusterCalib adjuster (actually, 10
   * LM steps)
   *  @return Did the step succeed? */
  bool CalibOneStep();

  /** @brief Removes MultiKeyFrames from the Map if it does/doesn't have a
   * certain camera
   *  @param camName The camera name to search for
   *  @param bShouldHave If true, camName needs to be found in MKF or else it is
   * removed. If false, camName needs to be NOT found
   *                     in MKF or else it is removed. */
  void RemoveMultiKeyFrames(std::string camName, bool bShouldHave);

  /// Pause the map making optimization (not the calibration optimization)
  void PauseRun();

  /// Resume the map making optimization (not the calibration optimization)
  void ResumeRun();

  SE3Map mmFinalPoses;      ///< Camera name => pose map of final relative transforms
                            /// (after adjustment)
  double mdSigmaSquared;    ///< Sigma squared value for the robust kernel
  double mdMeanChiSquared;  ///< Sum of errors squared for the optimization,
  /// weighted by information and robust kernel

  enum CalibState
  {
    CALIB_INVALID,
    CALIB_INITIALIZED,
    CALIB_RUNNING
  }
  meCalibState;  ///< State of calibration

protected:
  /** @brief Defined just to override the virtual function in the parent class
   *
   *  This is required because MapMakerCalib needs most of the functionality
   *from MapMaker
   *  but uses its own initialization method. Want to make sure nobody calls
   *this by accident.
   *  Normally we could set the function to deleted, but since its virtual we
   *can't do that
   *  (or at least I haven't figured out how to) */
  virtual bool Init(MultiKeyFrame *&pMKF_Incoming, bool bPutPlaneAtOrigin)
  {
    ROS_FATAL("MapMakerCalib: You shouldn't be here!");
    ROS_BREAK();
    return false;
  }

  /** @brief Finds the average poses of all KFs relative to the first. This
   * means that the relative pose of the first KF
   *  will be the identity pose.
   *  @return A map of camera names => relative poses */
  SE3Map FindAverageRelativePoses();

  /** @brief Removes MKFs from the map whose first KF does not match the first
   * camera in the overall list of cameras.
   *  Also removes points rendered bad from the removal of MKFs */
  void CleanMap();

  std::set<MultiKeyFrame *> mspValidSet;  ///< The set of valid MKFs we're adjusting

  int mnSavedRunState;  ///< Previous map state for pausing and resuming
  /// optimization
  bool mbPaused;  ///< Is the mapmaker thread paused?
};

#endif  // MCPTAM_MAPMAKERCALIB_H
