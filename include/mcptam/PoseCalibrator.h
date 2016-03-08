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
 * \file PoseCalibrator.h
 * \brief Declaration of PoseCalibrator class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * PoseCalibrator derives from SystemBase and implements the main running code
 *of the
 * pose calibrator. It gets new images, feeds them to the CalibratorTrackers,
 *and
 * assembles new MultiKeyFrames from them when one wants to add to the Map.
 *
 ****************************************************************************************/

#ifndef MCPTAM_POSECALIBRATOR_H
#define MCPTAM_POSECALIBRATOR_H

#include <mcptam/SystemBase.h>
#include <mcptam/Types.h>
#include <mcptam/Reset.h>
#include <cvd/image.h>
#include <queue>
#include <deque>
#include <vector>
#include <map>
#include <string>

class MapMakerCalib;
class BundleAdjusterSingle;
class TrackerCalib;
class MultiKeyFrame;
class KeyFrameViewer;

typedef std::map<std::string, TrackerCalib *> TrackerCalibPtrMap;

/** @brief Inherits from SystemBase, implements the main running code of the
 * pose calibrator */
class PoseCalibrator : public SystemBase
{
public:
  /// Creates objects, sets up GUI
  PoseCalibrator();

  /// Destructor
  ~PoseCalibrator();

  /** @brief Blocking function that loops indefinitiely
   *
   *  Calls either Track() or Optimize() depending on the user's choices */
  void Run();

private:
  /** @brief Take the KeyFrame from a TrackerCalib and put it into a
   *MultiKeyFrame
   *
   *  The TrackerCalib's current MultiKeyFrame will be deleted after this, so it
   *should
   *  regenerate it. There was already a mechanism in place to regenerate
   *MultiKeyFrames but
   *  not KeyFrames, which is why I decided to just destroy the whole
   *MultiKeyFrame instead of
   *  simply taking the KeyFrame.
   *
   *  Note that this function needs to be called in the appropriate order, ie
   *first call should
   *  be with the TrackerCalib corresponding to the first KeyFrame, etc. This is
   *because
   *  the first KeyFrame will set up the MultiKeyFrame's pose, and the
   *subsequent KeyFrames
   *  depend on this being set.
   *  @param pMKF Pointer to the MultiKeyFrame that will be the new owner of the
   *KeyFrame
   *  @param pCT Pointer to the TrackerCalib that will lose its KeyFrame */
  void TransferKeyFrame(MultiKeyFrame *pMKF, TrackerCalib *pCT);

  /** @brief Perform all tracking operations.
   *
   *  Gets new images, feeds them to the CalibratorTrackers, decides which one
   *should be
   *  trying to find the checkerboard, and assembles new MultiKeyFrames from the
   *  CalibratorTrackers if one wants to add to the Map.
   *  @return A message for the user */
  std::string Track();

  /** @brief Perform optimization of the relative poses of KeyFrames
   *
   *  @return A message for the user */
  std::string Optimize();

  /** @brief Publishes TF transforms for all the cameras relative to the frame
   * vision_world */
  void PublishPoses();

  /** @brief Deals with user interface commands
   *  @param command The saved command
   *  @param params The saved command parameters */
  void GUICommandHandler(std::string command, std::string params);

  /// Reset the individual CalibratorTrackers
  bool ResetSystemCallback(mcptam::Reset::Request &request, mcptam::Reset::Response &response);

  ImageBWMap mmFramesBW;  ///< %Map of greyscale CVD::Images

  BundleAdjusterSingle *mpBundleAdjuster;  ///< Pointer to the bundle adjuster
  /// used for map optimization during
  /// scene browsing
  MapMakerCalib *mpMapMaker;         ///< Pointer to the MapMakerCalib
  KeyFrameViewer *mpKeyFrameViewer;  ///< Pointer to the KeyFrameViewer
  TrackerCalibPtrMap mmTrackers;     ///< %Map of CalibratorTrackers

  SE3Map mmFinalPoses;  ///< %Map of camera names to relative poses

  bool mbDone;                   ///< Should I quit run loop?
  CVD::ImageRef mirPatternSize;  ///< The size of the checkerboard
  double mdSquareSize;           ///< Real-world edge length of checkerboard squares
  ros::Time mtLastMKF;           ///< When was the last MultiKeyFrame added to the Map?
  unsigned int mnLastNumInit;    ///< How many TrackerCalibs were initialized last time
  bool mbOptimizing;             ///< Am I optimizing?

  std::vector<TrackerCalib *> mvInitialized;  ///< Which CalibratorTrackers have been initialized

  ros::ServiceServer mResetSystemServer;  ///< Service to allow system reset,
  /// needed for map maker
};

#endif  // MCPTAM_POSECALIBRATOR_H
