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
 * \file TrackerCalib.h
 * \brief Declaration of TrackerCalib class
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * TrackerCalib derives from Tracker and performs virtually the same function, except
 * it has a checkerboard-finding stage used to initialize the Map with MapMakerCalib.
 *
 ****************************************************************************************/

#ifndef __CALIBRATOR_TRACKER_H
#define __CALIBRATOR_TRACKER_H

#include <mcptam/Tracker.h>
#include <mcptam/Types.h>

class CalibImageTaylor;
class KeyFrame;


/** @brief Inherits from Tracker, allows the detection of a checkerboard in an initialization phase
 *  
 *  Unlike Tracker, TrackerCalib tracks only one camera since during calibration the relative poses of the
 *  cameras are not known. Also, it has an initialization phase during which it attempts to find a checkerboard
 *  using CalibImageTaylor. TrackerCalib doesn't add KeyFrames or MultiKeyFrames directly to the map,
 *  but rather signals its intention to add by setting the mbNeedToDrop flag.
 * 
 *  The inheritance from Tracker is mostly done to prevent a lot of code reuse, since TrackerCalib is more
 *  a different type of Tracker than a child of it. Therefore, several functions and members of Tracker are not
 *  used. This is not great C++ programming but let's overlook that.
 *  */
class TrackerCalib : public Tracker
{
public:

  /** @brief Need to call constructor with arguments to ensure valid object
     *  @param map The Map being worked on
     *  @param cameras The camera models. Needs all models for patch finding.
     *  @param cameraName The name of the camera whose pose is being estimated
     *  @param irOffset Drawing offset for camera
     *  @param irPatternSize Size of the checkerboard pattern we are looking for
     *  @param dSquareSize The real world edge length of a checkerboard square
     *  @param pWindow Pointer to the OpenGL window*/
  TrackerCalib(Map &map, RelocaliserFabMap &reloc, MapMakerClientBase &mapMaker, TaylorCameraMap cameras, std::string cameraName, CVD::ImageRef irOffset, CVD::ImageRef irPatternSize, double dSquareSize, GLWindow2* pWindow);
  
  /** @brief The main working part of TrackerCalib, call this every frame
   *  @param imFrame An image containing the latest acquisition from the camera we've been assigned
   *  @param bDraw Should I draw the images and detected corner features? 
   *  @param bFindCheckerboard Is it my turn to try to find a checkerboard? */
  void TrackFrame(CVD::Image<CVD::byte>& imFrame, ros::Time timestamp, bool bDraw, bool bFindCheckerboard); 
  
  /// Deleted to prevent base class's TrackFrame usage
  void TrackFrame(ImageBWMap& imFrames, ros::Time timestamp, bool bDraw) = delete;  
  
  /// Call parent's Reset and reset member variables
  void Reset(bool bResetMap);
  
  bool mbNeedToDrop;  ///< Should my current KeyFrame be added to the Map?
  
  enum {CHECKERBOARD_INACTIVE, CHECKERBOARD_FIRST_STAGE, CHECKERBOARD_SECOND_STAGE, CHECKERBOARD_RUNNING} meCheckerboardStage;  ///< Active during checkerboard finding
  
protected:

  /// Set the need-to-drop flag to true
  void SignalNewKeyFrame();          
  
  /// Set the need-to-drop flag to false and regenerate our current MultiKeyFrame because it has been taken by the Map
  void MarkKeyFrameAdded(KeyFrame& kfSrc);
  
  /// Set the need-to-drop flag to false, used when map is first made from a CalibImage so no need to regenerate MKF
  void MarkKeyFrameAdded();
  
  /// Draw a border around the camera image indicating status (ie running, initializing, lost)
  void DrawStatusBorder();
  
  /// Draw a border of given thickness and color
  void DrawBorder(CVD::ImageRef irBorder, int nThickness, const TooN::Vector<-1>& vColor);

  /// Initialize a default pose map with a given camera name, used to call Tracker constructor
  SE3Map GetInitPoseMap(std::string name)
  {
    SE3Map m;
    m.insert(std::make_pair(name, TooN::SE3<>()));
    return m;
  }
  
  /// Initialize a default drawing offset map with a given camera name and offset, used to call Tracker constructor
  ImageRefMap GetInitOffsetMap(std::string name, CVD::ImageRef irOffset)
  {
    ImageRefMap m;
    m.insert(std::make_pair(name, irOffset));
    return m;
  }
  
  /// Finish computing the internal pose of the calibration image using the known parameters of our camera
  void FinishCalibImagePose();
  
  /// Optimize the pose computed by the calibration image using bundle adjustment
  void OptimizeCalibImagePose();

  CVD::ImageRef mirPatternSize;   ///< The number or rows and columns in the checkerboard we're finding
  double mdSquareSize;            ///< Real-world edge length of the checkerboard squares

  CalibImageTaylor* mpCalibImage;   ///< The calibration image used for checkerboard finding
  KeyFrame* mpCurrentKF;            ///< Pointer to the currently used KeyFrame in the parent's MultiKeyFrame (ie the one with the same camera name as us)
  
  std::string mCamName;      ///< The name of the camera we're processing
  
  TooN::Vector<4> v4FirstStageColor;   ///< GUI border color for when finding checkerboard (first stage)
  TooN::Vector<4> v4SecondStageColor;   ///< GUI border color for when finding checkerboard (second stage)
  TooN::Vector<4> v4RunningColor;   ///< GUI border color for when system is tracking/running
  TooN::Vector<4> v4LostColor;      ///< GUI border color for when lost
  TooN::Vector<4> v4InactiveColor;  ///< GUI border color for when the tracker is inactive
  
  // PoseCalibrator is pretty intimately tied to TrackerCalib, it manipulates a lot of its internal data directly. Ideally, TrackerCalib
  // would be rewritten as its own class, separate from Tracker, and its interface to PoseCalibrator would be more well defined.
  friend class PoseCalibrator; 
  
};

#endif

