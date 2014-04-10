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
 * \file CameraCalibrator.h
 * \brief Declaration of CameraCalibrator class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * CameraCalibrator is based off of PTAM's CameraCalibrator but has been modified heavily
 * since the camera model being optimized is different.
 *
 ****************************************************************************************/

#ifndef __CAMERA_CALIBRATOR_H
#define __CAMERA_CALIBRATOR_H

#include <mcptam/VideoSourceSingle.h>
#include <mcptam/GLWindow2.h>
#include <vector>
#include <queue>
#include <ros/ros.h>

class CalibImageTaylor;
class TaylorCamera;

/** @brief The main code for running the camera parameter calibrator
 * 
 *  Contains a run loop which acquires images or optimizes the found checkerboard
 *  corners. Also contains the static function ComputeParamsUpdatePoses, used to 
 *  update the pose of CalibImageTaylors as well as getting intrinsic camera params. */
class CameraCalibrator
{
public:
  /** @brief Default constructor */
  CameraCalibrator();
  
  /** @brief Default destructor */
  ~CameraCalibrator();
  
  /** @brief Blocking function that loops indefinitiely
   * 
   * Either tries to find a checkerboard in the current image, or takes one optimization step per loop iteration */
  void Run();
  
protected:

  /** @brief Delete all acquired data, reset camera model parameters to defaults */
  void Reset();
  
  /** @brief Finds the best image poses, camera polynomials and center of projection using linear methods
   *  
   *  Starting at the center of the image, finds the best center or projection by exhaustive search, testing several 
   *  points around the current best to find the next best step. According to Scaramuzza's thesis, the sum
   *  of projection errors is minimum at the best center of projection and is uniformly decreasing towards
   *  that point. */
  void InitOptimization();
  
  /** @brief Given a starting position and an area to test, finds the best center of projection
   *  
   *  Tests a certain number points in the x and y directions from the current center at v2StartCenter. 
   *  @param v2StartCenter The current estimate for the center of projection
   *  @param v2Spread The spread on either side of the center to test
   *  @param irNumPoints How many points in the x and y directions to test (should be odd, will be forced to next largest odd if even)
   *  @param [out] v2BestPos The best center of projection position found
   *  @param [out] dMinError The sum of projection errors at the best position */
  void FindBestCenter(TooN::Vector<2> v2StartCenter, TooN::Vector<2> v2Spread, CVD::ImageRef irNumPoints, TooN::Vector<2>& v2BestPos, TooN::Vector<4>& v4BestParams, double& dMinError);

  /** @brief Compute the z component of the translation vector of the CalibImateTaylor's poses, and the polynomial
   *         coefficients of the camera model.
   *  @param vpCalibImgs The calibration images, their poses WILL be modified by this function
   *  @param v2Center The center of projection to use
   *  @return The new camera polynomial coefficients */
  TooN::Vector<4> ComputeParamsUpdatePoses(std::vector<CalibImageTaylor*> vpCalibImgs, TooN::Vector<2> v2Center);

  /** @brief Takes one step of Levenberg-Marquardt optimization for the whole system (camera poses and parameters)
   * 
   *  This is a textbook implementation of LM optimization according to Hartley & Zisserman MVG book. No attempts have been made to save on any
   *  matrix multiplications.
   *  @return Was the step successful? */
  bool OptimizeOneStepLM();
  
  /// Update the LM lambda after a good optimization step
  void ModifyLambda_GoodStep()
  {
    mdLambda /= mdLambdaFactor;
  }

  /// Update the LM lambda after a bad optimization step
  void ModifyLambda_BadStep()
  {
    mdLambda *= mdLambdaFactor;
  }
  
  /** @brief Creates a new GLWindow2 object with the dimensions of the video source
   *  @param windowName The name of the window
   *  @return Pointer to new window */
  GLWindow2* InitWindow(std::string windowName)
  {
    return new GLWindow2(mVideoSource.GetSize(), windowName);
  }
  
  /// Used to save callback data in GUICommandCallBack
  struct Command 
  {
    std::string command; 
    std::string params; 
  };
  
  /** @brief This can be used with GUI.RegisterCommand to capture user input 
   * 
   *  Simply creates an instance of Command and pushes it to mqCommands
   *  @param ptr Pointer to an instance of the class that will save the captured input in mqCommands
   *  @param command The command that was received
   *  @param params Additional parameters that came with the command */
  void GUICommandHandler(std::string command, std::string params);
  
  /** @brief Deals with user interface commands
   *  @param command The saved command
   *  @param params The saved command parameters */
  static void GUICommandCallBack(void* ptr, std::string command, std::string params);

  VideoSourceSingle mVideoSource;  ///< The video source (only one camera at a time)
  GLWindow2* mpGLWindow;           ///< The OpenGL window
  TaylorCamera* mpCamera;          ///< The camera model
  CVD::ImageRef mirSize;           ///< Size of the images
  std::string mCamName;            ///< Name of the camera
  bool mbDone;                     ///< Run loop finished?

  std::vector<CalibImageTaylor*> mvpCalibImgs;   ///< Vector of calibration images
  
  std::string mSetInfoTopic;               ///< Topic where camera's calibrated parameters can be updated
  bool mbGrabNextFrame;                    ///< Should the next image be saved?
  bool mbOptimizing;                       ///< Is the system performing LM optimization?
  int mnImageToShow;                           ///< Which saved image should be displayed in GUI
  double mdMeanPixelError;                 ///< Sum of errors divided by number of calibration images
  
  bool mbInit;                    ///< Has optimization been initialized?
  
  double mdLambda;                ///< LM optimization parameter
  double mdLambdaFactor;          ///< LM optimization parameter

  ros::NodeHandle mNodeHandle;       ///< ROS global node handle
  ros::NodeHandle mNodeHandlePriv;   ///< ROS private node handle
  
  TooN::Vector<9> mv9DefaultParams;  ///< The default values that will be used to initialize the camera params
  
  std::queue<Command> mqCommands;   ///< Queued commands received by GUICommandCallBack
  
};

#endif

