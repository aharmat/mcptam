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
 * \file CalibImageTaylor.h
 * \brief Declaration of CalibImageTaylor class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * CalibImageTaylor is based heavily off the CalibImage class in the original PTAM. The
 * functions related to finding a checkerboard have been left alone, but since this
 * class is now being used to calibrate the TaylorCamera instead of an ATANCamera, the 
 * functions dealing with calibration are completely new.
 *
 ****************************************************************************************/


#ifndef __CALIB_IMAGE_TAYLOR_H
#define __CALIB_IMAGE_TAYLOR_H

#include <mcptam/CalibGridCorner.h>
#include <vector>
#include <cvd/image.h>
#include <TooN/se3.h>

class TaylorCamera;
class GLWindow2;

/** @brief A helper class used by CameraCalibrator to find a checkerboard in an image and compute
 *  calibration parameters.
 * 
 *  This class handles all the image-level calibration functions, including finding checkerboard
 *  corners, computing the pose of the image based on a known calibration pattern, getting 
 *  projection errors, and generating matrices/vectors that are aggregated by CameraCalibrator to
 *  solve a larger matrix. The calibration algorithms in this class are based on Davide Scaramuzza's
 *  PhD 2008 thesis "OMNIDIRECTIONAL VISION: FROM CALIBRATION TO ROBOT MOTION ESTIMATION" (the same
 *  thesis that TaylorCamera is based off of). Also, Scaramuzza's Matlab toolbox OCamCalib was of
 *  great help. */
class CalibImageTaylor
{
public:

  /** @brief Need to call constructor with arguments to ensure a valid object
   *  @param irDrawOffset The screen offset where the image associated with this CalibImageTaylor will be drawn 
   *  @param pWindow Pointer to the GL window */
  CalibImageTaylor(CVD::ImageRef irDrawOffset, GLWindow2* pWindow);
  
  /** @brief Finds checkerboard in image, optionally orders the found corners according to the size/orientation of the checkerboard
   *  @param im The image to process
   *  @param irPatternSize If specified, will check the found corners to see if they match the pattern size, and will reorder the points so that those with identical
   *                       grid coordinates in different images correspond to the same physical point on the checkerboard. This is necessary for pose calibration. If
   *                       not specified, will just find all available corners and return true, used for camera parameter calibration where it is not necessary to find
   *                       all checkerboard corners. */
  bool MakeFromImage(CVD::Image<CVD::byte> &im, CVD::ImageRef irPatternSize = CVD::ImageRef());
  
  /** @brief Draw the current grid (as defined by the current found checkerboard corners), projected into a given camera at the current pose of the CalibImageTaylor
   * 
   *  If the camera model is calibrated and the current pose is correct, then this grid would essentially be exactly the same as that drawn by DrawImageGrid. However, during
   *  calibration we don't yet know the camera parameters or the current pose, so the grid drawn by this function will be skewed and/or out of place. This is expected. As the 
   *  calibration process converges the fit of the drawn grid should also converge to the underlying checkerboard
   *  @param camera The camera model used for projection
   *  @param bDrawErrors Should I draw lines indicating the point projection errors? */
  void Draw3DGrid(TaylorCamera &camera, bool bDrawErrors);
  
  /** @brief Calculates an initial guess for the pose of the image, based on the given projection center. 
   * 
   *  The z component of the translation will be set to 1, because to calculate that properly we need to consider the set of all CalibImageTaylors captured.
   *  This is done later by CameraCalibrator's ComputeParamsUpdatePoses function. The algorithm used in this function is that of Scaramuzza thesis section 3.2.1
   *  "Estimation of the extrinsic parameters"
   *  @param v2Center The projection center */
  void GuessInitialPose(TooN::Vector<2> v2Center);
  
  /** @brief Called by CameraCalibrator's ComputeParamsUpdatePoses function to build a larger matrix.
   * 
   *  The large matrix is solved to find the intrinsic parameters of the camera as well as finding the z component of translation for each CalibImageTaylor.
   *  The algorithm used in this function is that of Scaramuzza thesis section 3.2.2 "Estimation of the intrinsic parameters". Note that nImgNum and 
   *  nTotalNum are in c++ array format, so if nTotalNum is 2, nImgNum is either 0 or 1 but definitely not 2. The internal pose mse3CamFromWorld should be 
   *  up to date before calling this!
   *  @param v2Center The image center of projection
   *  @param nDegree The degree of the polynomial that will be build into the matrix entry
   *  @param nImgNum The number of the current image out of all the images collected, controls where the nonzero entries of the matrix go
   *  @param nTotalNum The total number of all the images collected
   *  @return A Matrix and a Vector as a pair, these are sub-components of the larger matrix and vector being built by the calling code */
  std::pair< TooN::Matrix<>, TooN::Vector<> > BuildIntrinsicMatrixEntries(TooN::Vector<2> v2Center, int nDegree, int nImgNum, int nTotalNum);
  
  /// Helper structure to hold a projection's error and jacobian matrices
  struct ErrorAndJacobians
  {
    TooN::Vector<2> v2Error;        ///< The projection error
    TooN::Matrix<2,6> m26PoseJac;   ///< The derivative of the projection relative to the camera pose
    TooN::Matrix<2,9> m2NCameraJac; ///< The derivative of the projection relative to the camera internal parameters
  };

  /** @brief Projects all the found corners using the given camera model and the currently set pose, returns errors and jacobians
   *  @param camera The camera model to use
   *  @return A vector of projection errors and jacobian matrices */
  std::vector<ErrorAndJacobians> Project(TaylorCamera &camera);
  
  /** @brief Project the found corners using the given camera model, but return only only the projection errors without any jacobians
   *  
   *  Jacobians are only needed at the beginning of an optimization step, but after when we are checking to see if the step was good we 
   *  only need to get the errors, so call this funtion to save some time.
   *  @param camera The camera model to use
   *  @param bNewTransform Should I use mse3CamFromWorld or mse3CamFromWorldNew to compute the projection errors?
   *  @return A vector of projection errors */
  std::vector< TooN::Vector<2> > ProjectGetOnlyErrors(TaylorCamera &camera, bool bNewTransform = false);

  TooN::SE3<> mse3CamFromWorld;        ///< The current pose of the image
  TooN::SE3<> mse3CamFromWorldNew;     ///< The new pose of the image, used for testing optimization step success
  CVD::Image<CVD::byte> mImage;        ///< The image being worked on
  std::vector<CalibGridCorner> mvGridCorners;  ///< Vector of checkerboard corner structs
  
  // Static members
  static int snCornerPatchSize;       ///< Size of the patch to find a corner within
  static double sdBlurSigma;          ///< Sigma for the Gaussian blur pass
  static int snMeanGate;              ///< Value above and below mean intensity to label black and white
  static int snMinCornersForGrabbedImage;   ///< Minimum number of corners in the image
  static double sdExpandByStepMaxDistFrac;  
  
protected:
  
  // Functions called by MakeFromImage
  /// Draw a grid over the points that have been found
  void DrawImageGrid();
  
  /** @brief Try to find a new checkerboard corner by stepping from an already-found corner, based on the given direction
   *  @param nSrc The index of the corner to step from
   *  @param nDirn The direction to try stepping (0-3, 0 = to the right, 1 = down, 2 = to the left, 3 = up)
   *  @return Was a new checkerboard corner found? */
  bool ExpandByAngle(int nSrc, int nDirn);
  
  /// Finds the index of the current checkerboard corner with the best chance of finding a neighbor
  /** @return The index of the best corner */
  int NextToExpand();
  
  /** @brief Find a new checkerboard corner from an already-found corner, by choosing the best expansion direction
   *  @param n The index of the corner to step from */
  void ExpandByStep(int n);
  
  /** @brief Helper function to convert the direction 0-3 to an ImageRef
   *  @param nDirn The direction to convert
   *  @return The resulting ImageRef */
  CVD::ImageRef IR_from_dirn(int nDirn);
  
  // Calibration functions
  /** @brief Given the vector H from Scaramuzza thesis eq 3.7, solves for one of two entries of the 3rd column of the rotation
   *         matrix of the image pose.
   *  @param v6H The H vector
   *  @param nRNum Either 1 or 2, determines if function solves for r31 or r32 entry of matrix
   *  @return The solved matrix entry */
  double SolveForR3(TooN::Vector<6> v6H, int nRNum);
  
  /** @brief Converts a 3x3 matrix representation of the pose to an SE3
   *  
   *  In most of the internal calibration algorithms, the pose is represented by a 3x3 matrix where the first two columns
   *  are the first two columns of the rotation matrix, and the last column is the translation vector. This is possible because
   *  we assume a planar checkerboard, so the z=0 coordinate cancels out one of the rotation matrix columns during most calculations.
   *  @param R The 3x3 matrix to convert
   *  @return The full SE3 pose */
  TooN::SE3<> M3ToSE3(TooN::Matrix<3> R);
  
  /** @brief Decide which matrix from the inputs contains the correct rotation.
   * 
   *  Does this by finding the rotation matrix that produces a positive z value when solving for a second order camera polynomial
   *  as well as the z translation. This is similar to the procedure used in Scaramuzza's thesis, where he looks at the sign of the 
   *  second order coefficient (Fig. 2.10 in his thesis). However, for cameras with a very small amount of radial distortion, this
   *  coefficient is very close to zero, and can turn up positive or negative depending on measurement noise. Therefore, we look at
   *  the z value instead, which is far more robust.
   * 
   *  @param vRs The vector of rotation matrices to try
   *  @param v2Center The center of projection
   *  @return The index of the correct rotation matrix */
  int FindCorrectRotation(std::vector< TooN::Matrix<3> > vRs, TooN::Vector<2> v2Center);
  
  std::vector<CVD::ImageRef> mvCorners;        ///< Vector of raw checkerboard corner locations
  
  CVD::ImageRef mirDrawOffset;                 ///< The drawing offset
  GLWindow2* mpGLWindow;                       ///< Pointer to the OpenGL window
  
  //friend class CameraCalibrator;
  //friend class CalibratorCommon;
  //friend class MapMakerCalib;
  //friend class TrackerCalib;
 
};


#endif

