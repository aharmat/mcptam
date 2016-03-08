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
 * \file TaylorCamera.h
 * \brief Declaration of TaylorCamera class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * This class is based loosely on the ATANCamera class that came with PTAM,
 *mostly
 * the structure of the interface and the workflow inside the various functions.
 *However,
 * the math is completely new and is based on the Taylor camera model desribed
 *in
 * Davide Scaramuzza's PhD thesis "OMNIDIRECTIONAL VISION: FROM CALIBRATION TO
 *ROBOT
 * MOTION ESTIMATION", 2008.
 *
 * I have stripped out all UFB projection functions, which were used to render
 *into
 * OpenGL textures. This was used only by the augmented reality "eye-game" in
 *PTAM, which
 * is not included in mcptam.
 *
 * The following are some relevant comments from ATANCamera.h that I'm
 *reproducing here:
 *
 *   BEWARE: This camera model caches intermediate results in member variables
 *   Some functions therefore depend on being called in order: i.e.
 *   GetProjectionDerivs() uses data stored from the last Project() or
 *UnProject()
 *   THIS MEANS YOU MUST BE CAREFUL WITH MULTIPLE THREADS
 *   Best bet is to give each thread its own version of the camera!
 *
 *   Pixel conventions are as follows:
 *   For Project() and Unproject(),
 *   round pixel values - i.e. (0.0, 0.0) - refer to pixel centers
 *   I.e. the top left pixel in the image covers is centered on (0,0)
 *   and covers the area (-.5, -.5) to (.5, .5)
 *
 *   Be aware that this is not the same as what opengl uses but makes sense
 *   for acessing pixels using ImageRef, especially ir_rounded.
 *
 ****************************************************************************************/

#ifndef MCPTAM_TAYLORCAMERA_H
#define MCPTAM_TAYLORCAMERA_H

#include <TooN/TooN.h>
#include <cvd/vector_image_ref.h>
#include <ros/assert.h>

#define MAX_INV_DEGREE 30  ///< The maximum degree of the inverse of the 4th order polynomial defining
/// the camera model

class CameraCalibrator;
class CalibImage;

/** @brief Contains all Taylor camera related variables and operations
 *
 *  This class implements the Taylor camera model described in Davide
 *Scaramuzza's PhD thesis
 *  "OMNIDIRECTIONAL VISION: FROM CALIBRATION TO ROBOT MOTION ESTIMATION", 2008.
 *Please see
 *  the thesis for details on the projection model as the documentation shown
 *here is limited.
 *  Also, Scaramuzza's Matlab toolbox OCamCalib was of great help.
 *
 *  The Taylor camera model allows the usage of cameras with FOVs greater than
 *180 degrees. This
 *  class offers an interface very similar to the ATANCamera's interface in the
 *original PTAM, with
 *  the exception that GVars are no longer used anywhere.
 */
class TaylorCamera
{
public:
  /// Default constructor just to allow compilation with map container, it
  /// should never actually be called, error otherwise
  TaylorCamera()
  {
// testing
#ifndef PARALLELIZED
    ROS_BREAK();
#endif
  }

  /** @brief Constructor for live camera model, i.e. NOT for calibration! It
   *expects a calibrated parameter vector.
   *
   *  @param v9Params The Taylor model parameters, see detail in mv9CameraParams
   *variable
   *  @param irCalibSize The image size that was used for camera calibration
   *  @param irFullScaleSize The size, AT THE ORIGINAL PIXEL LEVEL, that the
   *currently used camera image occupies.
   *         This is a product of the actual size of the current image and any
   *binning that is being applied.
   *  @param irImageSize The actual size of the currently used camera image  */
  TaylorCamera(TooN::Vector<9> v9Params, CVD::ImageRef irCalibSize, CVD::ImageRef irFullScaleSize,
               CVD::ImageRef irImageSize);

  /** @brief Constructor for calibrating the camera model, i.e. NOT for live
   *applications! It expects no parameter vector
   *         since we'll be finding it through calibration.
   *
   *  @param irCalibSize The image size that was used for camera calibration
   *  @param irFullScaleSize The size, AT THE ORIGINAL PIXEL LEVEL, that the
   *currently used camera image occupies.
   *         This is a product of the actual size of the current image and any
   *binning that is being applied.
   *  @param irImageSize The actual size of the currently used camera image  */
  TaylorCamera(CVD::ImageRef irCalibSize, CVD::ImageRef irFullScaleSize, CVD::ImageRef irImageSize);

  // Functions for getting/setting image size
  /// Set the current image size. Useful for dealing with subsampled images ie
  /// SmallBlurryImage.
  /** @param irImageSize The new image size */
  void SetImageSize(CVD::ImageRef irImageSize);

  /// Get the current image size
  /** @return The current image size */
  inline CVD::ImageRef GetImageSize()
  {
    return CVD::ir(mv2ImageSize);
  }

  /// Get the full scale size of the camera
  /** @return The full scale size (no binning) */
  inline CVD::ImageRef GetFullScaleSize()
  {
    return CVD::ir(mv2FullScaleSize);
  }

  // Various projection functions
  /// Projects from camera frame to pixel coordinates, with distortion
  /** @param v3CamFrame The point to project given in the camera frame in
   * Cartesian coordinates
   *  @return The pixel coordinates of the projection */
  TooN::Vector<2> Project(const TooN::Vector<3> &v3CamFrame);

  /// Inverse projection operation, projects to the unit sphere
  /** @param v2ImFrame The pixel coordinate to un-project
   *  @return The resulting unit-length vector in Cartesian coordinates */
  TooN::Vector<3> UnProject(const TooN::Vector<2> &v2ImFrame);

  /** @brief The Jacobian of the projection, given as derivative of pixel
   *position W.R.T. position on unit sphere.
   *
   * In this function, position on the unit sphere is encoded in spherical
   *coordinates. Therefore, one would think that
   * the resulting Jacobian matrix should be 2x3 rather than 2x2. However,
   *motion along the radial direction has no effect
   * on the pixel location of a projection, so the last column in a 2x3 matrix
   *would always be the zero vector, so it is dropped.
   * @return The 2x2 Jacobian */
  TooN::Matrix<2> GetProjectionDerivs();

  // Utility functions
  /// Getter method for invalid flag
  /** @return Boolean indicating validity of last projection */
  inline bool Invalid()
  {
    return mbInvalid;
  }

  /// Getter method for largest image radius value
  /** @return The largest radius found in the image, computed as the largest
   *  distance between the center of projection and any of the corners */
  inline double LargestRadiusInImage()
  {
    return mdLargestRadius;
  }

  /// Get the minimum ray incidence angle allowed for valid projection
  inline double MinTheta()
  {
    return mdMinTheta;
  }

  /// Getter method for the angle spanned by one pixel
  /** @return The angle spanned by one pixel at the image center, used for
   * heuristics in epipolar search */
  inline double OnePixelAngle()
  {
    return mdOnePixelAngle;
  }

  /// Feedback for CameraCalibrator
  /** @return The ratio of vertical to horizontal edge length of a unit pixel
   * that has been transformed by the camera's affine matrix */
  double PixelAspectRatio()
  {
    TooN::Vector<2> v2Skewed = mm2Affine * TooN::makeVector(1, 1);
    return v2Skewed[1] / v2Skewed[0];
  }

  /** @return The center of projection */
  TooN::Vector<2> GetCenter()
  {
    return mv2Center;
  }

  // Static functions
  /// Get the derivatives of a cartesian 3-vector projected onto the unit
  /// sphere, in spherical coordinates
  /** @param v3Cam The position of the point in Cartesian coordinates
   *  @param [out] v3_dTheta The derivative of the theta component of the
   * spherical coordinate with respect to the position of the point
   *  @param [out] v3_dPhi The derivative of the phi component of the spherical
   * coordinate with respect to the position of the point */
  static void GetCamSphereDeriv(const TooN::Vector<3> &v3Cam, TooN::Vector<3> &v3_dTheta, TooN::Vector<3> &v3_dPhi);

  /// Convert Cartesian to spherical coordinates
  /** @param v3CartesianCoords The coordinates to convert
   *  @return The resultant spherical coordinates */
  static TooN::Vector<3> ConvertToSpherical(TooN::Vector<3> v3CartesianCoords);

  /// Const getter method for the parameter vector
  /** @return The parameter vector */
  const TooN::Vector<9> &GetParams() const
  {
    return mv9CameraParams;
  }

  /// Mutable getter method for the parameter vector
  /** @return The parameter vector */
  // TooN::Vector<9>& GetParams(){ return mv9CameraParams; }

  /// Const getter method for the inverse polynomial vector
  /** @return The parameter vector */
  const TooN::Vector<TooN::Resizable> &GetInvPoly() const
  {
    return mvxPolyInvCoeffs;
  }

  /// Mutable getter method for the inverse polynomial vector
  /** @return The parameter vector */
  TooN::Vector<TooN::Resizable> &GetInvPoly()
  {
    return mvxPolyInvCoeffs;
  }

  /** @brief Getter method for the Jacobian matrix of camera parameter
  *derivatives
  *
  * The Jacobian is the derivative of the last projected point with respect to
  *the 9 internal camera parameters.
  * Since this function is only used during calibration, the derivatives are
  *computed numerically.
  * @return The 2x9 Jacobian matrix */
  TooN::Matrix<2, 9> GetCameraParameterDerivs();

  /// Directly set the parameter vector
  /** @param v9Params The camera parameter vector, see mv9CameraParams for
   * details on what's what */
  void SetParams(TooN::Vector<9> v9Params);

  /// Update the camera parameters by summing with the argument vector
  /** @param v9Update The vector to sum with the current parameter vector.
   * Should be small in magnitude (ie from some kind of delta step)
   *         if you want things to work out as intended. */
  void UpdateParams(TooN::Vector<9> v9Update);

protected:
  // Polynomial functions
  /** @brief Fit a polynomial of specified degree to the given input vectors
   *
   * The coefficients of the polynomial of degree n (\f$ y = a_0 + a_1*x +
   *a_2*x^2 + ... + a_n*x^n \f$) are computed using a least squares fit
   * @param vxX Input points x, of arbitrary length, stacked as a column vector
   * @param vxY Input points y, of the same length as x, stacked as a column
   *vector
   * @param nDegree The degree of the polynomial that will be computed
   * @return The coefficients of the polynomial, with the lowest degree
   *coefficient first */
  TooN::Vector<> PolyFit(TooN::Vector<> vxX, TooN::Vector<> vxY, int nDegree);

  /// Evalute a polynomial with given coefficients at a specified location
  /** @param vxCoeff The vector of coefficients, ordered with the lowest degree
   * first
   *  @param dEvalAt The x value where the polynomial will be evaluated
   *  @return The resulting y value */
  template <int Size, class Precision, class Base>
  double PolyVal(const TooN::Vector<Size, Precision, Base> &vxCoeff, double dEvalAt);

  /** @brief Find the inverse of the polynomial describing the Taylor camera
   *model
   *
   * This function solves the specific problem of inverting the 4th order Taylor
   *camera polynomial \f$ tan\theta \rho = a_0 + a_2\rho^2 + a_3\rho^3 +
   *a_4\rho^4 \f$
   * into the function \f$ \rho = g(\theta) \f$, where \f$ g \f$ is a
   *polynomial. The degree of \f$ g \f$ can either be specified, in which case
   *the error
   * of the inverse fit could be large, or it could be found automatically by
   *attempting ever higher order inverse fits until the fit error drops below
   * a threshold. The maximum inverse polynomial degree is given by
   *MAX_INV_DEGREE, and if the inverse fit has a high error at this degree then
   *the camera
   * will not use the inverse polynomial but instead find roots using Newton's
   *method during projections (but this is slow).
   * @param v5PolyCoeffs The 5 coefficients of the 4th order camera polynomial
   * @param nSpecifiedDegree The degree of the inverse polynomial to fit. Set to
   *-1 (or leave at default) to get a fit with maximum error less than
   *dErrorLimit
   * @param dErrorLimit The maximum permissible error when attempting to find
   *the best inverse polynomial fit
   * @return The vector of coefficients of \f$ g \f$, ordered with the lowest
   *degree first*/
  TooN::Vector<> FindInvPolyUsingRoots(const TooN::Vector<5> &v5PolyCoeffs, int nSpecifiedDegree = -1,
                                       double dErrorLimit = 0.1);

  /** @brief Finds the root of the camera polynomial
   *
   * Uses Newton's method to find the roots of the given 4th order camera model
   *polynomial. To save on repeated work, the derivative also needs to be
   *supplied.
   * Note that this doesn't find the roots of an arbitrary polynomial (even if
   *it's degree 4), see the function PolyVal for the specific equation being
   *evaluted.
   * This function will assert on nMaxIter, because if Newton's method doesn't
   *find a root within this many iterations, either the starting position is way
   *off
   * or something else is wrong, but in any case the whole system will work very
   *poorly and slowly so you might as well stop and fix the problem.
   * @param v5Coeffs The coefficients of the 4th order camera polynomial
   * @param v4CoeffsDeriv The coefficients of the derivative of the polynomial
   * @param dTanTheta The tangent of the angle for which we want to find \f$
   *\rho \f$
   * @param dRhoInit The initial guess for \f$ \rho \f$
   * @param dErrorLimit The maximum error allowed in the solution
   * @param nMaxIter The maximum number of iterations allowed
   * @return The final \f$ \rho \f$ found */
  double FindRootWithNewton(TooN::Vector<5> v5Coeffs, TooN::Vector<4> v4CoeffsDeriv, double dTanTheta, double dRhoInit,
                            double dErrorLimit = 0.01, int nMaxIter = 50);

  /// Find the normalized Gaussian value of a vector of values
  /** @param vxX Vector of values
   *  @param dMean Mean of Gaussian
   *  @param dStd Standard deviation of Gaussian */
  TooN::Vector<> CenterAndScale(TooN::Vector<> vxX, double dMean, double dStd);

  /// Find the normalized Gaussian value of a value
  /** @param dVal Value
   *  @param dMean Mean of Gaussian
   *  @param dStd Standard deviation of Gaussian */
  double CenterAndScale(double dVal, double dMean, double dStd);

  /// Applies current settings by calculating new internal parameters
  void RefreshParams();

  // Variables
  /** @brief The current camera parameters
  *
  * The parameters (by index) are: \n
  * 0 - a0 coefficient \n
  * 1 - a2 coefficient \n
  * 2 - a3 coefficient \n
  * 3 - a4 coefficient \n
  * 4 - center of projection xc \n
  * 5 - center of projection yc \n
  * 6 - affine transform param c \n
  * 7 - affine transform param d \n
  * 8 - affine transform param e \n */
  TooN::Vector<9> mv9CameraParams;

  // Cached from the last project/unproject:
  TooN::Vector<3> mv3LastCam;      ///< Last point projected, in camera frame
  TooN::Vector<2> mv2LastIm;       ///< Last image coordinates
  TooN::Vector<2> mv2LastDistCam;  ///< Last distorted sensor plane coordinate
                                   /// (converted to mv2LastIm through affine transform and a translation)

  double mdLastRho;     ///< Last \f$ \rho \f$ value
  double mdLastCosPhi;  ///< Cosine of last \f$ \phi \f$
  double mdLastSinPhi;  ///< Sine of last \f$ \phi \f$

  bool mbInvalid;          ///< Was the last projection invalid?
  double mdLargestRadius;  ///< Largest radius in the image
  double mdMaxRho;         ///< Largest radius for which we consider projection valid
  double mdMinTheta;       ///< Minimum ray incidence angle allowed for valid projection
  double mdOnePixelAngle;  ///< Angle covered by a single pixel at the center of
  /// the image

  // Cached from last RefreshParams:
  TooN::Vector<2> mv2Center;         ///< Camera center of projection
  TooN::Vector<2> mv2ImageSize;      ///< The current image size
  TooN::Vector<2> mv2FullScaleSize;  ///< The area in unbinned pixels that the
  /// current image size takes up
  TooN::Vector<2> mv2CalibSize;  ///< The image sized used when the camera was
  /// calibrated. Always in unbinned pixels
  TooN::Vector<5> mv5PolyCoeffs;       ///< The 4th order polynomial coefficients
  TooN::Vector<4> mv4PolyDerivCoeffs;  ///< The coefficients of the derivatives
  /// of the polynomial
  TooN::Vector<5> mv5PolyDerivModCoeffs;  ///< Used for calculating projection derivatives
  TooN::Matrix<2> mm2Affine;              ///< The affine transformation matrix
  TooN::Matrix<2> mm2AffineInv;           ///< Inverse of the affine transformation

  // Related to inverting the polynomial
  bool mbUsingInversePoly;  ///< True if good inverse polynomial found, false
  /// otherwise
  TooN::Vector<2> mv2LinearInvCoeffs;  ///< Linear inverse fit coefficient to use
  /// for getting starting point for Newton's
  /// method
  TooN::Vector<TooN::Resizable> mvxPolyInvCoeffs;  ///< Inverse polynomial
  /// coefficients, ideally we'll
  /// use these
  double mdThetaMean;  ///< Used to scale theta for computing inverse polynomial
  /// to avoid bad conditioning
  double mdThetaStd;  ///< Used to scale theta for computing inverse polynomial
  /// to avoid bad conditioning

  bool mbCalibrationMode;  ///< Is the camera in calibration mode? If yes, only
  /// direct root finding will be used.
};

#endif  // MCPTAM_TAYLORCAMERA_H
