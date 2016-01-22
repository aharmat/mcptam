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
//=========================================================================================

#include <mcptam/TaylorCamera.h>
#include <mcptam/SmallMatrixOpts.h>
#include <mcptam/Utility.h>
#include <TooN/helpers.h>
#include <TooN/SVD.h>
#include <unsupported/Eigen/Polynomials>

using namespace TooN;

TaylorCamera::TaylorCamera(Vector<9> v9Params, CVD::ImageRef irCalibSize, CVD::ImageRef irFullScaleSize,
                           CVD::ImageRef irImageSize)
{
  mv9CameraParams = v9Params;
  mv2CalibSize = CVD::vec(irCalibSize);
  mv2FullScaleSize = CVD::vec(irFullScaleSize);
  mv2ImageSize = CVD::vec(irImageSize);
  mbCalibrationMode = false;

  RefreshParams();
}

TaylorCamera::TaylorCamera(CVD::ImageRef irCalibSize, CVD::ImageRef irFullScaleSize, CVD::ImageRef irImageSize)
{
  mv2CalibSize = CVD::vec(irCalibSize);
  mv2FullScaleSize = CVD::vec(irFullScaleSize);
  mv2ImageSize = CVD::vec(irImageSize);
  mbCalibrationMode = true;

  mv9CameraParams =
    makeVector(0, 0, 0, 0, 0, 0, 1, 0, 0);  // the only non-zero value is for affine matrix to be identity
  RefreshParams();

  ROS_WARN_STREAM(">>> TaylorCamera: In calibration mode, call SetParams AT LEAST ONCE before projecting or getting "
                  "derivatives!!! <<<");
}

void TaylorCamera::SetImageSize(CVD::ImageRef irImageSize)
{
  mv2ImageSize = CVD::vec(irImageSize);
  RefreshParams();
};

void TaylorCamera::SetParams(Vector<9> v9Params)
{
  mv9CameraParams = v9Params;
  RefreshParams();
}

// This updates internal member variables according to the current camera parameters,
// and the currently selected target image size.
void TaylorCamera::RefreshParams()
{
  // The parameters (by index) are:
  // 0 - a0 coefficient
  // 1 - a2 coefficient
  // 2 - a3 coefficient
  // 3 - a4 coefficient
  // 4 - center of projection xc
  // 5 - center of projection yc
  // 6 - affine transform param c
  // 7 - affine transform param d
  // 8 - affine transform param e

  // Fill the 4th order polynomial coefficient vector
  mv5PolyCoeffs[0] = mv9CameraParams[0];
  mv5PolyCoeffs[1] = 0;
  mv5PolyCoeffs[2] = mv9CameraParams[1];
  mv5PolyCoeffs[3] = mv9CameraParams[2];
  mv5PolyCoeffs[4] = mv9CameraParams[3];

  ROS_DEBUG_STREAM("TaylorCamera: Poly coeffs: " << mv5PolyCoeffs);

  // Used for calculating d_rho/d_theta in GetProjectionDerivs
  mv5PolyDerivModCoeffs = mv5PolyCoeffs;
  mv5PolyDerivModCoeffs[0] *= -1;
  mv5PolyDerivModCoeffs[3] *= 2;
  mv5PolyDerivModCoeffs[4] *= 3;

  ROS_DEBUG_STREAM("TaylorCamera: Poly deriv mod coeffs: " << mv5PolyDerivModCoeffs);

  Vector<2> v2Scale;
  v2Scale[0] = mv2ImageSize[0] / mv2FullScaleSize[0];
  v2Scale[1] = mv2ImageSize[1] / mv2FullScaleSize[1];

  ROS_DEBUG_STREAM("TaylorCamera: Scale: " << v2Scale);

  // The centers of projection xc,yc are found when the camera is calibrated,
  // so they are valid when the image size is mv2CalibSize. If mv2FullScaleSize
  // is different, the center of projection needs to be shifted
  Vector<2> v2FullScaleCenter;
  v2FullScaleCenter[0] = mv9CameraParams[4] - (mv2CalibSize[0] - mv2FullScaleSize[0]) / 2;
  v2FullScaleCenter[1] = mv9CameraParams[5] - (mv2CalibSize[1] - mv2FullScaleSize[1]) / 2;

  ROS_DEBUG_STREAM("TaylorCamera: Full scale center: " << v2FullScaleCenter);

  // Scale the center to the current image size
  mv2Center[0] = v2FullScaleCenter[0] * v2Scale[0];
  mv2Center[1] = v2FullScaleCenter[1] * v2Scale[1];

  ROS_DEBUG_STREAM("TaylorCamera: Center: " << mv2Center);

  // Find the furthest corner from the projection center
  Vector<2> v2Corner;
  v2Corner[0] = std::max(v2FullScaleCenter[0], mv2FullScaleSize[0] - v2FullScaleCenter[0] - 1);
  v2Corner[1] = std::max(v2FullScaleCenter[1], mv2FullScaleSize[1] - v2FullScaleCenter[1] - 1);

  // Largest radius is computed using full scale size because if the current image size is small due to
  // binning, it still captures the same volume of space as it would if it were bigger but with loss of information
  // Since the camera parameters are calibrated with no binning, all polynomial computations must be done with
  // a full scale image
  mdLargestRadius = sqrt(v2Corner * v2Corner);

  // At what stage does the model become invalid?
  mdMaxRho = 1.0 * mdLargestRadius;  // (no reason to allow more)

  ROS_DEBUG_STREAM("Max rho: " << mdMaxRho);

  // If the theta of a projection is less than this, it is outside the valid model region
  mdMinTheta = atan(PolyVal(mv5PolyCoeffs, mdMaxRho) / mdMaxRho);

  if (!mbCalibrationMode)
  {
    mbUsingInversePoly = true;
    mvxPolyInvCoeffs = FindInvPolyUsingRoots(mv5PolyCoeffs, -1, 0.0001);

    if (mvxPolyInvCoeffs.size() == 0)  // Couldn't get good inverse polynomial
    {
      ROS_ERROR_STREAM("TaylorCamera: Couldn't find inverse polynomial with degree <= "
                       << MAX_INV_DEGREE << ", will be solving for roots with Newton's method which is slow");

      mbUsingInversePoly = false;

      // Get a linear inverse model
      mv2LinearInvCoeffs = FindInvPolyUsingRoots(mv5PolyCoeffs, 1);
      ROS_INFO_STREAM("TaylorCamera: Linear inverse polynomial coefficients: " << mv2LinearInvCoeffs);

      mv4PolyDerivCoeffs = mv5PolyCoeffs.slice(1, 4);
      for (int i = 0; i < mv4PolyDerivCoeffs.size(); ++i)
        mv4PolyDerivCoeffs[i] *= (i + 1);

      ROS_INFO_STREAM("TaylorCamera: Derivative polynomial coefficients: " << mv4PolyDerivCoeffs);
    }
    else
    {
      ROS_INFO_STREAM("TaylorCamera: Inverse polynomial coefficients: " << mvxPolyInvCoeffs);
    }
  }

  // Create affine transformation matrix from given parameters and image scale
  mm2Affine[0][0] = v2Scale[0] * mv9CameraParams[6];
  mm2Affine[0][1] = v2Scale[1] * mv9CameraParams[7];
  mm2Affine[1][0] = v2Scale[0] * mv9CameraParams[8];
  mm2Affine[1][1] = v2Scale[1] * 1;

  ROS_DEBUG_STREAM("TaylorCamera: Affine transform: " << mm2Affine);

  mm2AffineInv = opts::M2Inverse(mm2Affine);  // From SmallMatrixOpts.h

  // Work out angle spanned by one pixel
  // (This only really makes sense for square-ish pixels)
  Vector<3> v3Center = UnProject(mv2ImageSize / 2);
  Vector<3> v3RootTwoAway = UnProject(mv2ImageSize / 2 + CVD::vec(CVD::ImageRef(1, 1)));
  mdOnePixelAngle = acos(v3Center * v3RootTwoAway) / sqrt(2.0);
}

// Project from the camera reference frame to image pixels,
// while storing intermediate calculation results in member variables
Vector<2> TaylorCamera::Project(const Vector<3>& v3CamFrame)
{
  mv3LastCam = v3CamFrame;

  double dNorm = sqrt(mv3LastCam[0] * mv3LastCam[0] + mv3LastCam[1] * mv3LastCam[1]);
  double dTheta, dTanTheta;

  if (dNorm == 0)  // Special case, need to short circuit everything because rho will be 0
  {
    dTheta = M_PI_2;
    dTanTheta = NAN;
  }
  else
  {
    dTanTheta = mv3LastCam[2] / dNorm;
    dTheta = atan(dTanTheta);
  }

  // ROS_DEBUG_STREAM("dNorm: "<<dNorm);
  // ROS_DEBUG_STREAM("dTanTheta: "<<dTanTheta<<" dTheta: "<<dTheta);

  mbInvalid = (dTheta < mdMinTheta);

  if (dNorm == 0)
  {
    mdLastRho = 0;
    mdLastCosPhi = 0;
    mdLastSinPhi = 0;
  }
  else
  {
    // If we're in calibration mode we want to find roots of polynomial directly
    // because bad roots indicate a bad polynomial and we need to set the invalid flag
    if (mbCalibrationMode)
    {
      Eigen::Matrix<double, 5, 1> polynomial;
      polynomial << mv5PolyCoeffs[0], mv5PolyCoeffs[1] - dTanTheta, mv5PolyCoeffs[2], mv5PolyCoeffs[3],
                 mv5PolyCoeffs[4];

      Eigen::PolynomialSolver<double, 4> psolve(polynomial);
      std::vector<double> vRealRoots;
      psolve.realRoots(vRealRoots);

      for (int i = vRealRoots.size() - 1; i >= 0; --i)
      {
        if (vRealRoots[i] < 0.0 || vRealRoots[i] > mdMaxRho)
          vRealRoots.erase(vRealRoots.begin() + i);
      }

      if (vRealRoots.size() != 1)  // no roots or more than 1 root means a bad polynomial
      {
        mbInvalid = true;
        return makeVector(-1, -1);
      }

      ROS_ASSERT(vRealRoots.size() > 0);
      mdLastRho = vRealRoots[0];
    }
    else  // Otherwise we're running live, want to find rho as fast as possible
    {
      if (mbUsingInversePoly)
        mdLastRho = PolyVal(mvxPolyInvCoeffs, CenterAndScale(dTheta, mdThetaMean, mdThetaStd));  // If we have a good
      // inverse poly, that's
      // the fastest
      else
      {
        // Otherwise use Newton's method. Get an approximate solution from linear inverse poly
        double rho_approx = PolyVal(mv2LinearInvCoeffs, CenterAndScale(dTheta, mdThetaMean, mdThetaStd));
        mdLastRho = FindRootWithNewton(mv5PolyCoeffs, mv4PolyDerivCoeffs, dTanTheta, rho_approx);
      }
    }

    mdLastCosPhi = mv3LastCam[0] / dNorm;
    mdLastSinPhi = mv3LastCam[1] / dNorm;
  }

  // ROS_DEBUG_STREAM("mdLastRho: "<<mdLastRho);
  // ROS_DEBUG_STREAM("mdLastCosPhi: "<<mdLastCosPhi<<" mdLastSinPhi: "<<mdLastSinPhi);

  // On the sensor plane, before affine transform to image plane
  mv2LastDistCam[0] = mdLastCosPhi * mdLastRho;
  mv2LastDistCam[1] = mdLastSinPhi * mdLastRho;

  mv2LastIm = mm2Affine * mv2LastDistCam + mv2Center;

  mbInvalid |= !(util::PointInRectangle(mv2LastIm, mv2ImageSize));

  return mv2LastIm;
}

// Uses Newton's method to find the roots of the given 4th order camera model polynomial. To save on repeated work, the
// derivative also needs to be supplied.
// Note that this doesn't find the roots of an arbitrary polynomial (even if it's degree 4), see the function PolyVal
// for the specific equation being evaluted.
// This function will assert on nMaxIter, because if Newton's method doesn't find a root within thins many iterations,
// either the starting position is way off
// or something else is wrong, but in any case the whole system will work very poorly and slowly so you might as well
// stop and fix the problem.
double TaylorCamera::FindRootWithNewton(Vector<5> v5Coeffs, Vector<4> v4CoeffsDeriv, double dTanTheta, double dRhoInit,
                                        double dErrorLimit, int nMaxIter)
{
  v5Coeffs[1] -= dTanTheta;
  v4CoeffsDeriv[0] -= dTanTheta;

  double dRho = dRhoInit;
  double dRhoPrev = dRhoInit;
  double dError = 1;
  int i = 0;

  // Main loop of Newton's method
  while (dError > dErrorLimit)
  {
    dRho = dRhoPrev - PolyVal(v5Coeffs, dRhoPrev) / PolyVal(v4CoeffsDeriv, dRhoPrev);
    dError = abs(dRho - dRhoPrev);
    dRhoPrev = dRho;

    ++i;
    ROS_ASSERT(i <= nMaxIter);  // shouldn't take this long to converge
  }

  return dRho;
}

// Un-project from image pixel coords to the camera r=1 sphere
// while storing intermediate calculation results in member variables
Vector<3> TaylorCamera::UnProject(const Vector<2>& v2ImFrame)
{
  mv2LastIm = v2ImFrame;
  mv2LastDistCam = mm2AffineInv * (v2ImFrame - mv2Center);

  mdLastRho = sqrt(mv2LastDistCam * mv2LastDistCam);

  mv3LastCam[0] = mv2LastDistCam[0];
  mv3LastCam[1] = mv2LastDistCam[1];
  mv3LastCam[2] = PolyVal(mv5PolyCoeffs, mdLastRho);

  // ROS_DEBUG_STREAM("mv3LastCam: "<<mv3LastCam);

  if (mdLastRho == 0)  // protect against div by zero
  {
    mdLastCosPhi = 0;
    mdLastSinPhi = 0;
  }
  else
  {
    mdLastCosPhi = mv3LastCam[0] / mdLastRho;
    mdLastSinPhi = mv3LastCam[1] / mdLastRho;
  }

  normalize(mv3LastCam);

  return mv3LastCam;
}

// Get the derivative of image frame wrt the camera sphere (radius 1) at the last computed projection
// in the matrix form |d_im1/d_theta  d_im1/d_phi  d_im1/d_r|
//                    |d_im2/d_theta  d_im2/d_phi  d_im2/d_r|
// Derivatives with respect to r are zero since image of a point does not change as it moves along the view ray
// therefore the output matrix is 2x2 instead
Matrix<2> TaylorCamera::GetProjectionDerivs()
{
  double w = PolyVal(mv5PolyCoeffs, mdLastRho);

  // This is found by taking the derivative of the above equation (keeping in mind w = rho * tan(theta))
  double dRho_dTheta = (mdLastRho * mdLastRho + w * w) / PolyVal(mv5PolyDerivModCoeffs, mdLastRho);

  // ROS_DEBUG_STREAM("mdLastRho: "<<mdLastRho);
  // ROS_DEBUG_STREAM("dRho_dTheta: "<<dRho_dTheta);

  // If the sensor plane coords are (u,v), then u = rho * cos(phi) and v = rho * sin(phi)
  // So d_u/d_theta = d_u/d_rho * d_rho/d_theta and similar for the others
  Vector<2> dLast_dTheta;
  dLast_dTheta[0] = mdLastCosPhi * dRho_dTheta;
  dLast_dTheta[1] = mdLastSinPhi * dRho_dTheta;

  Vector<2> dLast_dPhi;
  dLast_dPhi[0] = -mdLastSinPhi * mdLastRho;
  dLast_dPhi[1] = mdLastCosPhi * mdLastRho;

  // Transform to image plane
  Vector<2> dIm_dTheta = mm2Affine * dLast_dTheta;
  Vector<2> dIm_dPhi = mm2Affine * dLast_dPhi;

  Matrix<2> m2Derivs;

  m2Derivs.T()[0] = dIm_dTheta;
  m2Derivs.T()[1] = dIm_dPhi;

  return m2Derivs;
}

// Derivatives of the last projection wrt to the camera parameters
// Use these to calibrate the camera
// No need for this to be quick, so do them numerically
Matrix<2, 9> TaylorCamera::GetCameraParameterDerivs()
{
  Matrix<2, 9> m29NumDerivs;
  Vector<9> v9Original = mv9CameraParams;  // save the actual params
  Vector<3> v3Cam = mv3LastCam;
  Vector<2> v2Out = Project(v3Cam);

  for (int i = 0; i < 9; i++)
  {
    Vector<9> v9Update;
    v9Update = Zeros;
    double const dMinUpdateMag = 1e-10;

    double dUpdateMag = fabs(v9Original[i] / 100.0);  // Make update magnitude 1% of current value

    if (dUpdateMag < dMinUpdateMag)  // ie one of the default values is zero (ie d, e)
    {
      dUpdateMag = dMinUpdateMag;
    }

    v9Update[i] += dUpdateMag;
    UpdateParams(v9Update);
    Vector<2> v2Out_B = Project(v3Cam);                    // Get projected position with the new camera params
    m29NumDerivs.T()[i] = (v2Out_B - v2Out) / dUpdateMag;  // Derivative is change in image location / update magnitude
    mv9CameraParams = v9Original;                          // Reset camera params
    RefreshParams();
  }

  return m29NumDerivs;
}

// Update the camera parameters; use this as part of camera calibration.
void TaylorCamera::UpdateParams(Vector<9> v9Update)
{
  mv9CameraParams = mv9CameraParams + v9Update;
  RefreshParams();
}

// Find the normalized Gaussian value of a vector of values
Vector<> TaylorCamera::CenterAndScale(Vector<> vxX, double dMean, double dStd)
{
  return (vxX - TooN::Ones(vxX.size()) * dMean) / dStd;
}

// Find the normalized Gaussian value of a value
double TaylorCamera::CenterAndScale(double dVal, double dMean, double dStd)
{
  return (dVal - dMean) / dStd;
}

// Fit a polynomial of specified degree to the given input vectors
// See header file for more details and the doxygen HTML page for a nicer view of the equation
Vector<> TaylorCamera::PolyFit(Vector<> vxX, Vector<> vxY, int nDegree)
{
  Vector<> v0Empty(0);

  if (vxX.size() != vxY.size())
    return v0Empty;

  if (nDegree < 1)
    return v0Empty;

  int nDimensions = vxX.size();

  // Uses the Vandermonde matrix method of fitting a least squares polynomial to a set of data
  Matrix<> mxVandermondeTrans(nDegree + 1, nDimensions);

  mxVandermondeTrans[0] = Ones(nDimensions);
  mxVandermondeTrans[1] = vxX;

  for (int i = 2; i <= nDegree; ++i)
  {
    mxVandermondeTrans[i] = mxVandermondeTrans[i - 1] * vxX.as_diagonal();
  }

  // create the SVD decomposition
  SVD<> svd(mxVandermondeTrans.T());

  Vector<> a = svd.backsub(vxY);

  return a;
}

// Evalute a polynomial with given coefficients at a specified location
template <int Size, class Precision, class Base>
double TaylorCamera::PolyVal(const Vector<Size, Precision, Base>& vxCoeff, double dEvalAt)
{
  // Polynomials are stored with the coefficient of zero in the first spot
  // This is the opposive of how matlab's polyval function takes the coefficients
  double val = 0;
  for (int i = vxCoeff.size() - 1; i > 0; i--)
  {
    val += vxCoeff[i];
    val *= dEvalAt;
  }

  val += vxCoeff[0];
  return val;
}

// Find the inverse of the polynomial describing the Taylor camera model
// See the header and/or doxygen HTML file for more info
Vector<> TaylorCamera::FindInvPolyUsingRoots(const Vector<5>& v5PolyCoeffs, int nSpecifiedDegree, double dErrorLimit)
{
  // We're going to generate a series of closely spaced thetas, and find the corresponding rho for
  // each theta using a polynomial root finder in the Eigen package. After discrading all thetas
  // where no valid rho was found, a polynomial is fitted to the resulting data, keeping in mind
  // that the thetas are now the x values and the rhos are the y values. This polynomial is the
  // inverse of the one passed into the function, to a certain error tolerance.

  // Generate the vector of thetas
  double dThetaStart = -M_PI / 2 + 0.001;
  double dThetaEnd = M_PI / 2 - 0.001;
  double dThetaStep = 0.01;
  int nThetaNum = ceil((dThetaEnd - dThetaStart) / dThetaStep) + 1;

  std::vector<double> vThetaVector(nThetaNum);
  std::vector<double> vRhoVector(nThetaNum);
  int nDeleteNum = 0;

  vThetaVector[0] = dThetaStart;
  for (unsigned i = 1; i < vThetaVector.size(); ++i)
  {
    vThetaVector[i] = vThetaVector[i - 1] + dThetaStep;
  }

  // Find rho from the theta vector by solving for roots of polynomial equation
  for (unsigned i = 0; i < vRhoVector.size(); ++i)
  {
    Eigen::Matrix<double, 5, 1> polynomial;
    polynomial << v5PolyCoeffs[0], v5PolyCoeffs[1] - tan(vThetaVector[i]), v5PolyCoeffs[2], v5PolyCoeffs[3],
               v5PolyCoeffs[4];

    Eigen::PolynomialSolver<double, 4> psolve(polynomial);
    std::vector<double> vRealRoots;
    psolve.realRoots(vRealRoots);

    for (int j = vRealRoots.size() - 1; j >= 0; --j)
    {
      if (vRealRoots[j] < 0.0 || vRealRoots[j] > mdMaxRho)
        vRealRoots.erase(vRealRoots.begin() + j);
    }

    if (vRealRoots.size() != 1)  // Zero or more than one real root means theta outside range of model
    {
      vRhoVector[i] = -9999;  // mark with a recognizable value
      nDeleteNum++;
    }
    else
      vRhoVector[i] = vRealRoots[0];
  }

  // Create the vectors that will be fitted with a polynomial
  Vector<> vxTheta(nThetaNum - nDeleteNum);
  Vector<> vxRho(nThetaNum - nDeleteNum);
  std::vector<double> vError(nThetaNum - nDeleteNum);

  // Fill the vectors and compute mean of valid thetas
  int j = 0;
  mdThetaMean = 0;
  for (unsigned i = 0; i < vRhoVector.size(); ++i)
  {
    if (vRhoVector[i] == -9999)
      continue;

    vxTheta[j] = vThetaVector[i];
    vxRho[j] = vRhoVector[i];

    mdThetaMean += vThetaVector[i];

    j++;
  }

  mdThetaMean /= vxTheta.size();

  // Now get standard deviation
  Vector<> vxThetaShifted = vxTheta - TooN::Ones(vxTheta.size()) * mdThetaMean;
  mdThetaStd = sqrt((vxThetaShifted * vxThetaShifted) / vxThetaShifted.size());

  // Center and scale the thetas to make sure the polynomial is well conditioned
  vxTheta = CenterAndScale(vxTheta, mdThetaMean, mdThetaStd);

  if (nSpecifiedDegree < 0)  // We should find the best polynomial fit (within limits)
  {
    int nDegree = 2;  // Start with 2nd order poly
    double dMaxError = 1e10;

    while (dMaxError > dErrorLimit && nDegree <= MAX_INV_DEGREE)
    {
      Vector<> vxInvCoeffs = PolyFit(vxTheta, vxRho, nDegree);

      // Evaluate the fit at each theta
      for (int i = 0; i < vxTheta.size(); ++i)
      {
        double rho_fit = PolyVal(vxInvCoeffs, vxTheta[i]);
        vError[i] = fabs(vxRho[i] - rho_fit);
      }

      dMaxError = *(std::max_element(vError.begin(), vError.end()));

      if (dMaxError <= dErrorLimit)
        return vxInvCoeffs;

      ++nDegree;
    }

    ROS_ERROR_STREAM("Hit degree limit, max error was: " << dMaxError << " and the limit was: " << dErrorLimit);

    // If we're here, we've hit the degree limit so return an empty
    // vector, which will indicate a bad fit
    Vector<> v0Empty(0);
    return v0Empty;
  }

  // Otherwise we're given a specified degree, so just find a fit and return the coeffs
  return PolyFit(vxTheta, vxRho, nSpecifiedDegree);
}

// Convert Cartesian to spherical coordinates
Vector<3> TaylorCamera::ConvertToSpherical(Vector<3> v3CartesianCoords)
{
  double r = sqrt(v3CartesianCoords * v3CartesianCoords);
  double theta = asin(v3CartesianCoords[2] / r);
  double phi = atan2(v3CartesianCoords[1], v3CartesianCoords[0]);

  return makeVector(theta, phi, r);
}

// Get the derivatives of a cartesian 3-vector projected onto the unit sphere, in spherical coordinates
void TaylorCamera::GetCamSphereDeriv(const Vector<3>& v3Cam, TooN::Vector<3>& v3_dTheta, TooN::Vector<3>& v3_dPhi)
{
  // The derivatives to be found are d_theta/d_point (a 3-vector)
  // and d_phi/d_point (also a 3-vector)

  // The following math can be derived from the equations converting Cartesian coordinates
  // to spherical coordinates. In this particular implementation, theta is expressed
  // as zero on the xy plane and increasing positively towards the z axis, while phi is
  // expressed as zero on the x axis and increasing positively towards the y axis.

  const double& x = v3Cam[0];
  const double& y = v3Cam[1];
  const double& z = v3Cam[2];

  double x2 = x * x;
  double y2 = y * y;
  double z2 = z * z;

  double n = sqrt(x * x + y * y);
  double n2 = n * n;
  double n3 = n2 * n;

  // Derivative of theta component of spherical coordinates with
  // respect to point cartesian coords
  if (n == 0)
  {
    v3_dTheta[0] = 0;  // dTheta/dx
    v3_dTheta[1] = 0;  // dTheta/dy
    v3_dTheta[2] = 0;  // dTheta/dz
  }
  else
  {
    v3_dTheta[0] = -z * x / (n3 + n * z2);  // dTheta/dx
    v3_dTheta[1] = -z * y / (n3 + n * z2);  // dTheta/dy
    v3_dTheta[2] = n / (n2 + z2);           // dTheta/dz
  }

  // Derivative of phi component of spherical coordinates with
  // respect to point cartesian coords
  if (x == 0 && y == 0)
  {
    v3_dPhi[0] = 0;  // dPhi/dx
    v3_dPhi[1] = 0;  // dPhi/dy
    v3_dPhi[2] = 0;  // dPhi/dz, zero since phi = atan(y/x)
  }
  else
  {
    v3_dPhi[0] = -y / (x2 + y2);  // dPhi/dx
    v3_dPhi[1] = x / (x2 + y2);   // dPhi/dy
    v3_dPhi[2] = 0;               // dPhi/dz, zero since phi = atan(y/x)
  }
}
