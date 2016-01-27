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

// Copyright 2008 Isis Innovation Limited
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/TaylorCamera.h>
#include <mcptam/KeyFrame.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/vision.h>
#include <TooN/Cholesky.h>
#include <TooN/wls.h>
#include <TooN/SVD.h>
#include <utility>
#include <algorithm>


// #include <tag/kalmanfilter.h>
// #include <tag/constantposition.h>
// #include <tag/measurements.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

CVD::ImageRef SmallBlurryImage::sirSize(40, 30);

SmallBlurryImage::SmallBlurryImage(KeyFrame &kf, double dBlur)
{
  mbMadeJacs = false;
  MakeFromKF(kf, dBlur);
}

SmallBlurryImage::SmallBlurryImage()
{
  mbMadeJacs = false;
}

// Make a SmallBlurryImage from a KeyFrame This fills in the mimSmall
// image (Which is just a small un-blurred version of the KF) and
// mimTemplate (which is a floating-point, zero-mean blurred version
// of the above)
void SmallBlurryImage::MakeFromKF(KeyFrame &kf, double dBlur)
{
  mbMadeJacs = false;

  mimSmall.resize(sirSize);
  mimTemplate.resize(sirSize);

  mbMadeJacs = false;

  // Resize the level 0 image to get best results
  cv::Mat imgWrapped(kf.maLevels[0].image.size().y, kf.maLevels[0].image.size().x, CV_8U, kf.maLevels[0].image.data(),
                     kf.maLevels[0].image.row_stride());
  cv::Mat imgSmallWrapped(mimSmall.size().y, mimSmall.size().x, CV_8U, mimSmall.data(), mimSmall.row_stride());
  cv::resize(imgWrapped, imgSmallWrapped, imgSmallWrapped.size(), 0, 0, cv::INTER_LINEAR);

  CVD::ImageRef ir;
  unsigned int nSum = 0;
  do
  {
    nSum += mimSmall[ir];
  }
  while (ir.next(sirSize));

  float fMean = (static_cast<float>(nSum)) / sirSize.area();

  ir.home();
  do
  {
    mimTemplate[ir] = mimSmall[ir] - fMean;
  }
  while (ir.next(sirSize));

  convolveGaussian(mimTemplate, dBlur);
}

// Make the jacobians (actually, no more than a gradient image)
// of the blurred template
void SmallBlurryImage::MakeJacs()
{
  mimImageJacs.resize(sirSize);
  // Fill in the gradient image
  CVD::ImageRef ir;
  do
  {
    TooN::Vector<2> &v2Grad = mimImageJacs[ir];
    if (mimTemplate.in_image_with_border(ir, 1))
    {
      v2Grad[0] = mimTemplate[ir + CVD::ImageRef(1, 0)] - mimTemplate[ir - CVD::ImageRef(1, 0)];
      v2Grad[1] = mimTemplate[ir + CVD::ImageRef(0, 1)] - mimTemplate[ir - CVD::ImageRef(0, 1)];
      // N.b. missing 0.5 factor in above, this will be added later.
    }
    else
      v2Grad = TooN::Zeros;
  }
  while (ir.next(sirSize));
  mbMadeJacs = true;
};

// Calculate the zero-mean SSD between one image and the next.
// Since both are zero mean already, just calculate the SSD...
double SmallBlurryImage::ZMSSD(SmallBlurryImage &other)
{
  double dSSD = 0.0;
  CVD::ImageRef ir;
  do
  {
    double dDiff = mimTemplate[ir] - other.mimTemplate[ir];
    dSSD += dDiff * dDiff;
  }
  while (ir.next(sirSize));
  return dSSD;
}

// Find an SE2 which best aligns an SBI to a target
// Do this by ESM-tracking a la Benhimane & Malis
std::pair<TooN::SE2<>, double> SmallBlurryImage::IteratePosRelToTarget(SmallBlurryImage &other, int nIterations)
{
  TooN::SE2<> se2CtoC;
  TooN::SE2<> se2WfromC;
  CVD::ImageRef irCenter = sirSize / 2;
  se2WfromC.get_translation() = CVD::vec(irCenter);

  std::pair<TooN::SE2<>, double> result_pair;
  if (!other.mbMadeJacs)
  {
    ROS_FATAL("SmallBlurryImage: You spanner, you didn't make the jacs for the target.");
    ROS_ASSERT(other.mbMadeJacs);
  }

  double dMeanOffset = 0.0;
  TooN::Vector<4> v4Accum;

  TooN::Vector<10> v10Triangle;
  CVD::Image<float> imWarped(sirSize);

  double dFinalScore = 0.0;
  for (int it = 0; it < nIterations; it++)
  {
    dFinalScore = 0.0;
    v4Accum = TooN::Zeros;
    v10Triangle = TooN::Zeros;  // Holds the bottom-left triangle of JTJ
    TooN::Vector<4> v4Jac;
    v4Jac[3] = 1.0;

    TooN::SE2<> se2XForm = se2WfromC * se2CtoC * se2WfromC.inverse();

    // Make the warped current image template:
    TooN::Vector<2> v2Zero = TooN::Zeros;
    CVD::transform(mimTemplate, imWarped, se2XForm.get_rotation().get_matrix(), se2XForm.get_translation(), v2Zero,
                   -9e20f);

    // Now compare images, calc differences, and current image jacobian:
    CVD::ImageRef ir;
    do
    {
      if (!imWarped.in_image_with_border(ir, 1))
        continue;

      float l, r, u, d, here;
      l = imWarped[ir - CVD::ImageRef(1, 0)];
      r = imWarped[ir + CVD::ImageRef(1, 0)];
      u = imWarped[ir - CVD::ImageRef(0, 1)];
      d = imWarped[ir + CVD::ImageRef(0, 1)];
      here = imWarped[ir];
      if (l + r + u + d + here < -9999.9)  // This means it's out of the image; c.f. the -9e20f param to transform.
        continue;

      TooN::Vector<2> v2CurrentGrad;
      v2CurrentGrad[0] = r - l;  // Missing 0.5 factor
      v2CurrentGrad[1] = d - u;

      TooN::Vector<2> v2SumGrad = 0.25 * (v2CurrentGrad + other.mimImageJacs[ir]);
      // Why 0.25? This is from missing 0.5 factors: One for
      // the fact we average two gradients, the other from
      // each gradient missing a 0.5 factor.

      v4Jac[0] = v2SumGrad[0];
      v4Jac[1] = v2SumGrad[1];
      v4Jac[2] = -(ir.y - irCenter.y) * v2SumGrad[0] + (ir.x - irCenter.x) * v2SumGrad[1];
      //    v4Jac[3] = 1.0;

      double dDiff = imWarped[ir] - other.mimTemplate[ir] + dMeanOffset;
      dFinalScore += dDiff * dDiff;

      v4Accum += dDiff * v4Jac;

      // Speedy fill of the LL triangle of JTJ:
      double *p = &v10Triangle[0];
      *p++ += v4Jac[0] * v4Jac[0];
      *p++ += v4Jac[1] * v4Jac[0];
      *p++ += v4Jac[1] * v4Jac[1];
      *p++ += v4Jac[2] * v4Jac[0];
      *p++ += v4Jac[2] * v4Jac[1];
      *p++ += v4Jac[2] * v4Jac[2];
      *p++ += v4Jac[0];
      *p++ += v4Jac[1];
      *p++ += v4Jac[2];
      *p++ += 1.0;
    }
    while (ir.next(sirSize));

    TooN::Vector<4> v4Update;

    // Solve for JTJ-1JTv;
    {
      TooN::Matrix<4> m4;
      int v = 0;
      for (int j = 0; j < 4; j++)
      {
        for (int i = 0; i <= j; i++)
          m4[j][i] = m4[i][j] = v10Triangle[v++];
      }
      TooN::Cholesky<4> chol(m4);
      v4Update = chol.backsub(v4Accum);
    }

    TooN::SE2<> se2Update;
    se2Update.get_translation() = -v4Update.slice<0, 2>();
    se2Update.get_rotation() = TooN::SO2<>::exp(-v4Update[2]);
    se2CtoC = se2CtoC * se2Update;
    dMeanOffset -= v4Update[3];
  }

  result_pair.first = se2CtoC;
  result_pair.second = dFinalScore;
  return result_pair;
}

// What is the 3D camera rotation (zero trans) SE3<> which causes an
// input image SO2 rotation?
TooN::SE3<> SmallBlurryImage::SE3fromSE2(TooN::SE2<> se2, TaylorCamera &cameraSrc, TaylorCamera &cameraTarget)
{
  // Do this by projecting two points, and then iterating the SE3<> (SO3
  // actually) until convergence. It might seem stupid doing this so
  // precisely when the whole SE2-finding is one big hack, but hey.

  if (cameraSrc.GetImageSize() != sirSize)
    cameraSrc.SetImageSize(sirSize);

  if (cameraTarget.GetImageSize() != sirSize)
    cameraTarget.SetImageSize(sirSize);

  TooN::Vector<2> av2Turned[2];  // Our two warped points in pixels
  av2Turned[0] = CVD::vec(sirSize / 2) + se2 * CVD::vec(CVD::ImageRef(5, 0));
  av2Turned[1] = CVD::vec(sirSize / 2) + se2 * CVD::vec(CVD::ImageRef(-5, 0));

  TooN::Vector<3> av3OrigPoints[2];  // 3D versions of these points.
  av3OrigPoints[0] = cameraTarget.UnProject(CVD::vec(sirSize / 2) + CVD::vec(CVD::ImageRef(5, 0)));
  av3OrigPoints[1] = cameraTarget.UnProject(CVD::vec(sirSize / 2) + CVD::vec(CVD::ImageRef(-5, 0)));

  TooN::SO3<> so3;
  for (int it = 0; it < 3; it++)
  {
    TooN::WLS<3> wls;  // lazy; no need for the 'W'
    wls.add_prior(10.0);
    for (int i = 0; i < 2; i++)
    {
      // Project into the image to find error
      TooN::Vector<3> v3Cam = so3 * av3OrigPoints[i];
      TooN::Vector<2> v2Pixels = cameraSrc.Project(v3Cam);
      TooN::Vector<2> v2Error = av2Turned[i] - v2Pixels;

      TooN::Matrix<2> m2CamDerivs = cameraSrc.GetProjectionDerivs();
      TooN::Matrix<2, 3> m23Jacobian;

      for (int m = 0; m < 3; m++)
      {
        const TooN::Vector<3> v3Motion = TooN::SO3<>::generator_field(m, v3Cam);
        TooN::Vector<3> v3_dTheta, v3_dPhi;
        TaylorCamera::GetCamSphereDeriv(v3Cam, v3_dTheta, v3_dPhi);
        TooN::Vector<2> v2CamSphereMotion;
        v2CamSphereMotion[0] = v3_dTheta * v3Motion;  // theta component
        v2CamSphereMotion[1] = v3_dPhi * v3Motion;    // phi component

        m23Jacobian.T()[m] = m2CamDerivs * v2CamSphereMotion;
      }

      wls.add_mJ(v2Error[0], m23Jacobian[0], 1.0);
      wls.add_mJ(v2Error[1], m23Jacobian[1], 1.0);
    }

    wls.compute();
    TooN::Vector<3> v3Res = wls.get_mu();
    so3 = TooN::SO3<>::exp(v3Res) * so3;
  }

  TooN::SE3<> se3Result;
  se3Result.get_rotation() = so3;

  return se3Result;
}
/*
void SmallBlurryImage::SE3fromSE2(TooN::SE2<> se2, TaylorCamera& cameraSrc, TaylorCamera& cameraTarget, TooN::SE3<>& se3,
TooN::Matrix<6>& m6Cov)
{
  // Do this by projecting two points, and then iterating the SE3<> (SO3
  // actually) until convergence. It might seem stupid doing this so
  // precisely when the whole SE2-finding is one big hack, but hey.

  if(cameraSrc.GetImageSize() != sirSize)
    cameraSrc.SetImageSize(sirSize);

  if(cameraTarget.GetImageSize() != sirSize)
    cameraTarget.SetImageSize(sirSize);

  TooN::Vector<2> av2Turned[2];   // Our two warped points in pixels
  av2Turned[0] = CVD::vec(sirSize / 2) + se2 * CVD::vec(CVD::ImageRef(5,0));
  av2Turned[1] = CVD::vec(sirSize / 2) + se2 * CVD::vec(CVD::ImageRef(-5,0));

  TooN::Vector<3> av3OrigPoints[2];   // 3D versions of these points.
  av3OrigPoints[0] = cameraTarget.UnProject(CVD::vec(sirSize / 2) + CVD::vec(CVD::ImageRef(5,0)));
  av3OrigPoints[1] = cameraTarget.UnProject(CVD::vec(sirSize / 2) + CVD::vec(CVD::ImageRef(-5,0)));

  TooN::SO3<> so3;
  tag::KalmanFilter<tag::ConstantPosition::State, tag::ConstantPosition::Model> filter;
  for(int it = 0; it<3; it++)
  {
    WLS<3> wls;  // lazy; no need for the 'W'
    wls.add_prior(10.0);
    for(int i=0; i<2; i++)
    {
      // Project into the image to find error
      TooN::Vector<3> v3Cam = so3 * av3OrigPoints[i];
      TooN::Vector<2> v2Pixels = cameraSrc.Project(v3Cam);
      TooN::Vector<2> v2Error = av2Turned[i] - v2Pixels;

      TooN::Matrix<2> m2CamDerivs = cameraSrc.GetProjectionDerivs();
      TooN::Matrix<2,3> m23Jacobian;

      for(int m=0; m<3; m++)
      {
        const TooN::Vector<3> v3Motion = TooN::SO3<>::generator_field(m, v3Cam);
        TooN::Vector<3> v3_dTheta, v3_dPhi;
        TaylorCamera::GetCamSphereDeriv(v3Cam, v3_dTheta, v3_dPhi);
        TooN::Vector<2> v2CamSphereMotion;
        v2CamSphereMotion[0] = v3_dTheta * v3Motion;  // theta component
        v2CamSphereMotion[1] = v3_dPhi * v3Motion;  // phi component

        m23Jacobian.T()[m] = m2CamDerivs * v2CamSphereMotion;
      }

      wls.add_mJ(v2Error[0], m23Jacobian[0], 1.0);
      wls.add_mJ(v2Error[1], m23Jacobian[1], 1.0);
    }

    wls.compute();
    TooN::Vector<3> v3Res = wls.get_mu();

    if(it == 0)
    {
      filter.state.pose.get_rotation() = TooN::SO3<>::exp(v3Res);
      filter.state.covariance.slice<3,3,3,3>() = SVD<3>(wls.get_C_inv()).get_pinv();
    }
    else
    {
      tag::IncrementalPose<tag::ConstantPosition::State> meas;
      meas.covariance.slice<3,3,3,3>() = SVD<3>(wls.get_C_inv()).get_pinv();
      meas.measurement.slice<3,3>() = v3Res;
      filter.filter(meas);

    }

    so3 = filter.state.pose.get_rotation();
  }

  //TooN::SE3<> se3Result;
  //se3Result.get_rotation() = so3;

  se3 = filter.state.pose;
  m6Cov = filter.state.covariance;
}
*/
