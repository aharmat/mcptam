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

//=========================================================================================
//
// Modifications
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/CalibCornerPatch.h>
#include <mcptam/OpenGL.h>
#include <mcptam/SmallMatrixOpts.h>
#include <TooN/helpers.h>
#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/image_interpolate.h>
#include <TooN/TooN.h>
#include <TooN/Cholesky.h>
#include <algorithm>

CVD::Image<float> CalibCornerPatch::mimSharedSourceTemplate;

CalibCornerPatch::CalibCornerPatch(int nSideSize, CVD::ImageRef irDrawOffset) : mirDrawOffset(irDrawOffset)
{
  mimTemplate.resize(CVD::ImageRef(nSideSize, nSideSize));
  mimGradients.resize(CVD::ImageRef(nSideSize, nSideSize));
  mimAngleJacs.resize(CVD::ImageRef(nSideSize, nSideSize));
  if (mimSharedSourceTemplate.size().x == 0)
  {
    MakeSharedTemplate();
  }
}

void CalibCornerPatch::MakeTemplateWithCurrentParams()
{
  double dBlurSigma = 2.0;
  int nExtraPixels = static_cast<int>(dBlurSigma * 6.0) + 2;

  CVD::Image<float> imToBlur(mimTemplate.size() + CVD::ImageRef(nExtraPixels, nExtraPixels));
  CVD::Image<float> imTwiceToBlur(imToBlur.size() * 2);

  // Make actual template:
  int nOffset;
  {
    TooN::Matrix<2> m2Warp = mParams.m2Warp();
    CVD::transform(mimSharedSourceTemplate, imTwiceToBlur, opts::M2Inverse(m2Warp),
                   vec(mimSharedSourceTemplate.size() - CVD::ImageRef(1, 1)) * 0.5,
                   vec(imTwiceToBlur.size() - CVD::ImageRef(1, 1)) * 0.5);
    CVD::halfSample(imTwiceToBlur, imToBlur);
    CVD::convolveGaussian(imToBlur, dBlurSigma);

    nOffset = (imToBlur.size().x - mimTemplate.size().x) / 2;
    CVD::copy(imToBlur, mimTemplate, mimTemplate.size(), CVD::ImageRef(nOffset, nOffset));
  }

  // Make numerical angle jac images:
  for (int dof = 0; dof < 2; dof++)
  {
    TooN::Matrix<2> m2Warp;
    for (int i = 0; i < 2; i++)
    {
      double dAngle = mParams.v2Angles[i];
      if (dof == i)
        dAngle += 0.01;
      m2Warp[0][i] = cos(dAngle);
      m2Warp[1][i] = sin(dAngle);
    };

    CVD::transform(mimSharedSourceTemplate, imTwiceToBlur, opts::M2Inverse(m2Warp),
                   vec(mimSharedSourceTemplate.size() - CVD::ImageRef(1, 1)) * 0.5,
                   vec(imTwiceToBlur.size() - CVD::ImageRef(1, 1)) * 0.5);
    CVD::halfSample(imTwiceToBlur, imToBlur);
    CVD::convolveGaussian(imToBlur, dBlurSigma);
    CVD::ImageRef ir;

    do
    {
      mimAngleJacs[ir][dof] = (imToBlur[ir + CVD::ImageRef(nOffset, nOffset)] - mimTemplate[ir]) / 0.01;
    }
    while (ir.next(mimTemplate.size()));
  }

  // Make the image of image gradients here too (while we have the bigger template to work from)
  CVD::ImageRef ir;
  do
  {
    mimGradients[ir][0] =
      0.5 * (imToBlur[ir + CVD::ImageRef(nOffset + 1, nOffset)] - imToBlur[ir + CVD::ImageRef(nOffset - 1, nOffset)]);
    mimGradients[ir][1] =
      0.5 * (imToBlur[ir + CVD::ImageRef(nOffset, nOffset + 1)] - imToBlur[ir + CVD::ImageRef(nOffset, nOffset - 1)]);
  }
  while (ir.next(mimGradients.size()));
}

bool CalibCornerPatch::IterateOnImageWithDrawing(CalibCornerPatch::Params &params, CVD::Image<CVD::byte> &im)
{
  bool bReturn = IterateOnImage(params, im);
  if (!bReturn)
  {
    glPointSize(3);
    glColor3f(1, 0, 0);
    glBegin(GL_POINTS);
    CVD::glVertex(params.v2Pos + vec(mirDrawOffset));
    glEnd();
  }
  return bReturn;
}

bool CalibCornerPatch::IterateOnImage(CalibCornerPatch::Params &params, CVD::Image<CVD::byte> &im)
{
  mParams = params;
  double dLastUpdate = 0.0;
  for (int i = 0; i < 20; i++)
  {
    MakeTemplateWithCurrentParams();
    dLastUpdate = Iterate(im);

    if (dLastUpdate < 0)
      return false;
    if (dLastUpdate < 0.00001)
      break;
  }

  if (dLastUpdate > 0.001)
    return false;
  if (fabs(sin(mParams.v2Angles[0] - mParams.v2Angles[1])) < sin(M_PI / 6.0))
  {
    return false;
  }
  if (fabs(mParams.dGain) < 20.0)
    return false;
  if (mdLastError > 25.0)
    return false;

  params = mParams;
  return true;
}

double CalibCornerPatch::Iterate(CVD::Image<CVD::byte> &im)
{
  TooN::Vector<2> v2TL = mParams.v2Pos - vec(mimTemplate.size() - CVD::ImageRef(1, 1)) / 2.0;
  if (!(v2TL[0] >= 0.0 && v2TL[1] >= 0.0))
    return -1.0;

  TooN::Vector<2> v2BR = v2TL + vec(mimTemplate.size() - CVD::ImageRef(1, 1));
  if (!(v2BR[0] < (im.size().x - 1.0) && v2BR[1] < (im.size().y - 1.0)))
    return -1.0;

  CVD::image_interpolate<CVD::Interpolate::Bilinear, CVD::byte> imInterp(im);
  TooN::Matrix<6> m6JTJ = TooN::Zeros;
  TooN::Vector<6> v6JTD = TooN::Zeros;

  CVD::ImageRef ir;
  double dSum = 0.0;
  do
  {
    TooN::Vector<2> v2Pos_Template = vec(ir) - vec(mimTemplate.size() - CVD::ImageRef(1, 1)) / 2.0;
    TooN::Vector<2> v2Pos_Image = mParams.v2Pos + v2Pos_Template;
    double dDiff = imInterp[v2Pos_Image] - (mParams.dGain * mimTemplate[ir] + mParams.dMean);
    dSum += fabs(dDiff);
    TooN::Vector<6> v6Jac;
    // Jac for center pos: Minus sign because +pos equates to sliding template -
    v6Jac.slice<0, 2>() = -1.0 * mParams.dGain * mimGradients[ir];
    // Jac for angles: dPos/dAngle needs finishing by multiplying by pos..
    v6Jac[2] = mimAngleJacs[ir][0] * mParams.dGain;
    v6Jac[3] = mimAngleJacs[ir][1] * mParams.dGain;
    // Jac for mean:
    v6Jac[4] = 1.0;
    // Jac for gain:
    v6Jac[5] = mimTemplate[ir];

    m6JTJ += v6Jac.as_col() * v6Jac.as_row();
    v6JTD += dDiff * v6Jac;
  }
  while (ir.next(mimTemplate.size()));

  TooN::Cholesky<6> chol(m6JTJ);
  TooN::Vector<6> v6Update = 0.7 * chol.backsub(v6JTD);
  mParams.v2Pos += v6Update.slice<0, 2>();
  mParams.v2Angles += v6Update.slice<2, 2>();
  mParams.dMean += v6Update[4];
  mParams.dGain += v6Update[5];
  mdLastError = dSum / mimTemplate.size().area();

  return sqrt(v6Update.slice<0, 2>() * v6Update.slice<0, 2>());
}

void CalibCornerPatch::MakeSharedTemplate()
{
  const int nSideSize = 100;
  const int nHalf = nSideSize / 2;

  mimSharedSourceTemplate.resize(CVD::ImageRef(nSideSize, nSideSize));

  CVD::ImageRef ir;

  do
  {
    float fX = (ir.x < nHalf) ? 1.0 : -1.0;
    float fY = (ir.y < nHalf) ? 1.0 : -1.0;
    mimSharedSourceTemplate[ir] = fX * fY;
  }
  while (ir.next(mimSharedSourceTemplate.size()));
}

CalibCornerPatch::Params::Params()
{
  v2Angles[0] = 0.0;
  v2Angles[1] = M_PI / 2.0;
  dMean = 0.0;
  dGain = 1.0;
  v3Pos = TooN::Zeros;
}

TooN::Matrix<2> CalibCornerPatch::Params::m2Warp()
{
  TooN::Matrix<2> m2Warp;
  for (int i = 0; i < 2; i++)
  {
    m2Warp[0][i] = cos(v2Angles[i]);
    m2Warp[1][i] = sin(v2Angles[i]);
  }

  return m2Warp;
}
