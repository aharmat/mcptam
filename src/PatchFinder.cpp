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

#include <mcptam/PatchFinder.h>
#include <mcptam/SmallMatrixOpts.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/MapPoint.h>
#include <mcptam/TaylorCamera.h>

#include <cvd/vision.h>
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>
#include <TooN/Cholesky.h>
#include <ros/assert.h>

int PatchFinder::snMaxSSDPerPixel = 250;  // Lower for normal operation (~250), higher for calibration (500)

#define CVD_HAVE_XMMINTRIN 1
// tmmintrin.h contains SSE3<> instrinsics, used for the ZMSSD search at the bottom..
// If this causes problems, just do #define CVD_HAVE_XMMINTRIN 0
#if CVD_HAVE_XMMINTRIN
#include <tmmintrin.h>
#endif

using namespace TooN;

PatchFinder::PatchFinder(int nPatchSize) : mimTemplate(CVD::ImageRef(nPatchSize, nPatchSize))
{
  mnPatchSize = nPatchSize;
  mirCenter = CVD::ImageRef(nPatchSize / 2, nPatchSize / 2);
  mnMaxSSD = mnPatchSize * mnPatchSize * PatchFinder::snMaxSSDPerPixel;
  // Populate the speed-up caches with bogus values:
  mm2LastWarpMatrix = 9999.9 * Identity;
  mpLastTemplateMapPoint = NULL;
}

// Find the warping matrix and search level
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint &point, SE3<> se3CFromW, Matrix<2> &m2CamDerivs)
{
  // Calc point pos in new view camera frame
  // Slightly dumb that we re-calculate this here when the tracker's already done this!
  Vector<3> v3Cam = se3CFromW * point.mv3WorldPos;
  // Project the source keyframe's one-pixel-right and one-pixel-down vectors into the current view
  Vector<3> v3MotionRight = se3CFromW.get_rotation() * point.mv3PixelRight_W;
  Vector<3> v3MotionDown = se3CFromW.get_rotation() * point.mv3PixelDown_W;
  // Calculate in-image derivatives of source image pixel motions:
  Vector<3> v3_dTheta, v3_dPhi;
  TaylorCamera::GetCamSphereDeriv(v3Cam, v3_dTheta, v3_dPhi);

  Vector<2> v2CamSphereMotionRight, v2CamSphereMotionDown;
  v2CamSphereMotionRight[0] = v3_dTheta * v3MotionRight;  // theta component
  v2CamSphereMotionRight[1] = v3_dPhi * v3MotionRight;    // phi component
  v2CamSphereMotionDown[0] = v3_dTheta * v3MotionDown;    // theta component
  v2CamSphereMotionDown[1] = v3_dPhi * v3MotionDown;      // phi component

  mm2WarpInverse.T()[0] = m2CamDerivs * v2CamSphereMotionRight;
  mm2WarpInverse.T()[1] = m2CamDerivs * v2CamSphereMotionDown;

  double dDet = mm2WarpInverse[0][0] * mm2WarpInverse[1][1] - mm2WarpInverse[0][1] * mm2WarpInverse[1][0];
  if (!std::isfinite(dDet))
  {
    ROS_FATAL_STREAM("pixel right w: " << point.mv3PixelRight_W);
    ROS_FATAL_STREAM("pixel down w: " << point.mv3PixelDown_W);
    ROS_FATAL_STREAM("v3MotionDown: " << v3MotionDown);
    ROS_FATAL_STREAM("v3MotionRight: " << v3MotionRight);
    ROS_BREAK();
  }

  mnSearchLevel = 0;

  // This warp matrix is likely not appropriate for finding at level zero, which is
  // the level at which it has been calculated. Vary the search level until the
  // at that level would be appropriate (does not actually modify the matrix.)
  while (dDet > 3 && mnSearchLevel < LEVELS - 1)
  {
    mnSearchLevel++;
    dDet *= 0.25;
  }

  // Some warps are inappropriate, e.g. too near the camera, too far, or reflected,
  // or zero area.. reject these!
  if (dDet > 3 || dDet < 0.5)  // dDet < 0.25)
  {
    mbTemplateBad = true;
    return -1;
  }
  else
    return mnSearchLevel;
}

// This is just a convenience function wich caluclates the warp matrix and generates
// the template all in one call.
void PatchFinder::MakeTemplateCoarse(MapPoint &point, SE3<> se3CFromW, Matrix<2> &m2CamDerivs)
{
  CalcSearchLevelAndWarpMatrix(point, se3CFromW, m2CamDerivs);
  MakeTemplateCoarseCont(point);
}

// This function generates the warped search template.
void PatchFinder::MakeTemplateCoarseCont(MapPoint &point)
{
  // Get the warping matrix appropriate for use with CVD::transform...
  Matrix<2> m2 = opts::M2Inverse(mm2WarpInverse) * LevelScale(mnSearchLevel);
  // m2 now represents the number of pixels in the source image for one
  // pixel of template image

  // Optimisation: Don't re-gen the coarse template if it's going to be substantially the
  // same as was made last time. This saves time when the camera is not moving. For this,
  // check that (a) this patchfinder is still working on the same map point and (b) the
  // warping matrix has not changed much.

  bool bNeedToRefreshTemplate = false;
  if (&point != mpLastTemplateMapPoint)
    bNeedToRefreshTemplate = true;
  // Still the same map point? Then compare warping matrix..
  for (int i = 0; !bNeedToRefreshTemplate && i < 2; i++)
  {
    Vector<2> v2Diff = m2.T()[i] - mm2LastWarpMatrix.T()[i];
    const double dRefreshLimit = 0.07;  // Sort of works out as half a pixel displacement in src img
    if (v2Diff * v2Diff > dRefreshLimit * dRefreshLimit)
      bNeedToRefreshTemplate = true;
  }

  // Need to regen template? Then go ahead.
  if (bNeedToRefreshTemplate)
  {
    int nOutside;  // Use CVD::transform to warp the patch according the the warping matrix m2
    // This returns the number of pixels outside the source image hit, which should be zero.
    nOutside = CVD::transform(point.mpPatchSourceKF->maLevels[point.mnSourceLevel].image, mimTemplate, m2,
                              vec(point.mirCenter), vec(mirCenter));

    if (nOutside)
      mbTemplateBad = true;
    else
      mbTemplateBad = false;

    MakeTemplateSums();

    // Store the parameters which allow us to determine if we need to re-calculate
    // the patch next time round.
    mpLastTemplateMapPoint = &point;
    mm2LastWarpMatrix = m2;

    if (!TemplateBad())
      MakeSubPixTemplate();
  }
}

// This makes a template without warping. Used for epipolar search, where we don't really know
// what the warping matrix should be. (Although to be fair, I should do rotation for epipolar,
// which we could approximate without knowing patch depth!)
void PatchFinder::MakeTemplateCoarseNoWarp(KeyFrame &kf, int nLevel, CVD::ImageRef irLevelPos)
{
  mnSearchLevel = nLevel;
  CVD::Image<CVD::byte> &image = kf.maLevels[nLevel].image;
  if (!image.in_image_with_border(irLevelPos, mnPatchSize / 2 + 1))
  {
    mbTemplateBad = true;
    return;
  }
  mbTemplateBad = false;
  CVD::copy(image, mimTemplate, mimTemplate.size(), irLevelPos - mirCenter);
  MakeTemplateSums();
}

// Convenient wrapper for the above
void PatchFinder::MakeTemplateCoarseNoWarp(MapPoint &point)
{
  MakeTemplateCoarseNoWarp(*point.mpPatchSourceKF, point.mnSourceLevel, point.mirCenter);
}

// Finds the sum, and sum-squared, of template pixels. These sums are used
// to calculate the ZMSSD.
inline void PatchFinder::MakeTemplateSums()
{
  int nSum = 0;
  int nSumSq = 0;
  CVD::ImageRef ir;
  do
  {
    int b = mimTemplate[ir];
    nSum += b;
    nSumSq += b * b;
  } while (ir.next(mimTemplate.size()));
  mnTemplateSum = nSum;
  mnTemplateSumSq = nSumSq;
}

// One of the main functions of the class! Looks at the appropriate level of
// the target keyframe to try and find the template. Looks only at FAST corner points
// which are within radius nRange of the center. (Params are supplied in Level0
// coords.) Returns true on patch found.
bool PatchFinder::FindPatchCoarse(CVD::ImageRef irPos, KeyFrame &kf, unsigned int nRange, int &nScore, bool bExhaustive)
{
  mbFound = false;

  // Convert from L0 coords to search level quantities
  int nLevelScale = LevelScale(mnSearchLevel);
  mirPredictedPos = irPos;
  irPos = irPos / nLevelScale;
  nRange = (nRange + nLevelScale - 1) / nLevelScale;

  // Bounding box of search circle
  int nTop = irPos.y - nRange;
  int nBottomPlusOne = irPos.y + nRange + 1;
  int nLeft = irPos.x - nRange;
  int nRight = irPos.x + nRange;

  // Ref variable for the search level
  Level &L = kf.maLevels[mnSearchLevel];

  // Some bounds checks on the bounding box..
  if (nTop < 0)
    nTop = 0;
  if (nTop >= L.image.size().y)
    return false;
  if (nBottomPlusOne <= 0)
    return false;

  if (nLeft < 0)
    nLeft = 0;
  if (nLeft >= L.image.size().x)
    return false;

  CVD::ImageRef irBest;         // Best match so far
  int nBestSSD = mnMaxSSD + 1;  // Best score so far is beyond the max allowed

  int nValidCorners = 0;

  if (bExhaustive)
  {
    CVD::ImageRef irLoop;
    for (irLoop.y = nTop; irLoop.y < nBottomPlusOne && irLoop.y < L.image.size().y; irLoop.y++)
    {
      for (irLoop.x = nLeft; irLoop.x <= nRight && irLoop.x < L.image.size().x; irLoop.x++)
      {
        if ((irPos - irLoop).mag_squared() > nRange * nRange)
          continue;  // ... reject all those not close enough..

        nValidCorners++;

        int nSSD;  // .. and find the ZMSSD at those near enough.
        nSSD = ZMSSDAtPoint(L.image, irLoop);

        // std::cout<<"Predicted: "<<irPos<<"  Searching: "<<irLoop<<std::endl;
        // std::cout<<"SSD: "<<nSSD<<std::endl;

        if (nSSD < nBestSSD)  // Best yet?
        {
          irBest = irLoop;
          nBestSSD = nSSD;
        }
      }
    }
  }
  else
  {
    // The next section finds all the FAST corners in the target level which
    // are near enough the search center. It's a bit optimised to use
    // a corner row look-up-table, since otherwise the routine
    // would spend a long time trawling throught the whole list of FAST corners!
    std::vector<CVD::ImageRef>::iterator i;
    std::vector<CVD::ImageRef>::iterator i_end;

    i = L.vCorners.begin() + L.vCornerRowLUT[nTop];

    if (nBottomPlusOne >= L.image.size().y)
      i_end = L.vCorners.end();
    else
      i_end = L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];

    for (; i < i_end; i++)  // For each corner ...
    {
      if (i->x < nLeft || i->x > nRight)
        continue;
      if ((irPos - *i).mag_squared() > nRange * nRange)
        continue;  // ... reject all those not close enough..

      nValidCorners++;

      int nSSD;  // .. and find the ZMSSD at those near enough.
      nSSD = ZMSSDAtPoint(L.image, *i);
      /*
      SetSubPixPos(LevelZeroPos(*i, mnSearchLevel));
      bool bSubPixConverges=IterateSubPixToConvergence(kf, 2, 0.25);

      if(!bSubPixConverges)
        continue;
      */

      // cout<<"Predicted: "<<irPos<<"  Searching: "<<*i<<endl;
      // cout<<"SSD: "<<nSSD<<endl;

      if (nSSD < nBestSSD)  // Best yet?
      {
        irBest = *i;
        nBestSSD = nSSD;
      }
    }  // done looping over corners
  }

  // std::cout<<"Best SSD: "<<nBestSSD<<std::endl;
  // std::cout<<"Finder valid corners: "<<nValidCorners<<std::endl;

  nScore = nBestSSD;

  if (nBestSSD < mnMaxSSD)  // Found a valid match?
  {
    mv2CoarsePos = LevelZeroPos(irBest, mnSearchLevel);
    mbFound = true;
  }
  else
  {
    mbFound = false;
  }
  return mbFound;
}

// Makes an inverse composition template out of the coarse template.
// Includes calculating image of derivatives (gradients.) The inverse composition
// used here operates on three variables: x offet, y offset, and difference in patch
// means; hence things like mm3HInv are dim 3, but the trivial mean jacobian
// (always unity, for each pixel) is not stored.
void PatchFinder::MakeSubPixTemplate()
{
  mimJacs.resize(mimTemplate.size() - CVD::ImageRef(2, 2));
  Matrix<3> m3H = Zeros;  // This stores jTj.
  CVD::ImageRef ir;
  for (ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
  {
    for (ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
    {
      Vector<2> v2Grad;
      v2Grad[0] = 0.5 * (mimTemplate[ir + CVD::ImageRef(1, 0)] - mimTemplate[ir - CVD::ImageRef(1, 0)]);
      v2Grad[1] = 0.5 * (mimTemplate[ir + CVD::ImageRef(0, 1)] - mimTemplate[ir - CVD::ImageRef(0, 1)]);
      mimJacs[ir - CVD::ImageRef(1, 1)].first = v2Grad[0];
      mimJacs[ir - CVD::ImageRef(1, 1)].second = v2Grad[1];
      Vector<3> v3Grad = unproject(v2Grad);      // This adds the mean-difference jacobian..
      m3H += v3Grad.as_col() * v3Grad.as_row();  // Populate JTJ.
    }
  }

  // Invert JTJ..
  Cholesky<3> chol(m3H);
  mm3HInv = chol.get_inverse();
  // TOON2 Does not have a get_rank for cholesky
  // int nRank = chol.get_rank();
  // if(nRank < 3)
  // cout << "BAD RANK IN MAKESUBPIXELTEMPLATE!!!!" << endl; // This does not happen often (almost never!)

  mv2SubPixPos = mv2CoarsePos;  // Start the sub-pixel search at the result of the coarse search..
  mdMeanDiff = 0.0;
}

// Iterate inverse composition until convergence. Since it should never have
// to travel more than a pixel's distance, set a max number of iterations;
// if this is exceeded, consider the IC to have failed.
bool PatchFinder::IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts, double dConvLimit)
{
  // const double dConvLimit = 0.03;
  bool bConverged = false;
  int nIts;
  for (nIts = 0; nIts < nMaxIts && !bConverged; nIts++)
  {
    double dUpdateSquared = IterateSubPix(kf);
    if (dUpdateSquared < 0)  // went off edge of image
      return false;
    if (dUpdateSquared < dConvLimit * dConvLimit)
      return true;
  }
  return false;
}

// Single iteration of inverse composition. This compares integral image positions in the
// template image to floating point positions in the target keyframe. Interpolation is
// bilinear, and performed manually (rather than using CVD::image_interpolate) since
// this is a special case where the mixing fractions for each pixel are identical.
double PatchFinder::IterateSubPix(KeyFrame &kf)
{
  // Search level pos of patch center
  Vector<2> v2Center = LevelNPos(mv2SubPixPos, mnSearchLevel);
  CVD::BasicImage<CVD::byte> &image = kf.maLevels[mnSearchLevel].image;
  if (!image.in_image_with_border(CVD::ir_rounded(v2Center), mnPatchSize / 2 + 1))
    return -1.0;  // Negative return value indicates off edge of image

  // Position of top-left corner of patch in search level
  Vector<2> v2Base = v2Center - vec(mirCenter);

  // I.C. JT*d accumulator
  Vector<3> v3Accum = Zeros;

  CVD::ImageRef ir;

  CVD::byte *pTopLeftPixel;

  // Each template pixel will be compared to an interpolated target pixel
  // The target value is made using bilinear interpolation as the weighted sum
  // of four target image pixels. Calculate mixing fractions:
  double dX = v2Base[0] - floor(v2Base[0]);  // Distances from pixel center of TL pixel
  double dY = v2Base[1] - floor(v2Base[1]);
  float fMixTL = (1.0 - dX) * (1.0 - dY);
  float fMixTR = (dX) * (1.0 - dY);
  float fMixBL = (1.0 - dX) * (dY);
  float fMixBR = (dX) * (dY);

  // Loop over template image
  unsigned long nRowOffset =
      &kf.maLevels[mnSearchLevel].image[CVD::ImageRef(0, 1)] - &kf.maLevels[mnSearchLevel].image[CVD::ImageRef(0, 0)];
  for (ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
  {
    pTopLeftPixel = &image[CVD::ir(v2Base) + CVD::ImageRef(1, ir.y)];  // n.b. the x=1 offset, as with y
    for (ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
    {
      // Calc target interpolated pixel
      float fPixel = fMixTL * pTopLeftPixel[0] + fMixTR * pTopLeftPixel[1] + fMixBL * pTopLeftPixel[nRowOffset] +
                     fMixBR * pTopLeftPixel[nRowOffset + 1];

      pTopLeftPixel++;
      double dDiff = fPixel - mimTemplate[ir] + mdMeanDiff;
      v3Accum[0] += dDiff * mimJacs[ir - CVD::ImageRef(1, 1)].first;
      v3Accum[1] += dDiff * mimJacs[ir - CVD::ImageRef(1, 1)].second;
      v3Accum[2] += dDiff;  // Update JT*d
    }
  }

  // All done looping over image - find JTJ^-1 * JTd:
  Vector<3> v3Update = mm3HInv * v3Accum;
  mv2SubPixPos -= v3Update.slice<0, 2>() * LevelScale(mnSearchLevel);
  mdMeanDiff -= v3Update[2];

  double dPixelUpdateSquared = v3Update.slice<0, 2>() * v3Update.slice<0, 2>();
  return dPixelUpdateSquared;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
//
//
//              ZMSSDatpoint, which is SSE optimised, follows
//
// The top version is the SSE version for 8x8 patches. It is compiled
// only if CVD_HAVE_XMMINTRIN is true, also you need to give your
// compiler the appropriate flags (e.g. -march=core2 -msse3 for g++.)
// The standard c++ version, which is about half as quick (not a disaster
// by any means) is below.
//
// The 8x8 SSE version looks long because it has been unrolled,
// it just does the same thing eight times. Both versions are one-pass
// and need pre-calculated template sums and sum-squares.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#if CVD_HAVE_XMMINTRIN
// Horizontal sum of uint16s stored in an XMM register
inline int SumXMM_16(__m128i &target)
{
  unsigned short int sums_store[8];
  _mm_storeu_si128((__m128i *)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] + sums_store[4] + sums_store[5] + sums_store[6] +
         sums_store[7];
}
// Horizontal sum of uint32s stored in an XMM register
inline int SumXMM_32(__m128i &target)
{
  unsigned int sums_store[4];
  _mm_storeu_si128((__m128i *)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif

// Calculate the Zero-mean SSD of the coarse patch and a target imate at a specific
// point.
int PatchFinder::ZMSSDAtPoint(CVD::BasicImage<CVD::byte> &image, const CVD::ImageRef &ir)
{
  if (!image.in_image_with_border(ir, mirCenter[0]))
    return mnMaxSSD + 1;

  CVD::ImageRef irImgBase = ir - mirCenter;
  CVD::byte *imagepointer;
  CVD::byte *templatepointer;

  int nImageSumSq = 0;
  int nImageSum = 0;
  int nCrossSum = 0;

#if CVD_HAVE_XMMINTRIN
  if (mnPatchSize == 8)
  {
    long unsigned int imagepointerincrement;

    __m128i xImageAsEightBytes;
    __m128i xImageAsWords;
    __m128i xTemplateAsEightBytes;
    __m128i xTemplateAsWords;
    __m128i xZero;
    __m128i xImageSums;    // These sums are 8xuint16
    __m128i xImageSqSums;  // These sums are 4xint32
    __m128i xCrossSums;    // These sums are 4xint32
    __m128i xProduct;

    xImageSums = _mm_setzero_si128();
    xImageSqSums = _mm_setzero_si128();
    xCrossSums = _mm_setzero_si128();
    xZero = _mm_setzero_si128();

    imagepointer = &image[irImgBase + CVD::ImageRef(0, 0)];
    templatepointer = &mimTemplate[CVD::ImageRef(0, 0)];
    imagepointerincrement = &image[irImgBase + CVD::ImageRef(0, 1)] - imagepointer;

    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
    templatepointer += 16;
    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
    templatepointer += 16;
    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
    templatepointer += 16;
    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    imagepointer += imagepointerincrement;
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsEightBytes = _mm_load_si128((__m128i *)templatepointer);
    templatepointer += 16;
    xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
    xImageAsEightBytes = _mm_loadl_epi64((__m128i *)imagepointer);
    xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
    xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
    xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
    xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
    xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
    xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
    xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

    nImageSum = SumXMM_16(xImageSums);
    nCrossSum = SumXMM_32(xCrossSums);
    nImageSumSq = SumXMM_32(xImageSqSums);
  }
  else
#endif
  {
    for (int nRow = 0; nRow < mnPatchSize; nRow++)
    {
      imagepointer = &image[irImgBase + CVD::ImageRef(0, nRow)];
      templatepointer = &mimTemplate[CVD::ImageRef(0, nRow)];
      for (int nCol = 0; nCol < mnPatchSize; nCol++)
      {
        int n = imagepointer[nCol];
        nImageSum += n;
        nImageSumSq += n * n;
        nCrossSum += n * templatepointer[nCol];
      }
    }
  }

  int SA = mnTemplateSum;
  int SB = nImageSum;

  int N = mnPatchSize * mnPatchSize;
  return ((2 * SA * SB - SA * SA - SB * SB) / N + nImageSumSq + mnTemplateSumSq - 2 * nCrossSum);
}
