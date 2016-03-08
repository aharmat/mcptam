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
// Parts of the code are from PTAM, which is
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/CalibImageTaylor.h>
#include <mcptam/OpenGL.h>
#include <mcptam/CalibCornerPatch.h>
#include <mcptam/TaylorCamera.h>
#include <mcptam/GLWindow2.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/fast_corner.h>
#include <cvd/image_interpolate.h>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/wls.h>
#include <unsupported/Eigen/Polynomials>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <utility>

// Static members
int CalibImageTaylor::snCornerPatchSize = 20;
double CalibImageTaylor::sdBlurSigma = 1.0;
int CalibImageTaylor::snMeanGate = 20;
int CalibImageTaylor::snMinCornersForGrabbedImage = 20;
double CalibImageTaylor::sdExpandByStepMaxDistFrac = 0.3;

inline bool IsCorner(CVD::Image<CVD::byte>& im, CVD::ImageRef ir, int nGate)
{
  // Does a quick check to see if a point in an image could be a grid corner.
  // Does this by going around a 16-pixel ring, and checking that there's four
  // transitions (black - white- black - white - )
  // Also checks that the central pixel is blurred.

  // Find the mean intensity of the pixel ring...
  int nSum = 0;
  static CVD::byte abPixels[16];
  for (int i = 0; i < 16; i++)
  {
    abPixels[i] = im[ir + CVD::fast_pixel_ring[i]];
    nSum += abPixels[i];
  }

  int nMean = nSum / 16;
  int nHiThresh = nMean + nGate;
  int nLoThresh = nMean - nGate;

  // If the center pixel is roughly the same as the mean, this isn't a corner.
  int nCenter = im[ir];
  if (nCenter <= nLoThresh || nCenter >= nHiThresh)
    return false;

  // Count transitions around the ring... there should be four!
  bool bState = (abPixels[15] > nMean);
  int nSwaps = 0;
  for (int i = 0; i < 16; i++)
  {
    CVD::byte bValNow = abPixels[i];
    if (bState)
    {
      if (bValNow < nLoThresh)
      {
        bState = false;
        nSwaps++;
      }
    }
    else
    {
      if (bValNow > nHiThresh)
      {
        bState = true;
        nSwaps++;
      }
    }
  }
  return (nSwaps == 4);
}

inline TooN::Vector<2> GuessInitialAngles(CVD::Image<CVD::byte>& im, CVD::ImageRef irCenter)
{
  // The iterative patch-finder works better if the initial guess
  // is roughly aligned! Find one of the line-axes by searching round
  // the circle for the strongest gradient, and use that and +90deg as the
  // initial guesses for patch angle.
  //
  // Yes, this is a very poor estimate, but it's generally (hopefully?)
  // enough for the iterative finder to converge.

  CVD::image_interpolate<CVD::Interpolate::Bilinear, CVD::byte> imInterp(im);
  double dBestAngle = 0;
  double dBestGradMag = 0;
  double dGradAtBest = 0;
  for (double dAngle = 0.0; dAngle < M_PI; dAngle += 0.1)
  {
    TooN::Vector<2> v2Dirn;
    v2Dirn[0] = cos(dAngle);
    v2Dirn[1] = sin(dAngle);
    TooN::Vector<2> v2Perp;
    v2Perp[1] = -v2Dirn[0];
    v2Perp[0] = v2Dirn[1];

    double dG = imInterp[CVD::vec(irCenter) + v2Dirn * 3.0 + v2Perp * 0.1] -
                imInterp[CVD::vec(irCenter) + v2Dirn * 3.0 - v2Perp * 0.1] +
                imInterp[CVD::vec(irCenter) - v2Dirn * 3.0 - v2Perp * 0.1] -
                imInterp[CVD::vec(irCenter) - v2Dirn * 3.0 + v2Perp * 0.1];

    if (fabs(dG) > dBestGradMag)
    {
      dBestGradMag = fabs(dG);
      dGradAtBest = dG;
      dBestAngle = dAngle;
    }
  }

  TooN::Vector<2> v2Ret;
  if (dGradAtBest < 0)
  {
    v2Ret[0] = dBestAngle;
    v2Ret[1] = dBestAngle + M_PI / 2.0;
  }
  else
  {
    v2Ret[1] = dBestAngle;
    v2Ret[0] = dBestAngle - M_PI / 2.0;
  }

  return v2Ret;
}

CalibImageTaylor::CalibImageTaylor(CVD::ImageRef irDrawOffset, GLWindow2* pWindow, TaylorCamera* pCamera,
                                   CVD::Image<CVD::byte> imMask)
  : mirDrawOffset(irDrawOffset), mpGLWindow(pWindow), mpCamera(pCamera), mimMask(imMask)
{
}

// Finds checkerboard in image, optionally orders the found corners according to the size/orientation of the
// checkerboard
bool CalibImageTaylor::MakeFromImage(CVD::Image<CVD::byte>& im, CVD::ImageRef irPatternSize, bool bRadial)
{
  mvCorners.clear();
  mvGridCorners.clear();

  mImage = im;
  mImage.make_unique();

  // Find potential corners..
  // This works better on a blurred image, so make a blurred copy
  // and run the corner finding on that.

  CVD::Image<CVD::byte> imBlurred = mImage;
  imBlurred.make_unique();
  CVD::convolveGaussian(imBlurred, CalibImageTaylor::sdBlurSigma);
  CVD::ImageRef irTopLeft(5, 5);
  CVD::ImageRef irBotRight = mImage.size() - irTopLeft;
  CVD::ImageRef irCurr = irTopLeft;
  glPointSize(1);
  glColor3f(1, 0, 1);
  glBegin(GL_POINTS);
  int nGate = CalibImageTaylor::snMeanGate;
  do
  {
    if (mimMask.totalsize() > 0 && mimMask[irCurr] < 127)
      continue;

    if (IsCorner(imBlurred, irCurr, nGate))
    {
      mvCorners.push_back(irCurr);
      CVD::glVertex(irCurr + mirDrawOffset);
    }
  }
  while (irCurr.next(irTopLeft, irBotRight));
  glEnd();

  // If there's not enough corners, i.e. camera pointing somewhere random, abort.
  if (static_cast<int>(mvCorners.size()) < CalibImageTaylor::snMinCornersForGrabbedImage)
    return false;

  // Pick a central corner point...
  // CVD::ImageRef irCenterOfImage = mImage.size()  / 2;

  // Pick central corner to be median of found corners
  std::vector<int> vXCoords, vYCoords;
  vXCoords.reserve(mvCorners.size());
  vYCoords.reserve(mvCorners.size());

  for (unsigned i = 0; i < mvCorners.size(); ++i)
  {
    vXCoords.push_back(mvCorners[i].x);
    vYCoords.push_back(mvCorners[i].y);
  }

  std::sort(vXCoords.begin(), vXCoords.end());
  std::sort(vYCoords.begin(), vYCoords.end());

  CVD::ImageRef irCenterOfImage(vXCoords[vXCoords.size() / 2], vYCoords[vYCoords.size() / 2]);

  glPointSize(6);
  glColor3f(0, 0, 1);
  glBegin(GL_POINTS);
  CVD::glVertex(irCenterOfImage + mirDrawOffset);
  glEnd();

  CVD::ImageRef irBestCenterPos;
  unsigned int nBestDistSquared = 99999999;
  for (unsigned int i = 0; i < mvCorners.size(); i++)
  {
    unsigned int nDist = (mvCorners[i] - irCenterOfImage).mag_squared();
    if (nDist < nBestDistSquared)
    {
      nBestDistSquared = nDist;
      irBestCenterPos = mvCorners[i];
    }
  }

  // ... and try to fit a corner-patch to that.
  CalibCornerPatch patch(CalibImageTaylor::snCornerPatchSize, mirDrawOffset);
  CalibCornerPatch::Params params;
  params.v2Pos = CVD::vec(irBestCenterPos);
  params.v2Angles = GuessInitialAngles(mImage, irBestCenterPos);
  params.dGain = 80.0;
  params.dMean = 120.0;

  if (!patch.IterateOnImageWithDrawing(params, mImage))
    return false;

  // The first found corner patch becomes the origin of the detected grid.
  CalibGridCorner cFirst(mirDrawOffset);
  cFirst.mParams = params;
  mvGridCorners.push_back(cFirst);
  cFirst.Draw();

  // Next, go in two compass directions from the origin patch, and see if
  // neighbors can be found.
  if (!(ExpandByAngle(0, 0) || ExpandByAngle(0, 2)))
    return false;
  if (!(ExpandByAngle(0, 1) || ExpandByAngle(0, 3)))
    return false;

  mvGridCorners[1].mm2InheritedSteps = mvGridCorners[2].mm2InheritedSteps = mvGridCorners[0].GetSteps(mvGridCorners);

  if (mpCamera)
  {
    for (int i = 0; i < 3; ++i)
    {
      mvGridCorners[i].mParams.v3Pos = mpCamera->UnProject(mvGridCorners[i].mParams.v2Pos);
    }

    for (int i = 1; i < 3; ++i)
    {
      mvGridCorners[i].mm23InheritedSteps = mvGridCorners[0].GetSteps3D(mvGridCorners);
    }
  }

  // The three initial grid elements are enough to find the rest of the grid.
  int nNext;
  int nSanityCounter = 0;  // Stop it getting stuck in an infinite loop...
  const int nSanityCounterLimit = 500;
  while ((nNext = NextToExpand()) >= 0 && nSanityCounter < nSanityCounterLimit)
  {
    ExpandByStep(nNext);
    nSanityCounter++;
  }

  if (nSanityCounter == nSanityCounterLimit)
    return false;

  // Need to check x axis of the first corner (relative to image) to see if grid coordinates need to be rotated
  // We want x axis of the grid to be along x axis of image
  double dXAxisAngle = cFirst.mParams.v2Angles[0];

  if (bRadial)
  {
    // Figure out the angle of the radial vector at this location
    TooN::Vector<2> v2Radial = cFirst.mParams.v2Pos - TooN::makeVector(im.size().x / 2.0, im.size().y / 2.0);
    double dRadialAngle = atan2(v2Radial[1], v2Radial[0]);
    dXAxisAngle -= dRadialAngle;
  }

  TooN::Matrix<2> m2GridRot;
  int nDirnShift;

  if (dXAxisAngle <= M_PI / 4 && dXAxisAngle > -M_PI / 4)
  {
    m2GridRot[0][0] = 1;
    m2GridRot[0][1] = 0;
    m2GridRot[1][0] = 0;
    m2GridRot[1][1] = 1;

    nDirnShift = 0;
    ROS_INFO("Rotating by 0 degrees");
  }
  else if (dXAxisAngle <= 3 * M_PI / 4 && dXAxisAngle > M_PI / 4)  // rotate +90 deg
  {
    m2GridRot[0][0] = 0;
    m2GridRot[0][1] = -1;
    m2GridRot[1][0] = 1;
    m2GridRot[1][1] = 0;

    nDirnShift = 1;
    ROS_INFO("Rotating by 90 degrees");
  }
  else if (dXAxisAngle <= -M_PI / 4 && dXAxisAngle > -3 * M_PI / 4)  // rotate -90 deg
  {
    m2GridRot[0][0] = 0;
    m2GridRot[0][1] = 1;
    m2GridRot[1][0] = -1;
    m2GridRot[1][1] = 0;

    nDirnShift = 3;
    ROS_INFO("Rotating by -90 degrees");
  }
  else  // rotate 180 deg
  {
    m2GridRot[0][0] = -1;
    m2GridRot[0][1] = 0;
    m2GridRot[1][0] = 0;
    m2GridRot[1][1] = -1;

    nDirnShift = 2;
    ROS_INFO("Rotating by 180 degrees");
  }

  CVD::ImageRef irMinPos(1000, 1000);
  CVD::ImageRef irMaxPos(-1000, -1000);

  // Go through corners, rotate the grid pos and find min/max components
  for (unsigned i = 0; i < mvGridCorners.size(); ++i)
  {
    CVD::ImageRef& irGridPos = mvGridCorners[i].mirGridPos;
    TooN::Vector<2> v2GridPosVec = m2GridRot * CVD::vec(irGridPos);
    irGridPos = CVD::ir(v2GridPosVec);

    CalibGridCorner::NeighborState aNewNeighborStates[4];
    for (int dirn = 0; dirn < 4; dirn++)
    {
      int nNewDirn = (dirn + nDirnShift) % 4;
      aNewNeighborStates[nNewDirn] = mvGridCorners[i].maNeighborStates[dirn];
    }

    std::copy(aNewNeighborStates, aNewNeighborStates + 4, mvGridCorners[i].maNeighborStates);

    if (irGridPos[0] < irMinPos[0])
      irMinPos[0] = irGridPos[0];
    if (irGridPos[1] < irMinPos[1])
      irMinPos[1] = irGridPos[1];

    if (irGridPos[0] > irMaxPos[0])
      irMaxPos[0] = irGridPos[0];
    if (irGridPos[1] > irMaxPos[1])
      irMaxPos[1] = irGridPos[1];
  }

  // Shift grid points by minPos so that (0,0) will be the top left corner point
  for (unsigned i = 0; i < mvGridCorners.size(); ++i)
  {
    mvGridCorners[i].mirGridPos -= irMinPos;
  }

  ROS_INFO_STREAM("irMinPos: " << irMinPos << "   irMaxPos: " << irMaxPos);

  // If we weren't given a valid pattern, then we can just stop here because we don't need to check
  // pattern size or reorder points
  if (irPatternSize == CVD::ImageRef())
  {
    // Draw the grid before leaving. Do this here instead of before the "if" statement because
    // we'll modify the grid coordinates after this block, and the drawn grid should reflect that
    DrawImageGrid();
    return true;
  }

  CVD::ImageRef irSpan = irMaxPos - irMinPos + CVD::ImageRef(1, 1);
  if (irSpan != irPatternSize)
  {
    ROS_WARN_STREAM("Grid size not equal pattern size! Grid size: " << irSpan << " Pattern size: " << irPatternSize);
  }
  else
  {
    DrawImageGrid();
  }

  return irSpan == irPatternSize;
}

// Try to find a new checkerboard corner by stepping from an already-found corner, based on the given direction
bool CalibImageTaylor::ExpandByAngle(int nSrc, int nDirn)
{
  CalibGridCorner& gSrc = mvGridCorners[nSrc];

  CVD::ImageRef irBest;
  double dBestDist = 99999;
  TooN::Vector<2> v2TargetDirn = gSrc.mParams.m2Warp().T()[nDirn % 2];
  if (nDirn >= 2)
    v2TargetDirn *= -1;
  for (unsigned int i = 0; i < mvCorners.size(); i++)
  {
    TooN::Vector<2> v2Diff = CVD::vec(mvCorners[i]) - gSrc.mParams.v2Pos;
    if (v2Diff * v2Diff < 100)
      continue;

    if (v2Diff * v2Diff > dBestDist * dBestDist)
      continue;

    TooN::Vector<2> v2Dirn = v2Diff;
    normalize(v2Dirn);
    if (v2Dirn * v2TargetDirn < cos(M_PI / 18.0))
      continue;

    dBestDist = sqrt(v2Diff * v2Diff);
    irBest = mvCorners[i];
  }

  CalibGridCorner gTarget(mirDrawOffset);
  gTarget.mParams = gSrc.mParams;
  gTarget.mParams.v2Pos = CVD::vec(irBest);
  gTarget.mParams.dGain *= -1;

  CalibCornerPatch patch(CalibImageTaylor::snCornerPatchSize, mirDrawOffset);
  if (!patch.IterateOnImageWithDrawing(gTarget.mParams, mImage))
  {
    gSrc.maNeighborStates[nDirn].val = N_FAILED;
    return false;
  }

  gTarget.mirGridPos = gSrc.mirGridPos;
  if (nDirn < 2)
    gTarget.mirGridPos[nDirn]++;
  else
    gTarget.mirGridPos[nDirn % 2]--;

  // Update connection states:
  mvGridCorners.push_back(gTarget);  // n.b. This invalidates gSrc!
  mvGridCorners.back().maNeighborStates[(nDirn + 2) % 4].val = nSrc;
  mvGridCorners[nSrc].maNeighborStates[nDirn].val = mvGridCorners.size() - 1;

  mvGridCorners.back().Draw();
  return true;
}

// Finds the index of the current checkerboard corner with the best chance of finding a neighbor
int CalibImageTaylor::NextToExpand()
{
  int nBest = -1;
  double dBest = 0.0;

  for (unsigned int i = 0; i < mvGridCorners.size(); i++)
  {
    double d = mvGridCorners[i].ExpansionPotential();
    if (d > dBest)
    {
      nBest = i;
      dBest = d;
    }
  }
  return nBest;
}

// Find a new checkerboard corner from an already-found corner, by choosing the best expansion direction
bool CalibImageTaylor::ExpandByStep(int n)
{
  CalibGridCorner& gSrc = mvGridCorners[n];

  // First, choose which direction to expand in...
  // Ideally, choose a dirn for which the Step calc is good!
  int nDirn = -10;
  for (int i = 0; nDirn == -10 && i < 4; i++)
  {
    if (gSrc.maNeighborStates[i].val == N_NOT_TRIED && gSrc.maNeighborStates[(i + 2) % 4].val >= 0)
      nDirn = i;
  }

  if (nDirn == -10)
  {
    for (int i = 0; nDirn == -10 && i < 4; i++)
    {
      if (gSrc.maNeighborStates[i].val == N_NOT_TRIED)
        nDirn = i;
    }
  }

  ROS_ASSERT(nDirn != -10);

  CVD::ImageRef irGridStep = IR_from_dirn(nDirn);
  TooN::Vector<2> v2SearchPos;
  TooN::Vector<2> v2Step;

  if (mpCamera)
  {
    TooN::Vector<3> v3Step = gSrc.GetSteps3D(mvGridCorners).T() * CVD::vec(irGridStep);
    ROS_ASSERT(gSrc.mParams.v3Pos != TooN::Zeros);
    v2SearchPos = mpCamera->Project(gSrc.mParams.v3Pos + v3Step);
    v2Step = v2SearchPos - gSrc.mParams.v2Pos;  // the equivalent v2step
  }
  else
  {
    v2Step = gSrc.GetSteps(mvGridCorners).T() * CVD::vec(irGridStep);
    v2SearchPos = gSrc.mParams.v2Pos + v2Step;
  }

  // Before the search: pre-fill the failure result for easy returns.
  gSrc.maNeighborStates[nDirn].val = N_FAILED;

  CVD::ImageRef irBest;
  double dBestDist = 99999;
  for (unsigned int i = 0; i < mvCorners.size(); i++)
  {
    TooN::Vector<2> v2Diff = CVD::vec(mvCorners[i]) - v2SearchPos;
    if (v2Diff * v2Diff > dBestDist * dBestDist)
      continue;

    dBestDist = sqrt(v2Diff * v2Diff);
    irBest = mvCorners[i];
  }

  double dStepDist = sqrt(v2Step * v2Step);
  if (dBestDist > CalibImageTaylor::sdExpandByStepMaxDistFrac * dStepDist)
    return false;

  CalibGridCorner gTarget(mirDrawOffset);
  gTarget.mParams = gSrc.mParams;
  gTarget.mParams.v2Pos = CVD::vec(irBest);
  gTarget.mParams.dGain *= -1;
  gTarget.mirGridPos = gSrc.mirGridPos + irGridStep;
  gTarget.mm2InheritedSteps = gSrc.GetSteps(mvGridCorners);
  CalibCornerPatch patch(CalibImageTaylor::snCornerPatchSize);
  if (!patch.IterateOnImageWithDrawing(gTarget.mParams, mImage))
    return false;

  if (mpCamera)
  {
    gTarget.mParams.v3Pos = mpCamera->UnProject(gTarget.mParams.v2Pos);
    gTarget.mm23InheritedSteps = gSrc.GetSteps3D(mvGridCorners);
  }

  // Update connection states:
  int nTargetNum = mvGridCorners.size();
  for (int dirn = 0; dirn < 4; dirn++)
  {
    CVD::ImageRef irSearch = gTarget.mirGridPos + IR_from_dirn(dirn);
    for (unsigned int i = 0; i < mvGridCorners.size(); i++)
    {
      if (mvGridCorners[i].mirGridPos == irSearch)
      {
        gTarget.maNeighborStates[dirn].val = i;
        mvGridCorners[i].maNeighborStates[(dirn + 2) % 4].val = nTargetNum;
      }
    }
  }
  mvGridCorners.push_back(gTarget);
  mvGridCorners.back().Draw();

  return true;
}

// Draw a grid over the points that have been found
void CalibImageTaylor::DrawImageGrid()
{
  int nOriginIdx = -1;

  glLineWidth(2);
  glColor3f(0, 0, 1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);

  for (int i = 0; i < static_cast<int>(mvGridCorners.size()); i++)
  {
    if (mvGridCorners[i].mirGridPos == CVD::ImageRef(0, 0))
      nOriginIdx = i;

    for (int dirn = 0; dirn < 4; dirn++)
    {
      if (mvGridCorners[i].maNeighborStates[dirn].val > i)
      {
        CVD::glVertex(mvGridCorners[i].mParams.v2Pos + CVD::vec(mirDrawOffset));
        CVD::glVertex(mvGridCorners[mvGridCorners[i].maNeighborStates[dirn].val].mParams.v2Pos +
                      CVD::vec(mirDrawOffset));
      }
    }
  }
  glEnd();

  if (nOriginIdx >= 0)
  {
    glLineWidth(4);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);

    int nXNeighborIdx = mvGridCorners[nOriginIdx].maNeighborStates[0].val;
    int nYNeighborIdx = mvGridCorners[nOriginIdx].maNeighborStates[1].val;

    if (nXNeighborIdx >= 0)
    {
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);  // red for X axis
      CVD::glVertex(mvGridCorners[nOriginIdx].mParams.v2Pos + CVD::vec(mirDrawOffset));
      CVD::glVertex(mvGridCorners[nXNeighborIdx].mParams.v2Pos + CVD::vec(mirDrawOffset));
      glEnd();
    }

    if (nYNeighborIdx >= 0)
    {
      glBegin(GL_LINES);
      glColor3f(0, 1, 0);  // green for Y axis
      CVD::glVertex(mvGridCorners[nOriginIdx].mParams.v2Pos + CVD::vec(mirDrawOffset));
      CVD::glVertex(mvGridCorners[nYNeighborIdx].mParams.v2Pos + CVD::vec(mirDrawOffset));
      glEnd();
    }
  }

  glPointSize(5);
  glEnable(GL_POINT_SMOOTH);
  glColor3f(1, 1, 0);
  glBegin(GL_POINTS);
  for (unsigned int i = 0; i < mvGridCorners.size(); i++)
  {
    CVD::glVertex(mvGridCorners[i].mParams.v2Pos + CVD::vec(mirDrawOffset));
  }
  glEnd();

  glColor3f(0, 0, 1);
  for (unsigned int i = 0; i < mvGridCorners.size(); i++)
  {
    std::stringstream ss;
    ss << mvGridCorners[i].mirGridPos;
    mpGLWindow->PrintString(mirDrawOffset + CVD::ImageRef(5, -5) + CVD::ir_rounded(mvGridCorners[i].mParams.v2Pos),
                            ss.str(), 7);
  }
}

// Draw the current grid (as defined by the current found checkerboard corners), projected into a given camera at the
// current pose of the CalibImageTaylor
void CalibImageTaylor::Draw3DGrid(TaylorCamera& camera, bool bDrawErrors)
{
  glLineWidth(2);
  glColor3f(0, 0.7, 1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);

  for (int i = 0; i < static_cast<int>(mvGridCorners.size()); i++)
  {
    for (int dirn = 0; dirn < 4; dirn++)
    {
      if (mvGridCorners[i].maNeighborStates[dirn].val > i)
      {
        TooN::Vector<3> v3;
        v3[2] = 0.0;
        v3.slice<0, 2>() = CVD::vec(mvGridCorners[i].mirGridPos);
        CVD::glVertex(camera.Project(mse3CamFromWorld * v3) + CVD::vec(mirDrawOffset));
        v3.slice<0, 2>() = CVD::vec(mvGridCorners[mvGridCorners[i].maNeighborStates[dirn].val].mirGridPos);
        CVD::glVertex(camera.Project(mse3CamFromWorld * v3) + CVD::vec(mirDrawOffset));
      }
    }
  }
  glEnd();

  if (bDrawErrors)
  {
    glColor3f(1, 0, 0);
    glLineWidth(1);
    glBegin(GL_LINES);
    for (int i = 0; i < static_cast<int>(mvGridCorners.size()); i++)
    {
      TooN::Vector<3> v3;
      v3[2] = 0.0;
      v3.slice<0, 2>() = CVD::vec(mvGridCorners[i].mirGridPos);
      TooN::Vector<2> v2Pixels_Projected = camera.Project(mse3CamFromWorld * v3);
      TooN::Vector<2> v2Error = mvGridCorners[i].mParams.v2Pos - v2Pixels_Projected;
      CVD::glVertex(v2Pixels_Projected + CVD::vec(mirDrawOffset));
      CVD::glVertex(v2Pixels_Projected + 10.0 * v2Error + CVD::vec(mirDrawOffset));
    }
    glEnd();
  }
}

// Helper function to convert the direction 0-3 to an ImageRef
CVD::ImageRef CalibImageTaylor::IR_from_dirn(int nDirn)
{
  CVD::ImageRef ir;
  ir[nDirn % 2] = (nDirn < 2) ? 1 : -1;
  return ir;
}

// Given the vector H from Scaramuzza thesis eq 3.7, solves for one of two entries of the 3rd column of the rotation
// matrix of the image pose.
double CalibImageTaylor::SolveForR3(TooN::Vector<6> v6H, int nRNum)
{
  // See the Matlab code of OCamCalib for more info on this

  double r11 = v6H[0];
  double r12 = v6H[1];
  double r21 = v6H[2];
  double r22 = v6H[3];

  double A1 = pow(r11, 2) + pow(r21, 2);
  double A2 = pow(r12, 2) + pow(r22, 2);
  double A;

  if (nRNum == 1)
  {
    A = A1 - A2;
  }
  else
  {
    A = A2 - A1;
  }

  double B = pow((-r11 * r12 - r21 * r22), 2);

  Eigen::Matrix<double, 3, 1> polynomial;
  polynomial << -1 * B, A, 1;

  Eigen::PolynomialSolver<double, 2> psolve(polynomial);

  std::vector<double> realRoots;
  psolve.realRoots(realRoots);

  double xSoln = -1;

  // The first real root is the solution
  for (unsigned i = 0; i < realRoots.size(); ++i)
  {
    if (realRoots[i] >= 0)
    {
      xSoln = realRoots[i];
      break;
    }
  }

  ROS_ASSERT(xSoln >= 0);

  return sqrt(xSoln);
}

// Degree is degree of polynomial that will be built into the matrix, nImgNum is the number of this image in the overall
// sequence of nTotalNum
// Assumes that the current mse3CamFromWorld has been updated, so that the column vectors of the rotation matrix can be
// extracted as well as
// the first two components of the translation vector
// nImgNum and nTotalNum are in c++ vector format, ie if number of images is 2, nTotalNum = 2, and nImgNum = 0 or 1
std::pair<TooN::Matrix<>, TooN::Vector<>> CalibImageTaylor::BuildIntrinsicMatrixEntries(TooN::Vector<2> v2Center,
                           int nDegree, int nImgNum, int nTotalNum)
{
  // Wouldn't make sense for less than degree 2
  ROS_ASSERT(nDegree >= 2);

  int nPoints = mvGridCorners.size();
  TooN::Matrix<> mxM_segment = TooN::Zeros(2 * nPoints, nDegree + nTotalNum);
  TooN::Vector<> vxB_segment(2 * nPoints);
  int nIdx = 0;

  for (int n = 0; n < nPoints; n++)
  {
    // Assumes identity affine transform, will be estimated elsewhere with nonlinear refinement
    double u = mvGridCorners[n].mParams.v2Pos[0] - v2Center[0];
    double v = mvGridCorners[n].mParams.v2Pos[1] - v2Center[1];
    double Xt = mvGridCorners[n].mirGridPos.x;
    double Yt = mvGridCorners[n].mirGridPos.y;

    double rho = sqrt(u * u + v * v);

    // mse3CamFromWorld needs to be up to date!
    TooN::Matrix<3> R = mse3CamFromWorld.get_rotation().get_matrix();
    double& r11 = R[0][0];
    double& r21 = R[1][0];
    double& r31 = R[2][0];
    double& r12 = R[0][1];
    double& r22 = R[1][1];
    double& r32 = R[2][1];

    TooN::Vector<3> t = mse3CamFromWorld.get_translation();
    double& t1 = t[0];
    double& t2 = t[1];

    // See Scaramuzza thesis eq 3.8
    double A = r21 * Xt + r22 * Yt + t2;
    double B = v * (r31 * Xt + r32 * Yt);
    double C = r11 * Xt + r12 * Yt + t1;
    double D = u * (r31 * Xt + r32 * Yt);

    mxM_segment[nIdx][0] = A;
    mxM_segment[nIdx + 1][0] = C;

    for (int i = 1; i < nDegree; ++i)
    {
      mxM_segment[nIdx][i] = A * pow(rho, i + 1);
      mxM_segment[nIdx + 1][i] = C * pow(rho, i + 1);
    }

    mxM_segment[nIdx][nDegree + nImgNum] = -v;
    mxM_segment[nIdx + 1][nDegree + nImgNum] = -u;

    vxB_segment[nIdx] = B;
    vxB_segment[nIdx + 1] = D;

    nIdx += 2;
  }

  return std::make_pair(mxM_segment, vxB_segment);
}

// Converts a 3x3 matrix representation of the pose to an SE3
TooN::SE3<> CalibImageTaylor::M3ToSE3(TooN::Matrix<3> R)
{
  TooN::SE3<> se3;
  TooN::Matrix<3> rot;

  // First two columns are the same
  rot.T()[0] = R.T()[0];
  rot.T()[1] = R.T()[1];

  // Last column is cross product of first two
  rot.T()[2] = R.T()[0] ^ R.T()[1];

  se3.get_rotation() = rot;

  // Transltation is last column
  se3.get_translation() = R.T()[2];

  return se3;
}

// Decide which matrix from the inputs contains the correct rotation
int CalibImageTaylor::FindCorrectRotation(std::vector<TooN::Matrix<3>> vRs, TooN::Vector<2> v2Center)
{
  // Save old transform in case someone else is using it
  TooN::SE3<> se3Backup = mse3CamFromWorld;
  int nCorrectIdx = -1;
  double dSmallestCoeff = 1e100;

  // Try all the matrices in order
  // We'll try to find the camera parameters and t3 translation assuming a 2nd order polynomial, which will be enough
  // for us to check if t3 has the right sign
  for (unsigned i = 0; i < vRs.size(); ++i)
  {
    mse3CamFromWorld = M3ToSE3(vRs[i]);
    std::pair<TooN::Matrix<>, TooN::Vector<>> Mb = BuildIntrinsicMatrixEntries(v2Center, 2, 0, 1);

    TooN::SVD<> svd(Mb.first);
    TooN::Vector<> vxParams = svd.backsub(Mb.second);

    if (vxParams[2] > 0)  // This is the t3 term in a second order fit, if it's positive we have the right solution
    {
      if (vxParams[1] < dSmallestCoeff)
      {
        nCorrectIdx = i;
        dSmallestCoeff = vxParams[1];
      }
    }
  }

  ROS_ASSERT(nCorrectIdx != -1);

  // Restore old transform
  mse3CamFromWorld = se3Backup;

  return nCorrectIdx;
}

// Calculates an initial guess for the pose of the image, based on the given camera model
// See Scaramuzza thesis section 3.2.1
void CalibImageTaylor::GuessInitialPose(TooN::Vector<2> v2Center)
{
  int nPoints = mvGridCorners.size();
  TooN::Matrix<> mxN6(nPoints, 6);

  /*
  double dFirstU = 0, dFirstV = 0;
  double minXt = 1e10;
  double minYt = 1e10;
  */

  for (int n = 0; n < nPoints; n++)
  {
    // Assumes identity affine transform, will be estimated elsewhere with nonlinear refinement
    double u = mvGridCorners[n].mParams.v2Pos[0] - v2Center[0];
    double v = mvGridCorners[n].mParams.v2Pos[1] - v2Center[1];
    double Xt = mvGridCorners[n].mirGridPos.x;
    double Yt = mvGridCorners[n].mirGridPos.y;

    /*
    if(Xt < minXt || Yt < minYt)
    {
      dFirstU = u;
      dFirstV = v;
      minXt = Xt;
      minYt = Yt;
    }
    */

    mxN6[n][0] = -1 * v * Xt;
    mxN6[n][1] = -1 * v * Yt;
    mxN6[n][2] = u * Xt;
    mxN6[n][3] = u * Yt;
    mxN6[n][4] = -1 * v;
    mxN6[n][5] = u;
  }

  // Get H vector solution as last colum of V in SVD
  TooN::SVD<> svd(mxN6);
  TooN::Vector<6> v6H = svd.get_VT()[5];

  // debug
  TooN::Vector<> vxResidual = mxN6 * v6H;
  ROS_DEBUG_STREAM("####### residual error: " << sqrt(vxResidual * vxResidual));

  // Try solving for r32 first, if close to zero then solve for r31 directly instead of dividing by r32
  double r32 = SolveForR3(v6H, 2);
  double r31;

  double r11 = v6H[0];
  double r12 = v6H[1];
  double r21 = v6H[2];
  double r22 = v6H[3];
  double t1 = v6H[4];
  double t2 = v6H[5];

  if (r32 < 1e-10)
  {
    r31 = SolveForR3(v6H, 1);
    r32 = (-1 * r11 * r12 - r21 * r22) / r31;
  }
  else
  {
    r31 = (-1 * r11 * r12 - r21 * r22) / r32;
  }

  double dLambda = 1 / sqrt(r11 * r11 + r21 * r21 + r31 * r31);
  /*
  int nLambdaSign = 0;

  int nSignOfFirstU = dFirstU >= 0 ? 1 : -1;
  int nSignOfFirstV = dFirstV >= 0 ? 1 : -1;
  int nSignOfT1 = t1 >= 0 ? 1 : -1;
  int nSignOfT2 = t2 >= 0 ? 1 : -1;

  if(nSignOfFirstU == nSignOfT1 && nSignOfFirstV == nSignOfT2) // lambda should be positive
    nLambdaSign = 1;
  else //if(signOfFirstU != signOfT1 && signOfFirstV != signOfT2) // lambda should be negative
    nLambdaSign = -1;

  // The above else if condition was producing bad results when the origin of the grid was very close to the center of
  the image (< 0.5 pixels)
  // and the least squares fit solved for the t1 and/or t2 vectors with the wrong sign (but very small magnitude). In
  this case the assertion
  // below would trigger even though we could have picked either lambda sign and it would have worked
  // Therefore leave it as an "else" condition

  ROS_ASSERT(nLambdaSign != 0);
  */

  TooN::Vector<3> r1 = TooN::makeVector(r11, r21, r31);
  TooN::Vector<3> r2 = TooN::makeVector(r12, r22, r32);
  TooN::Vector<3> t = TooN::makeVector(t1, t2, 0);

  std::vector<TooN::Matrix<3>> vRs(4);
  int signs[] = {1, -1};
  int lambdaSigns[] = {1, -1};

  // Fill the two possible R matrices
  for (unsigned i = 0; i < vRs.size(); ++i)
  {
    TooN::Matrix<3>& m3R = vRs[i];
    m3R.T()[0] = dLambda * lambdaSigns[i % 2] * r1;
    m3R.T()[1] = dLambda * lambdaSigns[i % 2] * r2;
    m3R.T()[2] = dLambda * lambdaSigns[i % 2] * t;

    // Two possible choice of sign for r31 and r32
    m3R[2][0] *= signs[i / 2];
    m3R[2][1] *= signs[i / 2];
    m3R[2][2] = 1;  // Set t3 to 1 for now
  }

  int nCorrectIdx = FindCorrectRotation(vRs, v2Center);

  ROS_ASSERT(nCorrectIdx >= 0);

  // Store result
  mse3CamFromWorld = M3ToSE3(vRs[nCorrectIdx]);
};

// Project the found corners using the given camera model, but return only only the projection errors without any
// jacobians
std::vector<TooN::Vector<2>> CalibImageTaylor::ProjectGetOnlyErrors(TaylorCamera& camera, bool bNewTransform)
{
  std::vector<TooN::Vector<2>> vResult;
  TooN::SE3<> se3Transform = bNewTransform ? mse3CamFromWorldNew : mse3CamFromWorld;

  for (unsigned int n = 0; n < mvGridCorners.size(); n++)
  {
    // First, project into image...
    TooN::Vector<3> v3World;
    v3World[2] = 0.0;
    v3World.slice<0, 2>() = CVD::vec(mvGridCorners[n].mirGridPos);

    TooN::Vector<3> v3Cam = se3Transform * v3World;

    TooN::Vector<2> v2Proj = camera.Project(v3Cam);
    TooN::Vector<2> v2Error;
    if (camera.Invalid())
    {
      // continue;
      v2Error = TooN::makeVector(1e10, 1e10);
    }
    else
    {
      v2Error = mvGridCorners[n].mParams.v2Pos - v2Proj;
    }
    vResult.push_back(v2Error);
  }

  return vResult;
}

// Projects all the found corners using the given camera model and the currently set pose, returns errors and jacobians
std::vector<CalibImageTaylor::ErrorAndJacobians> CalibImageTaylor::Project(TaylorCamera& camera)
{
  std::vector<ErrorAndJacobians> vResult;
  for (unsigned int n = 0; n < mvGridCorners.size(); n++)
  {
    ErrorAndJacobians EAJ;

    // First, project into image...
    TooN::Vector<3> v3World;
    v3World[2] = 0.0;
    v3World.slice<0, 2>() = CVD::vec(mvGridCorners[n].mirGridPos);

    TooN::Vector<3> v3Cam = mse3CamFromWorld * v3World;

    TooN::Vector<2> v2Proj = camera.Project(v3Cam);
    if (camera.Invalid())
    {
      continue;
    }

    EAJ.v2Error = mvGridCorners[n].mParams.v2Pos - v2Proj;

    // Now find motion jacobian..
    TooN::Matrix<2> m2CamDerivs = camera.GetProjectionDerivs();

    // For each of six degrees of freedom...
    for (int dof = 0; dof < 6; dof++)
    {
      // Get the motion of the point in the camera's frame when the pose of the camera changes by one of the degrees of
      // freedom
      const TooN::Vector<4> v4Motion = TooN::SE3<>::generator_field(dof, unproject(v3Cam));

      TooN::Vector<3> v3_dTheta, v3_dPhi;
      TaylorCamera::GetCamSphereDeriv(v3Cam, v3_dTheta, v3_dPhi);
      TooN::Vector<2> v2CamSphereMotion;
      v2CamSphereMotion[0] = v3_dTheta * v4Motion.slice<0, 3>();  // theta component
      v2CamSphereMotion[1] = v3_dPhi * v4Motion.slice<0, 3>();    // phi component

      // The camera derivatives are motion of pixel relative to motion on unit sphere, so a multiplication gets the
      // desired Jacobian entry
      EAJ.m26PoseJac.T()[dof] = m2CamDerivs * v2CamSphereMotion;
    }

    // Finally, the camera provids its own jacobian
    EAJ.m2NCameraJac = camera.GetCameraParameterDerivs();
    vResult.push_back(EAJ);
  }
  return vResult;
}
