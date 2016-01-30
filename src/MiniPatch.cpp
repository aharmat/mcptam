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
#include <mcptam/MiniPatch.h>
#include <algorithm>
#include <vector>


// Scoring function
inline int MiniPatch::SSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir)
{
  if (!im.in_image_with_border(ir, mnHalfPatchSize))
    return mnMaxSSD + 1;
  CVD::ImageRef irImgBase = ir - CVD::ImageRef(mnHalfPatchSize, mnHalfPatchSize);
  int nRows = mimOrigPatch.size().y;
  int nCols = mimOrigPatch.size().x;
  CVD::byte *imagepointer;
  CVD::byte *templatepointer;
  int nDiff;
  int nSumSqDiff = 0;
  for (int nRow = 0; nRow < nRows; nRow++)
  {
    imagepointer = &im[irImgBase + CVD::ImageRef(0, nRow)];
    templatepointer = &mimOrigPatch[CVD::ImageRef(0, nRow)];
    for (int nCol = 0; nCol < nCols; nCol++)
    {
      nDiff = imagepointer[nCol] - templatepointer[nCol];
      nSumSqDiff += nDiff * nDiff;
    }
  }
  return nSumSqDiff;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
bool MiniPatch::FindPatch(CVD::ImageRef &irPos, CVD::BasicImage<CVD::byte> &im, int nRange,
                          std::vector<CVD::ImageRef> &vCorners, std::vector<int> *pvRowLUT)
{
  CVD::ImageRef irBest;
  int nBestSSD = mnMaxSSD + 1;
  CVD::ImageRef irBBoxTL = irPos - CVD::ImageRef(nRange, nRange);
  CVD::ImageRef irBBoxBR = irPos + CVD::ImageRef(nRange, nRange);
  std::vector<CVD::ImageRef>::iterator i;
  if (!pvRowLUT)
  {
    for (i = vCorners.begin(); i != vCorners.end(); i++)
    {
      if (i->y >= irBBoxTL.y)
        break;
    }
  }
  else
  {
    int nTopRow = irBBoxTL.y;
    if (nTopRow < 0)
      nTopRow = 0;

    if (nTopRow >= static_cast<int>(pvRowLUT->size()))
      nTopRow = static_cast<int>(pvRowLUT->size()) - 1;

    i = vCorners.begin() + (*pvRowLUT)[nTopRow];
  }

  for (; i != vCorners.end(); i++)
  {
    if (i->x < irBBoxTL.x || i->x > irBBoxBR.x)
      continue;
    if (i->y > irBBoxBR.y)
      break;
    int nSSD = SSDAtPoint(im, *i);

    if (nSSD < nBestSSD)
    {
      irBest = *i;
      nBestSSD = nSSD;
    }
  }
  if (nBestSSD < mnMaxSSD)
  {
    irPos = irBest;
    return true;
  }
  else
    return false;
}

// Define the patch from an input image
void MiniPatch::SampleFromImage(CVD::ImageRef irPos, CVD::BasicImage<CVD::byte> &im)
{
  assert(im.in_image_with_border(irPos, mnHalfPatchSize));
  CVD::ImageRef irPatchSize(2 * mnHalfPatchSize + 1, 2 * mnHalfPatchSize + 1);
  mimOrigPatch.resize(irPatchSize);
  CVD::copy(im, mimOrigPatch, mimOrigPatch.size(), irPos - mimOrigPatch.size() / 2);
}

// Static members
int MiniPatch::mnHalfPatchSize = 4;
int MiniPatch::mnRange = 10;
int MiniPatch::mnMaxSSD = 9999;
