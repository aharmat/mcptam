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
 * \file MiniPatch.h
 * \brief Declaration of MiniPatch class
 *
 * This code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * This is a simple pixel-patch class, used for tracking small patches
 * it's used by the tracker for building the initial map
 *
 ****************************************************************************************/

#ifndef __MINI_PATCH_H
#define __MINI_PATCH_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/utility.h>
#include <vector>

/** @brief A simple pixel-patch class, used for tracking small patches */
class MiniPatch
{
public:

  void SampleFromImage(CVD::ImageRef irPos, CVD::BasicImage<CVD::byte> &im);  // Copy pixels out of source image
  bool FindPatch(CVD::ImageRef &irPos,           // Find patch in a new image
		 CVD::BasicImage<CVD::byte> &im, 
		 int nRange,
		 std::vector<CVD::ImageRef> &vCorners,
		 std::vector<int>* pvRowLUT = NULL);
  
  inline int SSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir); // Score function
  static int mnHalfPatchSize;     // How big is the patch?
  static int mnRange;             // How far to search? 
  static int mnMaxSSD;            // Max SSD for matches?
  CVD::Image<CVD::byte> mimOrigPatch;  // Original pixels
};

#endif

