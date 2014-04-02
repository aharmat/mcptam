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
 * \file CalibGridCorner.h
 * \brief Declaration of CalibGridCorner class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Most of this code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * This class was extracted from PTAM's CalibImage.h file. While developing CalibImageTaylor,
 * it was necessary to concurrently test it alongside the original CalibImage and the 
 * definition of CalibGridCorner in both files was a problem, so it was moved. Now with
 * CalibImageTaylor functional, I could merge this back into CalibImageTaylor.h but haven't
 * found any good reason to do so.
 *
 ****************************************************************************************/

#ifndef __CALIB_GRID_CORNER_H
#define __CALIB_GRID_CORNER_H

#include <mcptam/CalibCornerPatch.h>
#include <vector>
#include <TooN/TooN.h>

const int N_NOT_TRIED=-1;
const int N_FAILED=-2;

/** @brief Holds various data about a checkerboard grid corner once its position has been found */
class CalibGridCorner
{
public:

  struct NeighborState
  {
    NeighborState() {val = N_NOT_TRIED;}
    int val;
  };
  
  CalibGridCorner(CVD::ImageRef irDrawOffset=CVD::ImageRef()) : mirDrawOffset(irDrawOffset) {};
  
  CalibCornerPatch::Params mParams;
  CVD::ImageRef mirGridPos;
  NeighborState maNeighborStates[4];
  
  TooN::Matrix<2> GetSteps(std::vector<CalibGridCorner> &vgc); 
  TooN::Matrix<2> mm2InheritedSteps;
  
  void Draw();
  
  double ExpansionPotential();
  
  CVD::ImageRef mirDrawOffset;
};

#endif

