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
 * \file LevelHelpers.h
 * \brief Declaration of various helper functions for Levels
 *
 * Copyright 2008 Isis Innovation Limited
 *
 * A few handy tools to ease using levels.
 * The important thing is the XXXPos functions, which convert
 * image positions from one level to another. Use these whenever
 * transforming positions to ensure consistent operation!!
 *
 ****************************************************************************************/

#ifndef MCPTAM_LEVELHELPERS_H
#define MCPTAM_LEVELHELPERS_H

#include <TooN/TooN.h>
#include <cvd/image_ref.h>

/// Set of global colours useful for drawing stuff:
extern TooN::Vector<3> gavLevelColors[];
// (These are filled in in KeyFrame.cc)

/// Get the scale of the level
inline int LevelScale(int nLevel)
{
  return 1 << nLevel;
}

/// 1-D transform to level zero:
inline double LevelZeroPos(double dLevelPos, int nLevel)
{
  return (dLevelPos + 0.5) * LevelScale(nLevel) - 0.5;
}

/// 2-D transforms to level zero:
inline TooN::Vector<2> LevelZeroPos(TooN::Vector<2> v2LevelPos, int nLevel)
{
  TooN::Vector<2> v2Ans;
  v2Ans[0] = LevelZeroPos(v2LevelPos[0], nLevel);
  v2Ans[1] = LevelZeroPos(v2LevelPos[1], nLevel);
  return v2Ans;
}

/// 2-D transforms to level zero:
inline TooN::Vector<2> LevelZeroPos(CVD::ImageRef irLevelPos, int nLevel)
{
  TooN::Vector<2> v2Ans;
  v2Ans[0] = LevelZeroPos(irLevelPos.x, nLevel);
  v2Ans[1] = LevelZeroPos(irLevelPos.y, nLevel);
  return v2Ans;
}

/// 1-D transform from level zero to level N:
inline double LevelNPos(double dRootPos, int nLevel)
{
  return (dRootPos + 0.5) / LevelScale(nLevel) - 0.5;
}

/// 2-D transform from level zero to level N:
inline TooN::Vector<2> LevelNPos(TooN::Vector<2> v2RootPos, int nLevel)
{
  TooN::Vector<2> v2Ans;
  v2Ans[0] = LevelNPos(v2RootPos[0], nLevel);
  v2Ans[1] = LevelNPos(v2RootPos[1], nLevel);
  return v2Ans;
}

#endif  // MCPTAM_LEVELHELPERS_H
