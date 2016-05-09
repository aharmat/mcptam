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
 * \file SmallMatrixOpts.h
 * \brief Define methods for operations on small matrices
 *
 * This code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 ****************************************************************************************/

// Inverse of 2-matrix
// Must be invertible!
#ifndef MCPTAM_SMALLMATRIXOPTS_H
#define MCPTAM_SMALLMATRIXOPTS_H

#include <TooN/TooN.h>
#include <ros/ros.h>

// Don't pollute the global namespace
namespace opts
{
/// Determinant of 2x2 matrix
inline double M2Det(const TooN::Matrix<2> &m)
{
  return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}

/// Determinant of 3x3 matrix
inline double M3Det(const TooN::Matrix<3> &m)
{
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/// Inverse of 2x2 matrix
inline TooN::Matrix<2> M2Inverse(const TooN::Matrix<2> &m)
{
  TooN::Matrix<2> m2Res;
  double dDet = M2Det(m);
  ROS_ASSERT(dDet != 0.0);
  double dInverseDet = 1.0 / dDet;
  m2Res[0][0] = m[1][1] * dInverseDet;
  m2Res[1][1] = m[0][0] * dInverseDet;
  m2Res[1][0] = -m[1][0] * dInverseDet;
  m2Res[0][1] = -m[0][1] * dInverseDet;
  return m2Res;
};

}  // end namespace opts

#endif  // MCPTAM_SMALLMATRIXOPTS_H
