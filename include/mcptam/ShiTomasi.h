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
 * \file ShiTomasi.h
 * \brief Define methods for evaluating Shi Tomasi features.
 *
 * This code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 ****************************************************************************************/

#ifndef MCPTAM_SHITOMASI_H
#define MCPTAM_SHITOMASI_H

#include <cvd/image.h>
#include <cvd/byte.h>

/** @brief Evaluate the Shi Tomasi Score at the specified coordinates
 *  @param image The image we're working on
 *  @param nHalfBoxSize Half of the width/height of the rectangle around the
 * point that will be used for computation
 *  @param irCenter The point location
 *  @return The Shi-Tomasi score */
double FindShiTomasiScoreAtPoint(CVD::BasicImage<CVD::byte> &image, int nHalfBoxSize, CVD::ImageRef irCenter);

#endif  // MCPTAM_SHITOMASI_H
