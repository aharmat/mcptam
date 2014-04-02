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
#include <mcptam/ShiTomasi.h>
#include <math.h>

double FindShiTomasiScoreAtPoint(CVD::BasicImage<CVD::byte> &image, int nHalfBoxSize, CVD::ImageRef irCenter)
{
  double dXX = 0;
  double dYY = 0;
  double dXY = 0;
  
  CVD::ImageRef irStart = irCenter - CVD::ImageRef(nHalfBoxSize, nHalfBoxSize);
  CVD::ImageRef irEnd = irCenter +CVD:: ImageRef(nHalfBoxSize, nHalfBoxSize);
  
  CVD::ImageRef ir;
  for(ir.y = irStart.y; ir.y<=irEnd.y; ir.y++)
  {
    for(ir.x = irStart.x; ir.x<=irEnd.x; ir.x++)
    {
      double dx = image[ir + CVD::ImageRef(1,0)] - image[ir - CVD::ImageRef(1,0)];
      double dy = image[ir + CVD::ImageRef(0,1)] - image[ir - CVD::ImageRef(0,1)];
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }
  }
  
  int nPixels = (irEnd - irStart + CVD::ImageRef(1,1)).area();
  dXX = dXX / (2.0 * nPixels);
  dYY = dYY / (2.0 * nPixels);
  dXY = dXY / (2.0 * nPixels);
  
  // Find and return smaller eigenvalue:
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
};

