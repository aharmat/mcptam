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
// Modifications
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Almost all the code below is from the original PTAM, which is 
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/CalibGridCorner.h>
#include <mcptam/OpenGL.h>
#include <cvd/image.h>
#include <cvd/vector_image_ref.h>

using namespace TooN;

void CalibGridCorner::Draw()
{
  glColor3f(0,1,0);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_LINES);
  CVD::glVertex(mParams.v2Pos + mParams.m2Warp() * CVD::vec(CVD::ImageRef( 10,0)) + CVD::vec(mirDrawOffset));
  CVD::glVertex(mParams.v2Pos + mParams.m2Warp() * CVD::vec(CVD::ImageRef(-10,0)) + CVD::vec(mirDrawOffset));
  CVD::glVertex(mParams.v2Pos + mParams.m2Warp() * CVD::vec(CVD::ImageRef( 0, 10)) + CVD::vec(mirDrawOffset));
  CVD::glVertex(mParams.v2Pos + mParams.m2Warp() * CVD::vec(CVD::ImageRef( 0,-10)) + CVD::vec(mirDrawOffset));
  glEnd();
}


double CalibGridCorner::ExpansionPotential()
{
  // Scoring function. How good would this grid corner be at finding a neighbor?
  // The best case is if it's already surrounded by three neighbors and only needs
  // to find the last one (because it'll have the most accurate guess for where
  // the last one should be) and so on.
  int nMissing = 0;
  for(int i=0; i<4; i++)
  {
    if(maNeighborStates[i].val == N_NOT_TRIED)
      nMissing++;
  }
  
  if(nMissing == 0)
    return 0.0;
  
  if(nMissing == 1)
    return 100.0;
  
  if(nMissing == 3)
    return 1.0;

  if(nMissing == 2)
  {
    int nFirst = 0;
    while(maNeighborStates[nFirst].val != N_NOT_TRIED)
      nFirst++;
    
    if(maNeighborStates[(nFirst + 2) % 4].val == N_NOT_TRIED)
      return 10.0;
    else
      return 20.0;
  }
  assert(0); // should never get here
  return 0.0;
}

Matrix<2> CalibGridCorner::GetSteps(std::vector<CalibGridCorner> &vgc)
{
  Matrix<2> m2Steps;
  for(int dirn=0; dirn<2; dirn++)
  {
    Vector<2> v2Dirn;
    int nFound = 0;
    v2Dirn = Zeros;
    if(maNeighborStates[dirn].val >=0)
    {
      v2Dirn += vgc[maNeighborStates[dirn].val].mParams.v2Pos - mParams.v2Pos;
      nFound++;
    }
    if(maNeighborStates[dirn+2].val >=0)
    {
      v2Dirn -= vgc[maNeighborStates[dirn+2].val].mParams.v2Pos - mParams.v2Pos;
      nFound++;
    }
    
    if(nFound == 0)
      m2Steps[dirn] = mm2InheritedSteps[dirn];
    else
      m2Steps[dirn] = v2Dirn / nFound;
  }
  return m2Steps;
}


