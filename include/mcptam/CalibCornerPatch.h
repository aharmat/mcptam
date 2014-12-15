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
 * \file CalibCornerPatch.h
 * \brief Declaration of CalibCornerPatch class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Most of this code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Finds a calibration corner (ie corner of a checkerboard) using a given starting pose
 * My only modification is to add mirDrawOffset which allows drawing over the images of
 * a multi-camera system.
 *
 ****************************************************************************************/

#ifndef __CALIB_CORNER_PATCH_H
#define __CALIB_CORNER_PATCH_H

#include <TooN/TooN.h>
#include <cvd/image.h>
#include <cvd/byte.h>


/** @brief Finds a calibration corner (ie a corner on a checkerboard) in an image
 * 
 *  Uses a given initial patch position to try to find an accurate corner location. */
class CalibCornerPatch
{
public:
  struct Params
  {
    Params();
    TooN::Matrix<2> m2Warp();
    TooN::Vector<2> v2Pos;
    TooN::Vector<3> v3Pos;
    TooN::Vector<2> v2Angles;
    double dMean;
    double dGain;
  };
  
  CalibCornerPatch(int nSideSize = 8, CVD::ImageRef irDrawOffset=CVD::ImageRef());
  bool IterateOnImage(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnImageWithDrawing(Params &params, CVD::Image<CVD::byte> &im);

 protected:
  void MakeTemplateWithCurrentParams();
  void FillTemplate(CVD::Image<float> &im, Params params);
  double Iterate(CVD::Image<CVD::byte> &im);
  Params mParams;
  CVD::Image<float> mimTemplate;
  CVD::Image<TooN::Vector<2> > mimGradients;
  CVD::Image<TooN::Vector<2> > mimAngleJacs;
  
  void MakeSharedTemplate();
  static CVD::Image<float> mimSharedSourceTemplate;

  double mdLastError;
  CVD::ImageRef mirDrawOffset;
};

#endif

