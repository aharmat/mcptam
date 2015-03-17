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
 * \file Types.h
 * \brief Declaration of generic types for mcptam
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * A bunch of typedefs that are used throughout mcptam.
 *
 ****************************************************************************************/

#ifndef __TYPES_H
#define __TYPES_H

#include <mcptam/TaylorCamera.h>
#include <mcptam/KeyFrame.h>
#include <map>
#include <list>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <boost/intrusive_ptr.hpp>

// Forward declarations
class MapPoint;
class TrackerData;
class KeyFrame;
class MultiKeyFrame;

typedef std::list<MapPoint*> MapPointPtrList;
typedef std::list<MultiKeyFrame*> MultiKeyFramePtrList;

typedef std::map<std::string, TaylorCamera> TaylorCameraMap;
typedef std::map<std::string, TooN::Vector<9> > ParamMap;

typedef std::map<std::string, CVD::ImageRef > ImageRefMap;
typedef std::map<std::string, CVD::Image<CVD::byte> > ImageBWMap;
typedef std::map<std::string, CVD::Image<CVD::Rgb<CVD::byte> > > ImageRGBMap;

typedef std::map<std::string, TooN::SE3<> > SE3Map;

/// Using a boost intrusive_ptr allows claiming a MapPoint as "used", so it won't
/// be deleted until it is released. See MapPoint.
typedef std::vector<boost::intrusive_ptr<TrackerData> > TrackerDataPtrVector;

/// Can't put a LEVEL-sized array of objects into another STL container, so use this intermediary
template<typename T>
struct LevelsArray
{
  inline T& operator[] (unsigned i)
  {
    return array[i];
  }
  
  T array[LEVELS];
};

typedef LevelsArray<TrackerDataPtrVector> TDVLevels;
typedef LevelsArray<int> IntLevels;
typedef LevelsArray<std::vector<TooN::Vector<2> > > V2Levels;

typedef std::map<std::string, IntLevels > IntLevelsMap;
typedef std::map<std::string, V2Levels > V2LevelsMap;

#endif
