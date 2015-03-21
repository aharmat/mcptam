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
 * \file RelocaliserFabMap.h
 * \brief Declares the RelocaliserFabMap class
 *
 * This code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Each KF stores a small, blurred version of itself;
 * Just compare a small, blurred version of the input frame to all the KFs,
 * choose the closest match, and then estimate a camera rotation by direct image
 * minimisation.
 *
 ****************************************************************************************/

#ifndef __RELOCALISER_BRUTE_FORCE_H
#define __RELOCALISER_BRUTE_FORCE_H

#include <mcptam/TaylorCamera.h>
#include <mcptam/Types.h>
#include <TooN/se3.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

class Map;
class GLWindow2;
class KeyFrame;
struct Level;


/** @brief Attempts to find the best pose for a given KeyFrame by comparing to others in the Map.
 * 
 * Uses the SmallBlurryImage to find the best KeyFrame match, then computes the 2D transform between
 * them, then finally finds the 3D transform that would have generated that 2D transform. This class
 * is essentially directly from PTAM */
class RelocaliserBruteForce
{
public:
  /** @brief Used when there is only one camera in the system
   *  @param map The Map being worked on
   *  @param camera The TaylorCamera model
   *  @param camName The name of the camera */
  RelocaliserBruteForce(Map &map, TooN::Vector<6> v6StepSizes);
  
  ~RelocaliserFabMap();

  /** @brief Get the best pose found from the last AttemptRecovery
   *  @return The pose */
  TooN::SE3<> NextPoseGuess();
  
  void Reset();
  
protected:

  void GatherWorkingIndices();

  Map &mMap;   ///< Reference to the Map
  TooN::Vector<6> mv6StepSizes;
  
  int mnDistFromCenter;
  std::vector<TooN::Vector<6, int> > mvWorkingIndices;
  int mnCurrIdx;
  TooN::Vector<6, int> mv6VolumeIdxSize;
};

#endif

