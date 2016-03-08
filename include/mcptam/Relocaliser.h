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
 * \file Relocaliser.h
 * \brief Declares the Relocaliser class
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

#ifndef MCPTAM_RELOCALISER_H
#define MCPTAM_RELOCALISER_H

#include <string>
#include <mcptam/TaylorCamera.h>
#include <mcptam/Types.h>
#include <TooN/se3.h>
#include <TooN/se2.h>

class Map;
class KeyFrame;

/** @brief Attempts to find the best pose for a given KeyFrame by comparing to
 *others in the Map.
 *
 * Uses the SmallBlurryImage to find the best KeyFrame match, then computes the
 *2D transform between
 * them, then finally finds the 3D transform that would have generated that 2D
 *transform. This class
 * is essentially directly from PTAM */
class Relocaliser
{
public:
  /** @brief Used when there is only one camera in the system
   *  @param map The Map being worked on
   *  @param camera The TaylorCamera model
   *  @param camName The name of the camera */
  Relocaliser(Map &map, TaylorCamera camera, std::string camName);

  /** @param map The Map being worked on
   *  @param cameras The TaylorCamera models */
  Relocaliser(Map &map, TaylorCameraMap cameras);

  /** @brief Tries to find the correct pose of its argument KeyFrame by matching
   * SmallBlurryImages in the Map
   *  @param kfCurrent The KeyFrame whose pose needs to be found
   *  @return Was the recovery successful? */
  bool AttemptRecovery(KeyFrame &kfCurrent);

  /** @brief Get the best pose found from the last AttemptRecovery
   *  @return The pose */
  TooN::SE3<> BestPose();

  static double sdRecoveryMaxScore;  ///> Maximum score of SmallBlurryImage's
  /// IteratePosRelToTarget for the resulting
  /// pose to be considered good

protected:
  /** @brief Finds the best matching KeyFrame in the Map by zero-mean SSD, and
   * saves the resulting score
   *  @param kfCurrent The KeyFrame we need to find a match for */
  void ScoreKFs(KeyFrame &kfCurrent);

  Map &mMap;                       ///< Reference to the Map
  TaylorCameraMap mmCameraModels;  ///< All the camera models being used in the system

  KeyFrame *mpBestKF;    ///< Pointer to the best KeyFrame found
  double mdBestScore;    ///< The score of the best KeyFrame
  TooN::SE2<> mse2;      ///< The 2D pose transform of the last attempted recovery
  TooN::SE3<> mse3Best;  ///< The 3D pose transform of the last attempted recovery
};
#endif  // MCPTAM_RELOCALISER_H
