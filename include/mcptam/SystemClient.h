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
 * \file SystemClient.h
 * \brief Declaration of SystemClient class
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * Defines the rest of the objects necessary to run mcptam_client. Provides the
 *blocking
 * Run function which loops until exiting, acquiring images, calling the
 *tracker,
 * and drawing to the window.
 *
 ****************************************************************************************/

#ifndef MCPTAM_SYSTEMCLIENT_H
#define MCPTAM_SYSTEMCLIENT_H

#include <mcptam/SystemFrontendBase.h>
#include <string>
// #include <ros/ros.h>

class MapMakerClient;

/** @brief Implements the rest of the objects necssary to run MCTAM_Client */
class SystemClient : public SystemFrontendBase
{
public:
  /// Creates objects, sets up GUI
  SystemClient();

  /// Destructor
  ~SystemClient();

  /** @brief Blocking function that loops indefinitiely
   *
   *  Acquires images, calls the tracker, and draws to the window.
   *MapMakerClient runs
   *  in its own thread in the background, so we don't have to call it here */
  void Run();

protected:
  /** @brief Deals with user interface commands
   *  @param command The saved command
   *  @param params The saved command parameters */
  void GUICommandHandler(std::string command, std::string params);

  MapMakerClient *mpMapMakerClient;  ///< Pointer to the MapMakerClient
};

#endif  // MCPTAM_SYSTEMCLIENT_H
