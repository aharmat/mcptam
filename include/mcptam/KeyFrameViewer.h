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
 * \file KeyFrameViewer.h
 * \brief Declaration of KeyFrameViewer class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * Defines KeyFrameViewer, a class that allows the viewing of the images making
 *up
 * KeyFrames in the Map.
 *
 ****************************************************************************************/

#ifndef MCPTAM_KEYFRAMEVIEWER_H
#define MCPTAM_KEYFRAMEVIEWER_H

#include <mcptam/Types.h>
#include <sstream>
#include <string>

class Map;
class GLWindow2;

/** @brief Allows the viewing of the images making up KeyFrames in the Map
 *
 *  Draws all KeyFrames in one MultiKeyFrame together, along with the detected
 *corners. Scrolling through
 *  MultiKeyFrames is accomplished with the Next and Prev functions. */
class KeyFrameViewer
{
public:
  /** @brief Only parameterized constructor to ensure a valid object
   *  @param map The Map being used
   *  @param glw The GL window object to draw to
   *  @param mDrawOffsets The draw offsets for each camera, determines where in
   * the window each KeyFrame is drawn */
  KeyFrameViewer(Map &map, GLWindow2 &glw, ImageRefMap mDrawOffsets, ImageRefMap mSizes);

  /// Draw the current MultiKeyFrame
  void Draw();

  /// Select the next MultiKeyFrame in the Map
  void Next();

  /// Select the previous MultiKeyFrame in the Map
  void Prev();

  /// Contains information that should be displayed to the user
  /** @return The message string */
  std::string GetMessageForUser();

protected:
  Map &mMap;                  ///< Reference to the Map
  GLWindow2 &mGLWindow;       ///< Reference to the GL window
  ImageRefMap mmDrawOffsets;  ///< The drawing offsets
  ImageRefMap mmSizes;        ///< The image offsets

  int nCurrentIdx;  ///< Current MultiKeyFrame's index, do this instead of
  /// keeping iterator because iterators can be invalidated by
  /// deletion of MultiKeyFrame

  std::ostringstream mMessageForUser;  ///< Message stream for user
};

#endif  // MCPTAM_KEYFRAMEVIEWER_H
