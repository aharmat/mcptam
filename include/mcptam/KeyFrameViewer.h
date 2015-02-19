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
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Defines KeyFrameViewer, a class that allows the viewing of the images making up
 * KeyFrames in the Map.
 *
 ****************************************************************************************/

#ifndef __KEYFRAME_VIEWER_H
#define __KEYFRAME_VIEWER_H

#include <mcptam/Types.h>
#include <mcptam/EditAction.h>
#include <sstream>

class Map;
class GLWindow2;

/** @brief Allows the viewing of the images making up KeyFrames in the Map
 * 
 *  Draws all KeyFrames in one MultiKeyFrame together, along with the detected corners. Scrolling through
 *  MultiKeyFrames is accomplished with the Next and Prev functions. */
class KeyFrameViewer
{
public:

  KeyFrameViewer(Map &map, GLWindow2 &glw);
  void Init();
  void Draw();
  bool GUICommandHandler(std::string command, std::string params, std::shared_ptr<EditAction>& pAction);
  std::string GetMessageForUser();
  
protected:

  std::vector<MapPoint*> GatherVisiblePoints(int nPointVis);
  std::vector<KeyFrame*> GatherKeyFrames(std::vector<MapPoint*> vpPoints, bool bSource);
  
  CVD::Image<CVD::byte> ResizeImageToWindow(CVD::Image<CVD::byte> imOrig, double dWidthFrac);
  
  void ToggleAllSourcePoints(bool bForceUnselect);
  void ToggleSourceSelection(CVD::ImageRef irPixel);
  void SetSourceSelectionInArea(CVD::ImageRef irBegin, CVD::ImageRef irEnd, bool bSelected);
  
  CVD::ImageRef ClampLocToSource(CVD::ImageRef irLoc);
  void DrawCrosshairs(CVD::ImageRef irPos, TooN::Vector<4> v4Color, float fLineWidth);
  void DrawRectangle(CVD::ImageRef irBegin, CVD::ImageRef irEnd, TooN::Vector<4> v4Color, float fLineWidth);
  
  std::vector<MapPoint*> GatherSelected();
  
  void InitOrthoDrawing();

  Map &mMap;   ///< Reference to the Map
  GLWindow2 &mGLWindow;  ///< Reference to the GL window
  
  int mnVerticalDrawOffset;
  double mdPointSizeFrac;
  
  int mnPointVis;
  
  int mnSourceIdx;         ///< Current MultiKeyFrame's index, do this instead of keeping iterator because iterators can be invalidated by deletion of MultiKeyFrame
  int mnTargetIdx;
  int mnTargetSearchDir;
  
  std::vector<KeyFrame*> mvpTargetKeyFrames;
  std::vector<KeyFrame*> mvpSourceKeyFrames;

  TooN::Matrix<2> mm2Scale;
  
  std::ostringstream mMessageForUser;   ///< Message stream for user
  
  enum SelectionMode{SINGLE, BOX_SELECT, BOX_UNSELECT} mSelectionMode;
  enum SelectionStatus{READY, SELECTING} mSelectionStatus;
  
  double mdSelectionThresh;
  CVD::ImageRef mirSelectionBegin;
  CVD::ImageRef mirSelectionCursor;
  
  CVD::Image<CVD::byte> mimSource;
  CVD::ImageRef mirSourceOffset;
  
  double mdPointRadius;
  
};

#endif

