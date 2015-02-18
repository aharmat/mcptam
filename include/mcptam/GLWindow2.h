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
 * \file GLWindow2.h
 * \brief Declaration of GLWindow2 classes
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Most of this code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 ****************************************************************************************/

#ifndef __GL_WINDOW_2_H
#define __GL_WINDOW_2_H

#include <cvd/glwindow.h>
#include <TooN/TooN.h>

class GLWindowMenu;

struct MouseUpdate
{
  MouseUpdate()
  {
    Reset();
  }
  
  void Reset()
  {
    //mv2LeftClick = TooN::Zeros; 
    mv2MiddleClick = TooN::Zeros;
    mv2ShiftMiddleClick = TooN::Zeros;
    //mv2RightClick = TooN::Zeros;
    mdWheel = 0;  
  }
  
  //TooN::Vector<2> mv2LeftClick; 
  TooN::Vector<2> mv2MiddleClick;
  TooN::Vector<2> mv2ShiftMiddleClick;
  //TooN::Vector<2> mv2RightClick;
  double mdWheel; // + is up, - is down
};

/** @brief Wraps CVD::GLWindow and provides some basic user-interface functionality
 * 
 *  Implements a gvars-driven clickable menu, and a caption line for text display. 
 *  Also provides some handy GL helpers and a wrapper for CVD's text display routines. */
class GLWindow2 : public CVD::GLWindow, public CVD::GLWindow::EventHandler
{
public:
  GLWindow2(CVD::ImageRef irSize, std::string sTitle);
  
  // The preferred event handler..
  void HandlePendingEvents();
  
  // Menu interface:
  void AddMenu(std::string sName, std::string sTitle);
  void DrawMenus();
  
  // Some OpenGL helpers:
  void SetupViewport();
  void SetupVideoOrtho();
  void SetupUnitOrtho();
  void SetupWindowOrtho();
  void SetupVideoRasterPosAndZoom();

  // Text display functions:
  //void PrintString(CVD::ImageRef irPos, std::string s);
  void PrintString(CVD::ImageRef irPos, std::string s, double scale=8, double spacing=1.6, double kerning=0.1);
  void DrawCaption(std::string s);
  
  CVD::ImageRef GetWindowSize(){ return mirVideoSize; }
  CVD::ImageRef GetRealWindowSize();
  
  // Map viewer mouse interface:
  MouseUpdate GetMouseUpdate();
  
  CVD::ImageRef GetMousePos(){ return mirLastMousePos; }
  

protected:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  
  // User interface menus:
  std::vector<GLWindowMenu*> mvpGLWindowMenus;

  CVD::ImageRef mirVideoSize;   // The size of the source video material.
  
  // Event handling routines:
  virtual void on_key_down(GLWindow&, int key);
  virtual void on_key_up(GLWindow&, int key);
  virtual void on_mouse_move(GLWindow& win, CVD::ImageRef where, int state);
  virtual void on_mouse_down(GLWindow& win, CVD::ImageRef where, int state, int button);
  virtual void on_mouse_up(GLWindow& win, CVD::ImageRef where, int state, int button);
  virtual void on_event(GLWindow& win, int event);
  CVD::ImageRef mirLastMousePos;

  // Storage for map viewer updates:
  //TooN::Vector<6> mvMCPoseUpdate;
  //TooN::Vector<6> mvLeftPoseUpdate;
  
  MouseUpdate mMouseUpdate;
  
};


#endif

