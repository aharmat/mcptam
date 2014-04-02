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
 * \file GLWindowMenu.h
 * \brief Declaration of GLWindowMenu classes
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Most of this code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 ****************************************************************************************/

#ifndef __GL_WINDOW_MENU_H
#define __GL_WINDOW_MENU_H

#include <mcptam/GLWindow2.h>
#include <vector>
#include <map>
#include <gvars3/gvars3.h>

/** @brief A simple gvars-driven menu system for GLWindow2
 * 
 *  N.b. each GLWindowMenu class internally contains sub-menus */
class GLWindowMenu
{
 public:
  
  GLWindowMenu(std::string sName, std::string sTitle);
  ~GLWindowMenu();
  void Render(int nTop, int nHeight, int nWidth, GLWindow2 &glw);
  void FillBox(int l, int r, int t, int b);
  void LineBox(int l, int r, int t, int b);
  
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  
  bool HandleClick(int button, int state, int x, int y);

 private:
  enum MenuItemType { Button, Toggle, Monitor, Slider };
  
  struct MenuItem
  {
    MenuItemType type;
    std::string sName;
    std::string sParam;
    std::string sNextMenu;
    GVars3::gvar2_int gvnIntValue;  // Not used by all, but used by some
    int min;
    int max;
  };
  
  struct SubMenu
  {
    std::vector<MenuItem> mvItems;
  };
  
  std::map<std::string, SubMenu> mmSubMenus;
  //std::string msCurrentSubMenu;
  std::string msName;
  std::string msTitle;

  
  int mnWidth;
  int mnMenuTop;
  int mnMenuHeight;
  int mnTextOffset;
  
  GVars3::gvar2_int mgvnEnabled;
  GVars3::gvar2_int mgvnMenuItemWidth;
  GVars3::gvar2_int mgvnMenuTextOffset;
  GVars3::gvar2_string mgvsCurrentSubMenu;
  
  int mnLeftMostCoord;
  
};

#endif

