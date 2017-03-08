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

// Copyright 2008 Isis Innovation Limited
#include <mcptam/GLWindowMenu.h>
#include <mcptam/OpenGL.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>
#include <sstream>
#include <string>
#include <vector>

using GVars3::GUI;
using GVars3::GV2;
using GVars3::HIDDEN;
using GVars3::SILENT;

GLWindowMenu::GLWindowMenu(std::string sName, std::string sTitle)
{
  msName = sName;
  msTitle = sTitle;

  GUI.RegisterCommand(msName + ".AddMenuButton", GUICommandCallBack, this);
  GUI.RegisterCommand(msName + ".AddMenuToggle", GUICommandCallBack, this);
  GUI.RegisterCommand(msName + ".AddMenuSlider", GUICommandCallBack, this);
  GUI.RegisterCommand(msName + ".AddMenuMonitor", GUICommandCallBack, this);
  GUI.RegisterCommand(msName + ".ShowMenu", GUICommandCallBack, this);
  GV2.Register(mgvnMenuItemWidth, msName + ".MenuItemWidth", 90, HIDDEN | SILENT);
  GV2.Register(mgvnMenuTextOffset, msName + ".MenuTextOffset", 20, HIDDEN | SILENT);
  GV2.Register(mgvnEnabled, msName + ".Enabled", 1, HIDDEN | SILENT);
  GV2.Register(mgvsCurrentSubMenu, msName + ".CurrentSubMenu", "", HIDDEN | SILENT);

  mmSubMenus.clear();
  *mgvsCurrentSubMenu = "";
}

GLWindowMenu::~GLWindowMenu()
{
  GUI.UnRegisterCommand(msName + ".AddMenuButton");
  GUI.UnRegisterCommand(msName + ".AddMenuToggle");
  GUI.UnRegisterCommand(msName + ".AddMenuSlider");
  GUI.UnRegisterCommand(msName + ".AddMenuMonitor");
  GUI.UnRegisterCommand(msName + ".ShowMenu");
};

void GLWindowMenu::GUICommandCallBack(void *ptr, std::string sCommand, std::string sParams)
{
  (reinterpret_cast<GLWindowMenu *>(ptr))->GUICommandHandler(sCommand, sParams);
}

void GLWindowMenu::GUICommandHandler(std::string sCommand, std::string sParams)
{
  std::vector<std::string> vs = GVars3::ChopAndUnquoteString(sParams);

  if (sCommand == msName + ".AddMenuButton")
  {
    if (vs.size() < 3)
    {
      std::cout << "? GLWindowMenu.AddMenuButton: Need 3/4 params: Target Menu, Name, Command , NextMenu=\"\""
                << std::endl;
      return;
    }
    MenuItem m;
    m.type = Button;
    m.sName = vs[1];
    m.sParam = GVars3::UncommentString(vs[2]);
    m.sNextMenu = (vs.size() > 3) ? (vs[3]) : ("");
    mmSubMenus[vs[0]].mvItems.push_back(m);
    return;
  }

  if (sCommand == msName + ".AddMenuToggle")
  {
    if (vs.size() < 3)
    {
      std::cout << "? GLWindowMenu.AddMenuToggle: Need 3/4 params: Target Menu, Name, gvar_int name , NextMenu=\"\""
                << std::endl;
      return;
    }
    MenuItem m;
    m.type = Toggle;
    m.sName = vs[1];
    GV2.Register(m.gvnIntValue, vs[2]);
    m.sNextMenu = (vs.size() > 3) ? (vs[3]) : ("");
    mmSubMenus[vs[0]].mvItems.push_back(m);
    return;
  }

  if (sCommand == msName + ".AddMenuMonitor")
  {
    if (vs.size() < 3)
    {
      std::cout << "? GLWindowMenu.AddMenuMonitor: Need 3/4 params: Target Menu, Name, gvar name , NextMenu=\"\""
                << std::endl;
      return;
    }
    MenuItem m;
    m.type = Monitor;
    m.sName = vs[1];
    m.sParam = vs[2];
    m.sNextMenu = (vs.size() > 3) ? (vs[3]) : ("");
    mmSubMenus[vs[0]].mvItems.push_back(m);
    return;
  }

  if (sCommand == msName + ".AddMenuSlider")
  {
    if (vs.size() < 5)
    {
      std::cout << "? GLWindowMenu.AddMenuSlider: Need 5/6 params: Target Menu, Name, gvar_int name, min, max, "
                "NextMenu=\"\"" << std::endl;
      return;
    }
    MenuItem m;
    m.type = Slider;
    m.sName = vs[1];
    GV2.Register(m.gvnIntValue, vs[2]);
    int *a;
    a = GVars3::ParseAndAllocate<int>(vs[3]);
    if (a)
    {
      m.min = *a;
      delete a;
    }
    a = GVars3::ParseAndAllocate<int>(vs[4]);
    if (a)
    {
      m.max = *a;
      delete a;
    }
    m.sNextMenu = (vs.size() > 5) ? (vs[5]) : ("");
    mmSubMenus[vs[0]].mvItems.push_back(m);
    return;
  }

  if (sCommand == msName + ".ShowMenu")
  {
    if (vs.size() == 0)
      *mgvsCurrentSubMenu = "";
    else
      *mgvsCurrentSubMenu = vs[0];
  }
}

void GLWindowMenu::LineBox(int l, int r, int t, int b)
{
  glBegin(GL_LINE_STRIP);
  glVertex2i(l, t);
  glVertex2i(l, b);
  glVertex2i(r, b);
  glVertex2i(r, t);
  glVertex2i(l, t);
  glEnd();
}

void GLWindowMenu::FillBox(int l, int r, int t, int b)
{
  glBegin(GL_QUADS);
  glVertex2i(l, t);
  glVertex2i(l, b);
  glVertex2i(r, b);
  glVertex2i(r, t);
  glEnd();
}

void GLWindowMenu::Render(int nTop, int nHeight, int nWidth, GLWindow2 &glw)
{
  if (!*mgvnEnabled)
    return;

  mnWidth = nWidth;
  mnMenuTop = nTop;
  mnMenuHeight = nHeight;

  double dAlpha = 0.8;
  if (*mgvsCurrentSubMenu == "")  // No Menu selected  - draw little arrow.
  {
    glColor4d(0, 0.5, 0, 0.5);
    FillBox(mnWidth - 30, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
    glColor4d(0, 1, 0, 0.5);
    LineBox(mnWidth - 30, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
    mnLeftMostCoord = mnWidth - 30;
    return;
  }

  SubMenu &m = mmSubMenus[*mgvsCurrentSubMenu];

  mnLeftMostCoord = mnWidth - (1 + m.mvItems.size()) **mgvnMenuItemWidth;
  int nBase = mnLeftMostCoord;
  for (std::vector<MenuItem>::reverse_iterator i = m.mvItems.rbegin(); i != m.mvItems.rend(); i++)
  {
    switch (i->type)
    {
    case Button:
      glColor4d(0, 0.5, 0, dAlpha);
      FillBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glColor4d(0, 1, 0, dAlpha);
      LineBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glw.PrintString(CVD::ImageRef(nBase + 3, mnMenuTop + *mgvnMenuTextOffset), i->sName);
      break;

    case Toggle:
      if (*(i->gvnIntValue))
        glColor4d(0, 0.5, 0.5, dAlpha);
      else
        glColor4d(0.5, 0, 0, dAlpha);
      FillBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      if (*(i->gvnIntValue))
        glColor4d(0, 1, 0.5, dAlpha);
      else
        glColor4d(1, 0, 0, dAlpha);
      LineBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glw.PrintString(CVD::ImageRef(nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
                      i->sName + " " + ((*(i->gvnIntValue)) ? ("On") : ("Off")));
      break;

    case Monitor:
      glColor4d(0, 0.5, 0.5, dAlpha);
      FillBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glColor4d(0, 1, 1, dAlpha);
      LineBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glw.PrintString(CVD::ImageRef(nBase + 3, mnMenuTop + *mgvnMenuTextOffset),
                      i->sName + " " + GV2.StringValue(i->sParam, true));
      break;

    case Slider:
    {
      glColor4d(0.0, 0.0, 0.5, dAlpha);
      FillBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      glColor4d(0.5, 0.0, 0.5, dAlpha);
      double dFrac = static_cast<double>((*(i->gvnIntValue) - i->min)) / (i->max - i->min);
      if (dFrac < 0.0)
        dFrac = 0.0;
      if (dFrac > 1.0)
        dFrac = 1.0;
      FillBox(nBase, static_cast<int>(nBase + dFrac * (*mgvnMenuItemWidth + 1)), mnMenuTop, mnMenuTop + mnMenuHeight);
      glColor4d(0, 1, 1, dAlpha);
      LineBox(nBase, nBase + *mgvnMenuItemWidth + 1, mnMenuTop, mnMenuTop + mnMenuHeight);
      std::ostringstream ost;
      ost << i->sName << " " << *(i->gvnIntValue);
      glw.PrintString(CVD::ImageRef(nBase + 3, mnMenuTop + *mgvnMenuTextOffset), ost.str());
    }
    break;
    }
    nBase += *mgvnMenuItemWidth;
  }
  glColor4d(0.5, 0.5, 0, dAlpha);
  FillBox(mnWidth - *mgvnMenuItemWidth, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
  glColor4d(1, 1, 0, dAlpha);
  LineBox(mnWidth - *mgvnMenuItemWidth, mnWidth - 1, mnMenuTop, mnMenuTop + mnMenuHeight);
  CVD::ImageRef ir(mnWidth - *mgvnMenuItemWidth + 5, mnMenuTop + *mgvnMenuTextOffset);
  if (*mgvsCurrentSubMenu == "Root")
    glw.PrintString(ir, msTitle + ":");
  else
    glw.PrintString(ir, *mgvsCurrentSubMenu + ":");
}

bool GLWindowMenu::HandleClick(int nMouseButton, int state, int x, int y)
{
  if (!*mgvnEnabled)
    return false;

  if ((y < mnMenuTop) || (y > mnMenuTop + mnMenuHeight))
    return false;
  if (x < mnLeftMostCoord)
    return false;

  // if no menu displayed, then must display root menu!
  if (*mgvsCurrentSubMenu == "")
  {
    *mgvsCurrentSubMenu = "Root";
    return true;
  }

  // Figure out which button was pressed:
  int nButtonNumber = (mnWidth - x) / *mgvnMenuItemWidth;
  if (nButtonNumber > static_cast<int>(mmSubMenus[*mgvsCurrentSubMenu].mvItems.size()))
    nButtonNumber = 0;

  if (nButtonNumber == 0)  // Clicked on menu name .. . go to root.
  {
    if (*mgvsCurrentSubMenu == "Root")
      *mgvsCurrentSubMenu = "";
    else
      *mgvsCurrentSubMenu = "Root";
    return true;
  }

  MenuItem SelectedItem = mmSubMenus[*mgvsCurrentSubMenu].mvItems[nButtonNumber - 1];
  *mgvsCurrentSubMenu = SelectedItem.sNextMenu;
  switch (SelectedItem.type)
  {
  case Button:
    GUI.ParseLine(SelectedItem.sParam);
    break;
  case Toggle:
    *(SelectedItem.gvnIntValue) ^= 1;
    break;
  case Slider:
  {
    if (nMouseButton == CVD::GLWindow::BUTTON_WHEEL_UP)
    {
      *(SelectedItem.gvnIntValue) += 1;
      if (*(SelectedItem.gvnIntValue) > SelectedItem.max)
        *(SelectedItem.gvnIntValue) = SelectedItem.max;
    }
    else if (nMouseButton == CVD::GLWindow::BUTTON_WHEEL_DOWN)
    {
      *(SelectedItem.gvnIntValue) -= 1;
      if (*(SelectedItem.gvnIntValue) < SelectedItem.min)
        *(SelectedItem.gvnIntValue) = SelectedItem.min;
    }
    else
    {
      int nPos = *mgvnMenuItemWidth - ((mnWidth - x) % *mgvnMenuItemWidth);
      double dFrac = static_cast<double>(nPos) / *mgvnMenuItemWidth;
      *(SelectedItem.gvnIntValue) =
        static_cast<int>(dFrac * (1.0 + SelectedItem.max - SelectedItem.min)) + SelectedItem.min;
    }
    break;
  }
  case Monitor:
    break;
  }
  return true;
}
