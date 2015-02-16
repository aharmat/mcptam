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
#include <mcptam/OpenGL.h>
#include <mcptam/GLWindow2.h>
#include <mcptam/GLWindowMenu.h>
#include <stdlib.h>
#include <gvars3/GStringUtil.h>
#include <gvars3/instances.h>
#include <TooN/helpers.h>
#include <ros/ros.h>

using namespace GVars3;
using namespace TooN;

GLWindow2::GLWindow2(CVD::ImageRef irSize, std::string sTitle)
  : GLWindow(irSize, sTitle)
{
  mirVideoSize = irSize;
  GUI.RegisterCommand("GLWindow.AddMenu", GUICommandCallBack, this);
  CVD::glSetFont("sans");
  
  //std::cout<< get_glx_version() <<std::endl;
};


void GLWindow2::AddMenu(std::string sName, std::string sTitle)
{
  GLWindowMenu* pMenu = new GLWindowMenu(sName, sTitle); 
  mvpGLWindowMenus.push_back(pMenu);
}

void GLWindow2::GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams)
{
  ((GLWindow2*) ptr)->GUICommandHandler(sCommand, sParams);
}

void GLWindow2::GUICommandHandler(std::string sCommand, std::string sParams)  // Called by the callback func..
{
  std::vector<std::string> vs=ChopAndUnquoteString(sParams);
  if(sCommand=="GLWindow.AddMenu")
  {
    switch(vs.size())
    {
      case 1:
        AddMenu(vs[0], "Root");
        return;
      case 2:
        AddMenu(vs[0], vs[1]);
        return;
      default:
        std::cout << "? AddMenu: need one or two params (internal menu name, [caption])." << std::endl;
        return;
    }
  }
  
  // Should have returned to caller by now - if got here, a command which 
  // was not handled was registered....
  std::cout << "! GLWindow::GUICommandHandler: unhandled command "<< sCommand << std::endl;
  ros::shutdown();
}; 

void GLWindow2::DrawMenus()
{
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_POLYGON_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColorMask(1,1,1,1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  SetupWindowOrtho();
  glLineWidth(1);
  
  int nTop = 30;
  int nHeight = 30;
  for(std::vector<GLWindowMenu*>::iterator i = mvpGLWindowMenus.begin(); i!= mvpGLWindowMenus.end(); i++)
  {
    (*i)->Render(nTop, nHeight, size()[0], *this);
    nTop+=nHeight+1;
  }
  
}

void GLWindow2::SetupUnitOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,1,1,0,0,1);
}

void GLWindow2::SetupWindowOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(size());
}

void GLWindow2::SetupVideoOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5,(double)mirVideoSize.x - 0.5, (double) mirVideoSize.y - 0.5, -0.5, -1.0, 1.0);
}

void GLWindow2::SetupVideoRasterPosAndZoom()
{ 
  glRasterPos2d(-0.5,-0.5);
  double adZoom[2];
  adZoom[0] = (double) size()[0] / (double) mirVideoSize[0];
  adZoom[1] = (double) size()[1] / (double) mirVideoSize[1];
  glPixelZoom(adZoom[0], -adZoom[1]);
}

void GLWindow2::SetupViewport()
{
  glViewport(0, 0, size()[0], size()[1]);
}

void GLWindow2::PrintString(CVD::ImageRef irPos, std::string s, double scale, double spacing, double kerning)
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glTranslatef(irPos.x, irPos.y, 0.0);
  glScalef(scale,-scale,1);
  CVD::glDrawText(s, CVD::NICE, spacing, kerning);
  glPopMatrix();
}

void GLWindow2::DrawCaption(std::string s)
{
  if(s.length() == 0)
    return;
  
  SetupWindowOrtho();
  // Find out how many lines are in the caption:
  // Count the endls
  int nLines = 0;
  {
    std::string sendl("\n");
    std::string::size_type st=0;
    while(1)
    {
      nLines++;
      st = s.find(sendl, st);
      if(st==std::string::npos)
        break;
      else
        st++;
    }
  }
  
  int nTopOfBox = size().y - nLines * 16;
  
  // Draw a grey background box for the text
  glColor4f(0,0,0,0.4);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glVertex2d(-0.5, nTopOfBox);
  glVertex2d(size().x, nTopOfBox);
  glVertex2d(size().x, size().y);
  glVertex2d(-0.5, size().y);
  glEnd();
  
  // Draw the caption text in yellow
  glColor3f(1,1,0);      
  PrintString(CVD::ImageRef(10,nTopOfBox + 13), s);
}


void GLWindow2::HandlePendingEvents()
{
  handle_events(*this);
}

void GLWindow2::on_mouse_move(GLWindow& win, CVD::ImageRef where, int state)
{
  CVD::ImageRef irMotion = where - mirLastMousePos;
  mirLastMousePos = where;
  
  if(state & BUTTON_LEFT && ! (state & BUTTON_RIGHT))
  {
    mMouseUpdate.mv2LeftClick[0] += irMotion[0];
    mMouseUpdate.mv2LeftClick[1] += irMotion[1];
  }
  else if(!(state & BUTTON_LEFT) && state & BUTTON_RIGHT)
  {
    mMouseUpdate.mv2RightClick[0] += irMotion[0];
    mMouseUpdate.mv2RightClick[1] += irMotion[1];
  }
  else if(state & BUTTON_MIDDLE  || (state & BUTTON_LEFT && state & BUTTON_RIGHT))
  {
    mMouseUpdate.mv2MiddleClick[0] += irMotion[0];
    mMouseUpdate.mv2MiddleClick[1] += irMotion[1];
  }
  
}

void GLWindow2::on_mouse_down(GLWindow& win, CVD::ImageRef where, int state, int button)
{
  bool bHandled = false;
  for(unsigned int i=0; !bHandled && i<mvpGLWindowMenus.size(); i++)
    bHandled = mvpGLWindowMenus[i]->HandleClick(button, state, where.x, where.y);
    
  if(button & BUTTON_WHEEL_UP)
  {
    mMouseUpdate.mdWheel += 1.0;
    bHandled = true;
  }
  else if(button & BUTTON_WHEEL_DOWN)
  {
    mMouseUpdate.mdWheel -= 1.0;
    bHandled = true;
  }
    
  if(!bHandled)  // button press wasn't over any menu item
  {
    std::stringstream ss;
    ss<<button<<" "<<state<<" "<<where.x<<" "<<where.y;
    GUI.ParseLine("try MouseDown "+ss.str());
  }
}

void GLWindow2::on_event(GLWindow& win, int event)
{
  if(event==EVENT_CLOSE)
    GUI.ParseLine("quit");
}

MouseUpdate GLWindow2::GetMouseUpdate()
{
  MouseUpdate ret = mMouseUpdate;
  mMouseUpdate.Reset();
  
  return ret;
}

#include <X11/keysym.h>
void GLWindow2::on_key_down(GLWindow&, int k)
{
  std::string s;
  switch(k)
  {
    case XK_a:   case XK_A:  s="a"; break;
    case XK_b:   case XK_B:  s="b"; break;
    case XK_c:   case XK_C:  s="c"; break;
    case XK_d:   case XK_D:  s="d"; break;
    case XK_e:   case XK_E:  s="e"; break;
    case XK_f:   case XK_F:  s="f"; break;
    case XK_g:   case XK_G:  s="g"; break;
    case XK_h:   case XK_H:  s="h"; break;
    case XK_i:   case XK_I:  s="i"; break;
    case XK_j:   case XK_J:  s="j"; break;
    case XK_k:   case XK_K:  s="k"; break;
    case XK_l:   case XK_L:  s="l"; break;
    case XK_m:   case XK_M:  s="m"; break;
    case XK_n:   case XK_N:  s="n"; break;
    case XK_o:   case XK_O:  s="o"; break;
    case XK_p:   case XK_P:  s="p"; break;
    case XK_q:   case XK_Q:  s="q"; break;
    case XK_r:   case XK_R:  s="r"; break;
    case XK_s:   case XK_S:  s="s"; break;
    case XK_t:   case XK_T:  s="t"; break;
    case XK_u:   case XK_U:  s="u"; break;
    case XK_v:   case XK_V:  s="v"; break;
    case XK_w:   case XK_W:  s="w"; break;
    case XK_x:   case XK_X:  s="x"; break;
    case XK_y:   case XK_Y:  s="y"; break;
    case XK_z:   case XK_Z:  s="z"; break;
    case XK_1:   s="1"; break;
    case XK_2:   s="2"; break;
    case XK_3:   s="3"; break;
    case XK_4:   s="4"; break;
    case XK_5:   s="5"; break;
    case XK_6:   s="6"; break;
    case XK_7:   s="7"; break;
    case XK_8:   s="8"; break;
    case XK_9:   s="9"; break;
    case XK_0:   s="0"; break;
    case XK_KP_Prior: case XK_Page_Up:     s="PageUp"; break;
    case XK_KP_Next:  case XK_Page_Down:   s="PageDown"; break;
    case XK_Return: s="Enter"; break;
    case XK_space:  s="Space"; break;
    case XK_BackSpace:  s="BackSpace"; break;
    case XK_Escape:  s="Escape"; break;
    default: ;
  }

  if(s!="")
    GUI.ParseLine("try KeyPress "+s);
}
