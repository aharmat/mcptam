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


//=========================================================================================
//
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Parts of this code are from the original PTAM, which are
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/MapEditor.h>
#include <mcptam/MapPoint.h>
#include <mcptam/OpenGL.h>
#include <mcptam/Map.h>
#include <mcptam/Utility.h>
#include <mcptam/DeletePointsAction.h>
#include <mcptam/FitGroundPlaneAction.h>
#include <cvd/image_io.h>
#include <stdlib.h>
#include <fstream>
#include <gvars3/instances.h>

using namespace GVars3;

MapEditor::MapEditor(std::string windowName)
: mNodeHandlePriv("~")
, mpGLWindow(InitWindow(windowName))
{
  glewInit();
  
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("MouseDown", GUICommandCallBack, this);
  GUI.RegisterCommand("MouseUp", GUICommandCallBack, this);
  GUI.RegisterCommand("MouseMove", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyRelease", GUICommandCallBack, this);
  
  GUI.ParseLine("Layer1=1");
  GUI.ParseLine("Layer2=0");
  GUI.ParseLine("Layer3=0");
  GUI.ParseLine("Layer4=0");
  
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  
  GUI.ParseLine("Menu.AddMenuToggle Root \"Layer 4\" Layer4 Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Layer 3\" Layer3 Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Layer 2\" Layer2 Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Layer 1\" Layer1 Root");
    
  mNodeHandle.setCallbackQueue(&mCallbackQueueROS);
  mNodeHandlePriv.setCallbackQueue(&mCallbackQueueROS);
  
  mNodeHandlePriv.getParam("save_folder", mSaveFolder);
  ROS_ASSERT(!mSaveFolder.empty());
  
  LoadCamerasFromFolder(mSaveFolder);
  
  mpMap = new Map;  
  mpMap->LoadFromFolder(mSaveFolder, mmPoses, mmCameraModels, false);
  
  PutPointsOnLayer(1, false);
  
  mpMapViewer = new MapViewer(*mpMap, *mpGLWindow);
  
  mdSelectionThresh = 1.0;
  mSelectionMode = SINGLE;
  mSelectionStatus = READY;
  
  mbCtrl = false;
  
  mbDone = false;
}

MapEditor::~MapEditor()
{
  // This is needed because we're hijacking the mnUsing field of MapPoint to 
  // keep track of what layer it's on, and deleting the Map waits for all the
  // mnUsing counts to go to zero before it can delete the map points
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mnUsing = 0;
  }
  
  delete mpGLWindow;
  delete mpMap;
  delete mpMapViewer;
}

// This can be used with GUI.RegisterCommand to capture user input
void MapEditor::GUICommandCallBack(void *ptr, std::string command, std::string params)
{
  Command c;
  c.command = command;
  c.params = params;
  static_cast<MapEditor*>(ptr)->mqCommands.push(c);
}

// Creates a new GLWindow2 object
GLWindow2* MapEditor::InitWindow(std::string windowName)
{
  return new GLWindow2(CVD::ImageRef(800,600), windowName);
}

void MapEditor::LoadCamerasFromFolder(std::string folder)
{
  std::string calibrationsFile = folder + "/calibrations.dat";
  std::ifstream file(calibrationsFile);
  if(!file.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<calibrationsFile<<"]");
    ros::shutdown();
    return;
  }
  
  std::string readline;
  std::stringstream readlineSS;
  
  std::string conversion;
  std::stringstream conversionSS;
  
  // First three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of cameras
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numCams = 0;
  readlineSS>>numCams;
  
  std::cout<<"Reading "<<numCams<<" camera models"<<std::endl;
  
  for(int i=0; i < numCams; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // Camera Name, image size (2 vector), projection center (2 vector), polynomial coefficients (5 vector), affine matrix components (3 vector), inverse polynomial coefficents (variable size)
      
      // First is camera name
      std::string camName;
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> camName;
      
      std::cout<<"Read "<<camName<<std::endl;
      
      CVD::ImageRef irImageSize = CVD::ir(util::readVector<2>(readlineSS));
      TooN::Vector<2> v2Center = util::readVector<2>(readlineSS);
      TooN::Vector<5> v5Coeffs = util::readVector<5>(readlineSS);
      TooN::Vector<3> v3Affine = util::readVector<3>(readlineSS);
      
      TooN::Vector<9> v9Params;
      
      // The parameters (by index) are:
      // 0 - a0 coefficient 
      // 1 - a2 coefficient 
      // 2 - a3 coefficient 
      // 3 - a4 coefficient 
      // 4 - center of projection xc 
      // 5 - center of projection yc 
      // 6 - affine transform param c 
      // 7 - affine transform param d
      // 8 - affine transform param e
      
      v9Params[0] = v5Coeffs[0];
      v9Params[1] = v5Coeffs[2];
      v9Params[2] = v5Coeffs[3];
      v9Params[3] = v5Coeffs[4];
      v9Params[4] = v2Center[0];
      v9Params[5] = v2Center[1];
      v9Params[6] = v3Affine[0];
      v9Params[7] = v3Affine[1];
      v9Params[8] = v3Affine[2];
      
      mmCameraModels.insert(std::pair<std::string,TaylorCamera>(camName, TaylorCamera(v9Params, irImageSize, irImageSize, irImageSize)));
    }
    else
    {
      ROS_FATAL_STREAM("Error reading camera calibration file, bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mmCameraModels.size()<<" camera models"<<std::endl;
  
  file.close();
  
  std::string posesFile = folder + "/poses.dat";
  file.open(posesFile);
  if(!file.is_open())
  {
    ROS_FATAL_STREAM("Couldn't open file ["<<posesFile<<"]");
    ros::shutdown();
    return;
  }
  
  // First three lines are comments
  std::getline(file, readline);
  std::getline(file, readline);
  std::getline(file, readline);
  
  // Next line is number of poses
  std::getline(file,readline);
  readlineSS.clear();
  readlineSS.str(readline);
  
  int numPoses = 0;
  readlineSS>>numPoses;
  
  std::cout<<"Reading "<<numPoses<<" poses"<<std::endl;
  
  for(int i=0; i < numPoses; ++i)
  {
    if(std::getline(file, readline))
    {
      readlineSS.clear();
      readlineSS.str(readline);
      
      // First is camera name
      std::string camName;
      std::getline(readlineSS,conversion,',');
      conversionSS.clear();
      conversionSS.str(conversion);
      conversionSS >> camName;
      
      std::cout<<"Got cam name: "<<camName<<std::endl;
      
      // Next is pose
      mmPoses[camName] = (util::readSE3(readlineSS)).inverse();
    }
    else
    {
      ROS_FATAL_STREAM("Error reading camera poses file, bailing");
      ros::shutdown();
      return;
    }
  }
  
  std::cout<<"Got "<<mmPoses.size()<<" poses"<<std::endl;
}

void MapEditor::InitOrthoDrawing()
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
  mpGLWindow->SetupWindowOrtho();
}

void MapEditor::DrawCrosshairs(CVD::ImageRef irPos, TooN::Vector<4> v4Color, float fLineWidth)
{
  int nWidth = mpGLWindow->size().x;
  int nHeight = mpGLWindow->size().y;
  
  InitOrthoDrawing();
  
  glLineWidth(fLineWidth);
  CVD::glColor(v4Color);
  CVD::glLine(CVD::ImageRef(0,irPos.y), CVD::ImageRef(nWidth, irPos.y));
  CVD::glLine(CVD::ImageRef(irPos.x,0), CVD::ImageRef(irPos.x, nHeight));
}

void MapEditor::DrawRectangle(CVD::ImageRef irBegin, CVD::ImageRef irEnd, TooN::Vector<4> v4Color, float fLineWidth)
{
  InitOrthoDrawing();
  
  glLineWidth(fLineWidth);
  CVD::glColor(v4Color);
  
  glBegin(GL_LINE_LOOP);
  glVertex(irBegin);
  glVertex(CVD::ImageRef(irEnd.x, irBegin.y));
  glVertex(irEnd);
  glVertex(CVD::ImageRef(irBegin.x, irEnd.y));
  glEnd();
}

void MapEditor::Run()
{
  static gvar3<int> gvnLayer1("Layer1", 1, HIDDEN|SILENT);
  static gvar3<int> gvnLayer2("Layer2", 0, HIDDEN|SILENT);
  static gvar3<int> gvnLayer3("Layer3", 0, HIDDEN|SILENT);
  static gvar3<int> gvnLayer4("Layer4", 0, HIDDEN|SILENT);
  
  // Loop until instructed to stop
  while(!mbDone && ros::ok())
  {
    // Required before drawing
    mpGLWindow->SetupViewport();  
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();
    
    // Clear screen with black
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT);
    
    std::stringstream captionStream;
    
    int nPointVis = (*gvnLayer1 << 0) | (*gvnLayer2 << 1) | (*gvnLayer3 << 2) | (*gvnLayer4 << 3);
    
    mpMapViewer->DrawMap(nPointVis);
    captionStream << mpMapViewer->GetMessageForUser();
    captionStream << std::endl << "Selection Mode: ";
    
    if(mSelectionMode == SINGLE)
    {
      captionStream << "SINGLE";
    }
    else if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
    {
      TooN::Vector<4> v4Color;
      captionStream << "BOX ";
      
      if(mSelectionMode == BOX_SELECT)
      {
        captionStream << "SELECT";
        v4Color = TooN::makeVector(0.5, 1, 0.5, 0.8);  // greenish
      }
      else
      {
        captionStream << "UN-SELECT";
        v4Color = TooN::makeVector(1, 0.5, 0.5, 0.8);  // reddish
      }
      
      if(mSelectionStatus == READY)
      {
        DrawCrosshairs(mirSelectionCursor, v4Color, 2);
      }
      else if(mSelectionStatus == SELECTING)
      {
        DrawRectangle(mirSelectionBegin, mirSelectionCursor, v4Color, 2);
      }
    }
    
    captionStream << std::endl <<"Move to Layer [1] [2] [3] [4]";
    
    /*
    TooN::Vector<2> v2Projected;
    bool bSuccess = mpMapViewer->ProjectPoint((*mpMap->mlpPoints.begin())->mv3WorldPos, v2Projected);
    
    if(bSuccess)
      captionStream << std::endl << "First point projected into viewer: "<<v2Projected;
    else
      captionStream << std::endl << "First point does not project into viewer";
    */
    
    mpGLWindow->DrawCaption(captionStream.str());
    mpGLWindow->DrawMenus();
    mpGLWindow->swap_buffers();
    mpGLWindow->HandlePendingEvents();
    
    // Handle any remaining commands from the GUI interface
    while(!mqCommands.empty())
    {
      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }
    
    mCallbackQueueROS.callAvailable();
  }
}

void MapEditor::GUICommandHandler(std::string command, std::string params)  
{
  if(command=="quit" || command == "exit")
  {
    mbDone = true;
    return;
  }
  
  if(command=="KeyPress")
  {
    if(params == "b")
    {
      if(mSelectionMode == SINGLE || mSelectionMode == BOX_UNSELECT)
      {
        mSelectionMode = BOX_SELECT;
      }
      else if(mSelectionMode == BOX_SELECT)
      {
        std::cout<<"Just set BOX_UNSELECT"<<std::endl;
        mSelectionMode = BOX_UNSELECT;
      }
      
      mSelectionStatus = READY;
      mirSelectionCursor = mpGLWindow->cursor_position();
    }
    else if(params == "Escape")
    {
      mSelectionMode = SINGLE;
    }
    else if(params == "a")
    {
      ToggleAllPoints();
    }
    else if(params == "Delete")
    {
      DeleteSelected();
    }
    else if(params == "g")
    {
      // Ground plane align
      FitGroundPlaneToSelected();
    }
    else if(params == "Ctrl")
    {
      mbCtrl = true;
    }
    else if(params == "z")
    {
      if(mbCtrl)
      {
        std::cout<<"UNDO"<<std::endl;
        
        if(!mspUndoStack.empty())
        {
          std::shared_ptr<EditAction> pAction = mspUndoStack.top();
          pAction->Undo();
          
          mspRedoStack.push(pAction);
          mspUndoStack.pop();
        }
      }
    }
    else if(params == "y")
    {
      if(mbCtrl)
      {
        std::cout<<"REDO"<<std::endl;
        
        if(!mspRedoStack.empty())
        {
          std::shared_ptr<EditAction> pAction = mspRedoStack.top();
          pAction->Do();
          
          mspUndoStack.push(pAction);
          mspRedoStack.pop();
        }
      }
    }
    else if(params == "1" || params == "2" || params == "3" || params == "4")
    {
      int nTargetLayer = std::stoi(params);
      PutPointsOnLayer(nTargetLayer, true);
    }
    
    return;
  }
  
  if(command=="KeyRelease")
  {
    if(params == "Ctrl")
    {
      mbCtrl = false;
    }
    
    return;
  }
  
  if(command=="MouseDown")
  {
    std::stringstream ss;
    ss<<params;
    
    int button;
    int state;
    CVD::ImageRef irClicked;
    ss>>button>>state>>irClicked.x>>irClicked.y;
    
    CVD::ImageRef irClickedNorm = NormalizeWindowLoc(irClicked);
    
    std::cout<<"irClicked: "<<irClicked<<std::endl;
    std::cout<<"irClickedNorm: "<<irClickedNorm<<std::endl;
    
    //std::cout<<"button: "<<button<<" state: "<<state<<" clicked: "<<irClicked<<" win size: "<<mpGLWindow->size()<<std::endl;
    if(button == CVD::GLWindow::BUTTON_LEFT)
    {
      if(mSelectionMode == SINGLE)
      {
        ToggleSelection(irClicked);
      }
      else if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
      {
        mSelectionStatus = SELECTING;
        mirSelectionBegin = irClicked;
        mirSelectionCursor = irClicked;
      }
    }
    
    return;
  }
  
  if(command=="MouseUp")
  {
    std::stringstream ss;
    ss<<params;
    
    int button;
    int state;
    CVD::ImageRef irClicked;
    ss>>button>>state>>irClicked.x>>irClicked.y;
    
    //CVD::ImageRef irClickedNorm = NormalizeWindowLoc(irClicked);
    
    //std::cout<<"button: "<<button<<" state: "<<state<<" clicked: "<<irClicked<<" win size: "<<mpGLWindow->size()<<std::endl;
    if(button == CVD::GLWindow::BUTTON_LEFT)
    {
      if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
      {
        SetSelectionInArea(mirSelectionBegin, irClicked, mSelectionMode == BOX_SELECT);
        
        mSelectionMode = SINGLE;
        mSelectionStatus = READY;
      }
    }
    
    return;
  }
  
  if(command=="MouseMove")
  {
    std::stringstream ss;
    ss<<params;
    
    int state;
    CVD::ImageRef irWhere;
    ss>>state>>irWhere.x>>irWhere.y;
    
    if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
    {
      mirSelectionCursor = irWhere;
    }
    
    return;
  }
  
  ROS_FATAL_STREAM("System: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}

void MapEditor::SetSelectionInArea(CVD::ImageRef irBegin, CVD::ImageRef irEnd, bool bSelected)
{
  // Figure out top left and bottom right corners
  CVD::ImageRef irTopLeft, irBottomRight;
  
  if(irBegin.x < irEnd.x && irBegin.y < irEnd.y)  // drew from top left to bottom right
  {
    irTopLeft = irBegin;
    irBottomRight = irEnd;
  }
  else if(irBegin.x > irEnd.x && irBegin.y < irEnd.y)  // drew from top right to bottom left
  {
    irTopLeft.x = irEnd.x;
    irTopLeft.y = irBegin.y;
    
    irBottomRight.x = irBegin.x;
    irBottomRight.y = irEnd.y;
  }
  else if(irBegin.x > irEnd.x && irBegin.y > irEnd.y)  // drew from bottom right to top left
  {
    irTopLeft = irEnd;
    irBottomRight = irBegin;
  }
  else if(irBegin.x < irEnd.x && irBegin.y > irEnd.y)  // drew from bottom left to top right
  {
    irTopLeft.x = irBegin.x;
    irTopLeft.y = irEnd.y;
    
    irBottomRight.x = irEnd.x;
    irBottomRight.y = irBegin.y;
  }
  else  // Some rectangle with at least one side zero width, don't do anything
  {
    return;
  }
  
  TooN::Vector<2> v2RectCorner = CVD::vec(irTopLeft);
  TooN::Vector<2> v2RectExtents = CVD::vec(irBottomRight - irTopLeft);
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    TooN::Vector<2> v2Projected;
    double dDistance;
    double dRadius;
    bool bSuccess = mpMapViewer->ProjectPoint(point.mv3WorldPos, v2Projected, dDistance, dRadius);
    
    if(!bSuccess)
      continue;
      
    if(util::PointInRectangle(v2Projected, v2RectCorner,  v2RectExtents))
    {
      point.mbSelected = bSelected;
    }
  }
}

void MapEditor::ToggleSelection(CVD::ImageRef irPixel)
{
  MapPoint* pClosestPoint = NULL;
  double dClosestDist = 1e10;
  TooN::Vector<2> v2Selected = CVD::vec(irPixel);
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    TooN::Vector<2> v2Projected;
    double dDistance;
    double dRadius;
    bool bSuccess = mpMapViewer->ProjectPoint(point.mv3WorldPos, v2Projected, dDistance, dRadius);
    
    if(!bSuccess)
      continue;
      
    double dDiff = TooN::norm(v2Selected - v2Projected);
    if(dDistance < dClosestDist && dDiff < (dRadius + mdSelectionThresh))
    {
      dClosestDist = dDistance;
      pClosestPoint = &point;
    }
  }
  
  if(!pClosestPoint)
    return;
  
  pClosestPoint->mbSelected = !pClosestPoint->mbSelected;
}

void MapEditor::ToggleAllPoints()
{
  // If any point is currently selected, unselect all
  // If no point is currently selected, select all
  
  bool bAnySelected = false;
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbSelected)
    {
      bAnySelected = true;
      break;
    }
  }
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mbSelected = !bAnySelected;
  }
}

std::vector<MapPoint*> MapEditor::GatherSelected()
{
  std::vector<MapPoint*> vpPoints;
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    if(point.mbSelected)
    {
      vpPoints.push_back(&point);
    }
  }
  
  return vpPoints;
}

void MapEditor::FitGroundPlaneToSelected()
{
  std::shared_ptr<FitGroundPlaneAction> pAction(new FitGroundPlaneAction( mpMap, GatherSelected() ));
  DoEditAction(std::dynamic_pointer_cast<EditAction>(pAction));
}

void MapEditor::DeleteSelected()
{
  std::shared_ptr<DeletePointsAction> pAction(new DeletePointsAction( GatherSelected() ));
  DoEditAction(std::dynamic_pointer_cast<EditAction>(pAction));
}

void MapEditor::DoEditAction(std::shared_ptr<EditAction> pAction)
{
  pAction->Do();
  mspUndoStack.push(pAction);
  
  // When we carry out a new action, anything in the redo stack becomes
  // invalid, so just get rid of it all
  
  while(!mspRedoStack.empty())
  {
    mspRedoStack.pop();
  }
}

CVD::ImageRef MapEditor::NormalizeWindowLoc(CVD::ImageRef irLoc)
{
  double dXScale = mpGLWindow->size().x / (double)(mpGLWindow->GetWindowSize().x);  
  double dYScale = mpGLWindow->size().y / (double)(mpGLWindow->GetWindowSize().y);
  CVD::ImageRef irNormalizedLoc = irLoc;
  irNormalizedLoc.x /= dXScale;
  irNormalizedLoc.y /= dYScale;
  
  return irNormalizedLoc;
}

void MapEditor::PutPointsOnLayer(int nLayer, bool bOnlySelected)
{
  ROS_ASSERT(nLayer >= 1 && nLayer <= 4);
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(bOnlySelected && !point.mbSelected)
      continue;
    
    point.mnUsing = 1 << (nLayer-1);
  }
}
