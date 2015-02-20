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
//=========================================================================================

#include <mcptam/KeyFrameViewer.h>
#include <mcptam/Map.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/MapPoint.h>
#include <mcptam/GLWindow2.h>
#include <mcptam/OpenGL.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/Utility.h>
#include <mcptam/DeletePointsAction.h>
#include <mcptam/DeleteMeasurementsAction.h>
#include <gvars3/instances.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cvd/draw.h>

using namespace GVars3;

KeyFrameViewer::KeyFrameViewer(Map &map, GLWindow2 &glw)
: mMap(map)
, mGLWindow(glw)
{
  mnSourceIdx = 0;
  mnTargetIdx = -1;
  
  mnVerticalDrawOffset = 100;
  mdPointSizeFrac = 1.0/300.0;
  
  mdSelectionThresh = 1.0;
  mirSourceOffset = CVD::ImageRef(0,mnVerticalDrawOffset);
  
  mpKFSource = NULL;
  mpKFTarget = NULL;
}

std::vector<MapPoint*> KeyFrameViewer::GatherVisiblePoints()
{
  std::vector<MapPoint*> vpPoints;
  
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mnUsing & mnPointVis)  // bitfield AND to determine point visibility
    { 
      vpPoints.push_back(&point);
    }
  }
  
  return vpPoints;
}

std::vector<KeyFrame*> KeyFrameViewer::GatherKeyFrames(std::vector<MapPoint*> vpPoints, bool bSource)
{
  std::set<KeyFrame*> spKeyFrames;
  
  for(unsigned i=0; i < vpPoints.size(); ++i)
  {
    MapPoint* pPoint = vpPoints[i];
    
    if(bSource)
    {
      spKeyFrames.insert(pPoint->mpPatchSourceKF);
    }
    else
    {
      std::set<KeyFrame*> spMeasKeyFrames = pPoint->mMMData.spMeasurementKFs;
      spMeasKeyFrames.erase(pPoint->mpPatchSourceKF);  // Remove the source keyframe from the set
      spKeyFrames.insert(spMeasKeyFrames.begin(), spMeasKeyFrames.end());
    }
  }
  
  std::vector<KeyFrame*> vpKeyFrames;
  
  if(bSource)
  {
    int nNumPoints = (int)vpPoints.size();
    std::vector<std::pair<int, KeyFrame*> > vScoresAndKeyFrames;
    
    for(std::set<KeyFrame*>::iterator kf_it = spKeyFrames.begin(); kf_it != spKeyFrames.end(); ++kf_it)
    {
      KeyFrame* pKF = *kf_it;
      int nScore = 0;
      
      for(int i=0; i < nNumPoints; ++i)
      {
        MapPoint* pPoint = vpPoints[i];
        
        if(pPoint->mpPatchSourceKF != pKF)
          continue;
        
        MeasPtrMap::iterator meas_it = pKF->mmpMeasurements.find(pPoint);
        
        if(meas_it == pKF->mmpMeasurements.end())
          continue;
          
        Measurement* pMeas = meas_it->second;
        
        if(pMeas->bDeleted)
          continue;
        
        // We have a good measurement of this point, add one to KF's score
        nScore += 1;
        
        // If the point is selected, want to push this KF high in the rankings
        // so add nNumPoints to the score, which guarantees that this KF will 
        // rank higher than any that don't have any selected point measurements
        if(pPoint->mbSelected)
          nScore += nNumPoints;
      }
      
      // hijack mdSceneDepthMean for debugging
      pKF->mdSceneDepthMean = nScore;
    
      vScoresAndKeyFrames.push_back(std::make_pair(nScore, pKF));
    }
    
    std::sort(vScoresAndKeyFrames.begin(), vScoresAndKeyFrames.end(), std::greater<std::pair<int, KeyFrame*> >());
    
    
    vpKeyFrames.reserve(vScoresAndKeyFrames.size());
    
    for(unsigned i=0; i < vScoresAndKeyFrames.size(); ++i)
    {
      vpKeyFrames.push_back(vScoresAndKeyFrames[i].second);
    }
  }
  else
  {
    vpKeyFrames.reserve(spKeyFrames.size());
    for(std::set<KeyFrame*>::iterator kf_it = spKeyFrames.begin(); kf_it != spKeyFrames.end(); ++kf_it)
    {
      KeyFrame* pKF = *kf_it;
      vpKeyFrames.push_back(pKF);
    }
  }
  
  return vpKeyFrames;
}

void KeyFrameViewer::Init()
{
  static gvar3<int> gvnLayer1("Layer1", 1, HIDDEN|SILENT);
  static gvar3<int> gvnLayer2("Layer2", 0, HIDDEN|SILENT);
  static gvar3<int> gvnLayer3("Layer3", 0, HIDDEN|SILENT);
  static gvar3<int> gvnLayer4("Layer4", 0, HIDDEN|SILENT);
  
  mnPointVis = (*gvnLayer1 << 0) | (*gvnLayer2 << 1) | (*gvnLayer3 << 2) | (*gvnLayer4 << 3);
  
  std::vector<MapPoint*> vpPoints = GatherVisiblePoints();
  mvpSourceKeyFrames = GatherKeyFrames(vpPoints, true);
  
  //std::cout<<"Visible point measuring keyframes: "<<mvpSourceKeyFrames.size()<<std::endl;
  
  mnSourceIdx = mvpSourceKeyFrames.size() > 0 ? 0 : -1;
  mnTargetIdx = -1;
  
  //std::cout<<"mnSourceIdx: "<<mnSourceIdx<<std::endl;
  
  //UnSelectAllPoints();
  
  mSelectionMode = SINGLE;
  mSelectionStatus = READY;
}

CVD::Image<CVD::byte> KeyFrameViewer::ResizeImageToWindow(CVD::Image<CVD::byte> imOrig, double dWidthFrac, TooN::Matrix<2>& m2Scale)
{
  CVD::ImageRef irWindowSize = mGLWindow.size(); // mGLWindow.GetWindowSize();
  double dNewWidth = irWindowSize.x * dWidthFrac;
  double dResizeRatio = dNewWidth / imOrig.size().x;
  
  m2Scale = TooN::Identity;
  m2Scale *= dResizeRatio;
  
  /*
  std::cout<<"Window width: "<< irWindowSize.x<<std::endl;
  std::cout<<"Width Frac: "<<dWidthFrac<<std::endl;
  std::cout<<"Orig image width: "<<imOrig.size().x<<std::endl;
  std::cout<<"New width: "<<dNewWidth<<std::endl;
  std::cout<<"Resize ratio: "<<dResizeRatio<<std::endl;
  */
  
  CVD::Image<CVD::byte> imResized(CVD::ImageRef(dNewWidth, imOrig.size().y * dResizeRatio));
  
  cv::Mat imOrigWrapped(imOrig.size().y, imOrig.size().x, CV_8U, imOrig.data(), imOrig.row_stride());
  cv::Mat imResizedWrapped(imResized.size().y, imResized.size().x, CV_8U, imResized.data(), imResized.row_stride());
  cv::resize(imOrigWrapped, imResizedWrapped, imResizedWrapped.size(), 0, 0, cv::INTER_CUBIC);
  
  return imResized;
}

// Draw the current MultiKeyFrame
void KeyFrameViewer::Draw()
{
  GUI.ParseLine("Menu.ShowMenu KFViewer");
  
  mMessageForUser.str(""); // Wipe the user message clean
  
  if(mnSourceIdx == -1)
  {
    mMessageForUser << "No Source KFs to display! Make sure there are points on visible layer(s)";
    return;
  }
  
  mpKFSource = mvpSourceKeyFrames[mnSourceIdx];
  mimSource= ResizeImageToWindow(mpKFSource->maLevels[0].image, 0.5, mm2SourceScale);
  
  CVD::ImageRef irTargetOffset = mirSourceOffset + CVD::ImageRef(mimSource.size().x,0);
  CVD::ImageRef irTitleTextOffset = CVD::ImageRef(mimSource.size().x*0.5 - 135,-20);
  
  glRasterPos(mirSourceOffset);
  glDrawPixels(mimSource);
  
  std::stringstream titleStream;
  titleStream << "Source KeyFrame " << mnSourceIdx + 1 << " / " << mvpSourceKeyFrames.size();
  
  glColor3f(0.9,0.9,0);      
  mGLWindow.PrintString(mirSourceOffset + irTitleTextOffset, titleStream.str(), 15);
    
  // For circle drawing
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // First draw all the points
  mdPointRadius = (mGLWindow.size().x * mdPointSizeFrac) * 0.5;
  std::vector<CVD::ImageRef> vCirclePoints = CVD::getCircle(mdPointRadius + 2);
  
  std::vector<MapPoint*> vpSourcePoints = GatherSourcePoints(false);
  
  for(unsigned i=0; i < vpSourcePoints.size(); ++i)
  {
    MapPoint* pPoint = vpSourcePoints[i];
    Measurement* pMeas = mpKFSource->mmpMeasurements[pPoint];

    TooN::Vector<3> v3Color = gavLevelColors[pMeas->nLevel];
    TooN::Vector<2> v2Point = mm2SourceScale*pMeas->v2RootPos + CVD::vec(mirSourceOffset);
    
    CVD::glColor(v3Color);
    
    glPointSize(2*mdPointRadius); 
    glBegin(GL_POINTS);
    CVD::glVertex(v2Point);
    glEnd();
    
    // Now draw a circle around the ones that are selected
    if(pPoint->mbSelected)
    {
      glLineWidth(mdPointRadius*0.3);
      glBegin(GL_LINE_LOOP);
      for(unsigned i=0; i < vCirclePoints.size(); ++i)
      {
        CVD::glVertex(v2Point + CVD::vec(vCirclePoints[i]));
      }
      glEnd();
    }
  }
  
  std::vector<MapPoint*> vpSelectedPoints = GatherSourcePoints(true);
  mvpTargetKeyFrames = GatherKeyFrames(vpSelectedPoints, false);
  
  // If there's some selected points, need to draw a target KF
  if(mvpTargetKeyFrames.size() > 0)
  {
    // If we didn't have a valid target before, start at 0
    if(mnTargetIdx == -1)
    {
      mnTargetIdx = 0;
      mpKFTarget = mvpTargetKeyFrames[mnTargetIdx];
    }
    else
    {
      ROS_ASSERT(mpKFTarget != NULL);
      
      // See if mpKFTarget is still in vector of targets
      std::vector<KeyFrame*>::iterator kf_it = std::find(mvpTargetKeyFrames.begin(), mvpTargetKeyFrames.end(), mpKFTarget);
        
      if(kf_it == mvpTargetKeyFrames.end()) // target kf not in vector anymore, switch to a new target 
      {
        mnTargetIdx = 0;
        mpKFTarget = mvpTargetKeyFrames[mnTargetIdx];
      }
      else
      {
        mnTargetIdx = std::distance(mvpTargetKeyFrames.begin(), kf_it);
      }
    }
  }
  else
  {
    mnTargetIdx = -1;
    mpKFTarget = NULL;
  }
  
  
  titleStream.str("");
  titleStream << "Target KeyFrame " << (mnTargetIdx == -1 ? 0 : mnTargetIdx + 1) << " / " << mvpTargetKeyFrames.size();
  
  glColor3f(0.9,0.9,0);      
  mGLWindow.PrintString(irTargetOffset + irTitleTextOffset, titleStream.str(), 15);
  
  
  // Now actually draw target KF
  if(mnTargetIdx != -1)
  {
    CVD::Image<CVD::byte> imTarget = ResizeImageToWindow(mpKFTarget->maLevels[0].image, 0.5, mm2TargetScale);
    
    glRasterPos(irTargetOffset);
    glDrawPixels(imTarget);
    
    // Now draw the selected points in the target
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    for(unsigned i=0; i < vpSelectedPoints.size(); ++i)
    {
      MapPoint* pPoint = vpSelectedPoints[i];
      
      MeasPtrMap::iterator meas_it = mpKFTarget->mmpMeasurements.find(pPoint);
      
      if(meas_it == mpKFTarget->mmpMeasurements.end())
        continue;
        
      Measurement* pMeas = meas_it->second;;
      
      if(pMeas->bDeleted)
        continue;
      
      TooN::Vector<3> v3Color = gavLevelColors[pMeas->nLevel];
      TooN::Vector<2> v2Point = mm2TargetScale*pMeas->v2RootPos + CVD::vec(irTargetOffset);
      
      CVD::glColor(v3Color);
      
      glPointSize(2*mdPointRadius); 
      glBegin(GL_POINTS);
      CVD::glVertex(v2Point);
      glEnd();
      
      // Draw circles around points
      glLineWidth(mdPointRadius*0.3);
      glBegin(GL_LINE_LOOP);
      for(unsigned i=0; i < vCirclePoints.size(); ++i)
      {
        CVD::glVertex(v2Point + CVD::vec(vCirclePoints[i]));
      }
      glEnd();
    }
  }
  
  mMessageForUser << "Source Points: " << vpSourcePoints.size();
  mMessageForUser << std::endl << "Selection Mode: ";
    
  if(mSelectionMode == SINGLE)
  {
    mMessageForUser << "SINGLE";
  }
  else if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
  {
    TooN::Vector<4> v4Color;
    mMessageForUser << "BOX ";
    
    if(mSelectionMode == BOX_SELECT)
    {
      mMessageForUser << "SELECT";
      v4Color = TooN::makeVector(0.5, 1, 0.5, 0.8);  // greenish
    }
    else
    {
      mMessageForUser << "UN-SELECT";
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
  
  mMessageForUser << std::endl << "Selected Points: " << vpSelectedPoints.size();
  mMessageForUser << std::endl;
  mMessageForUser << std::endl << "NAVIGATION";
  mMessageForUser << std::endl << "A/D: Prev/Next source KeyFrame";
  mMessageForUser << std::endl << "W/S: Prev/Next target KeyFrame";
  mMessageForUser << std::endl;
  mMessageForUser << std::endl << "COMMANDS";
  mMessageForUser << std::endl << "        U: Un-select all points";
  mMessageForUser << std::endl << "        B: Enter box select mode / Switch to box un-select mode";
  mMessageForUser << std::endl << "      ESC: Exit box select mode";
  mMessageForUser << std::endl << "      DEL: Delete selected points (careful about selected points on other source KeyFrames)";
  mMessageForUser << std::endl << "BackSpace: Delete target KeyFrame's measurements of selected points. If a point has <2 measurements, it is also deleted.";
  
}

std::string KeyFrameViewer::GetMessageForUser()
{
  return mMessageForUser.str();
}

bool KeyFrameViewer::GUICommandHandler(std::string command, std::string params, std::shared_ptr<EditAction>& pAction)
{
  bool bHandled = false;
  
  if(mnSourceIdx == -1)
    return bHandled;
  
  if(command=="KeyPress")
  {
    if(params == "a")
    {
      mnSourceIdx--;
      if(mnSourceIdx < 0)
        mnSourceIdx = (int)mvpSourceKeyFrames.size()-1;
        
      //UnSelectAllPoints();
    }
    
    if(params == "d")
    {
      mnSourceIdx++;
      if(mnSourceIdx == (int)mvpSourceKeyFrames.size())
        mnSourceIdx = 0;
      
      //UnSelectAllPoints();
    }
    
    if(params == "w")
    {
      if(mnTargetIdx != -1)
      {
        mnTargetIdx--;
        if(mnTargetIdx < 0)
          mnTargetIdx = (int)mvpTargetKeyFrames.size()-1;
          
        mpKFTarget = mvpTargetKeyFrames[mnTargetIdx];
      }
    }
    
    if(params == "s")
    {
      if(mnTargetIdx != -1)
      {
        mnTargetIdx++;
        if(mnTargetIdx == (int)mvpTargetKeyFrames.size())
          mnTargetIdx = 0;
          
        mpKFTarget = mvpTargetKeyFrames[mnTargetIdx];
      }
    }
    
    if(params == "b")
    {
      if(mSelectionMode == SINGLE || mSelectionMode == BOX_UNSELECT)
      {
        mSelectionMode = BOX_SELECT;
      }
      else if(mSelectionMode == BOX_SELECT)
      {
        mSelectionMode = BOX_UNSELECT;
      }
      
      mSelectionStatus = READY;
      mirSelectionCursor = mGLWindow.cursor_position();
    }
    else if(params == "Escape")
    {
      mSelectionMode = SINGLE;
    }
    else if(params == "u")
    {
      UnSelectAllPoints();
    }
    else if(params == "Delete")
    {
      std::shared_ptr<DeletePointsAction> pDeleteAction(new DeletePointsAction( GatherSourcePoints(true) ));
      pAction = std::dynamic_pointer_cast<EditAction>(pDeleteAction);
    }
    else if(params == "BackSpace")
    {
      std::shared_ptr<DeleteMeasurementsAction> pDeleteAction(new DeleteMeasurementsAction( GatherSelectedTargetMeasurements() ));
      pAction = std::dynamic_pointer_cast<EditAction>(pDeleteAction);
    }
    
    bHandled = true;
  }
  
  if(command=="MouseDown")
  {
    std::stringstream ss;
    ss<<params;
    
    int button;
    int state;
    CVD::ImageRef irClicked;
    ss>>button>>state>>irClicked.x>>irClicked.y;
    
    if(button == CVD::GLWindow::BUTTON_LEFT)
    {
      if(mSelectionMode == SINGLE)
      {
        ToggleSourceSelection(irClicked);
      }
      else if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
      {
        mSelectionStatus = SELECTING;
        mirSelectionBegin = irClicked;
        mirSelectionCursor = irClicked;
      }
    }
    
    bHandled = true;
  }
  
  if(command=="MouseUp")
  {
    std::stringstream ss;
    ss<<params;
    
    int button;
    int state;
    CVD::ImageRef irClicked;
    ss>>button>>state>>irClicked.x>>irClicked.y;
    
    if(button == CVD::GLWindow::BUTTON_LEFT)
    {
      if(mSelectionMode == BOX_SELECT || mSelectionMode == BOX_UNSELECT)
      {
        SetSourceSelectionInArea(mirSelectionBegin, irClicked, mSelectionMode == BOX_SELECT);
        
        mSelectionMode = SINGLE;
        mSelectionStatus = READY;
      }
    }
    
    bHandled = true;
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
    
    bHandled = true;
  }
  
  return bHandled;
}

void KeyFrameViewer::ToggleSourceSelection(CVD::ImageRef irPixel)
{
  if(!util::PointInRectangle(irPixel, mirSourceOffset, mimSource.size()))
    return;
  
  irPixel -= mirSourceOffset;
  
  //MapPoint* pClosestPoint = NULL;
  //double dSmallestDiff = 1e10;
  TooN::Vector<2> v2Selected = CVD::vec(irPixel);
  
  std::vector<MapPoint*> vpSourcePoints = GatherSourcePoints(false);
  
  for(unsigned i=0; i < vpSourcePoints.size(); ++i)
  {
    MapPoint* pPoint = vpSourcePoints[i];
    Measurement* pMeas = mpKFSource->mmpMeasurements[pPoint];
      
    TooN::Vector<2> v2Point = mm2SourceScale*pMeas->v2RootPos;
    
    double dDiff = TooN::norm(v2Selected - v2Point);
    if(dDiff < (mdPointRadius + mdSelectionThresh)) // && dDiff < dSmallestDiff)
    {
      //dSmallestDiff = dDiff;
      //pClosestPoint = &point;
      pPoint->mbSelected = !pPoint->mbSelected;
    }
  }
  
  /*
  if(!pClosestPoint)
    return;
  
  pClosestPoint->mbSelected = !pClosestPoint->mbSelected;
  */
}

void KeyFrameViewer::UnSelectAllPoints()
{
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mbSelected = false;
  }
}

CVD::ImageRef KeyFrameViewer::ClampLocToSource(CVD::ImageRef irLoc)
{
  CVD::ImageRef irLocShifted = irLoc - mirSourceOffset;
  
  if(irLocShifted.x < 0)
    irLocShifted.x = 0;
    
  if(irLocShifted.y < 0)
    irLocShifted.y = 0;
    
  if(irLocShifted.x >= mimSource.size().x)
    irLocShifted.x = mimSource.size().x-1;
    
  if(irLocShifted.y >= mimSource.size().y)
    irLocShifted.y = mimSource.size().y-1;
  
  return irLocShifted + mirSourceOffset;
}

void KeyFrameViewer::SetSourceSelectionInArea(CVD::ImageRef irBegin, CVD::ImageRef irEnd, bool bSelected)
{
  irBegin = ClampLocToSource(irBegin);
  irEnd = ClampLocToSource(irEnd);
  
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
  
  std::vector<MapPoint*> vpSourcePoints = GatherSourcePoints(false);
  
  for(unsigned i=0; i < vpSourcePoints.size(); ++i)
  {
    MapPoint* pPoint = vpSourcePoints[i];
    Measurement* pMeas = mpKFSource->mmpMeasurements[pPoint];
    
    TooN::Vector<2> v2Point = mm2SourceScale*pMeas->v2RootPos + CVD::vec(mirSourceOffset);
    
    if(util::PointInRectangle(v2Point, v2RectCorner,  v2RectExtents))
    {
      pPoint->mbSelected = bSelected;
    }
  }
}

void KeyFrameViewer::DrawCrosshairs(CVD::ImageRef irPos, TooN::Vector<4> v4Color, float fLineWidth)
{
  irPos = ClampLocToSource(irPos);
  
  int nWidth = mimSource.size().x;
  int nHeight = mimSource.size().y;
  
  InitOrthoDrawing();
  
  glLineWidth(fLineWidth);
  CVD::glColor(v4Color);
  CVD::glLine(CVD::ImageRef(mirSourceOffset.x, irPos.y), CVD::ImageRef(mirSourceOffset.x+nWidth, irPos.y));
  CVD::glLine(CVD::ImageRef(irPos.x, mirSourceOffset.y), CVD::ImageRef(irPos.x, mirSourceOffset.y+nHeight));
}

void KeyFrameViewer::DrawRectangle(CVD::ImageRef irBegin, CVD::ImageRef irEnd, TooN::Vector<4> v4Color, float fLineWidth)
{
  irBegin = ClampLocToSource(irBegin);
  irEnd = ClampLocToSource(irEnd);
  
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

void KeyFrameViewer::InitOrthoDrawing()
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
  mGLWindow.SetupWindowOrtho();
}

MeasPtrMap KeyFrameViewer::GatherSelectedTargetMeasurements()
{
  MeasPtrMap mpMeas;
  
  for(MeasPtrMap::iterator meas_it = mpKFTarget->mmpMeasurements.begin(); meas_it != mpKFTarget->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    if(meas.bDeleted)
      continue;
      
    if(!point.mbSelected)
      continue;
  
    mpMeas[&point] = &meas;
  }
  
  return mpMeas;
}

std::vector<MapPoint*> KeyFrameViewer::GatherSourcePoints(bool bOnlySelected)
{
  std::vector<MapPoint*> vpPoints;
  
  for(MeasPtrMap::iterator meas_it = mpKFSource->mmpMeasurements.begin(); meas_it != mpKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    //Measurement& meas = *(meas_it->second);
    
    if(point.mbDeleted)
      continue;
      
    if(point.mbBad)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
    
    if(point.mpPatchSourceKF != mpKFSource)
      continue;
      
    if(!bOnlySelected || point.mbSelected)
    {
      vpPoints.push_back(&point);
    }
  }
  
  return vpPoints;
}
