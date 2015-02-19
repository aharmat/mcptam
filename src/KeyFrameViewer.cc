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
}

std::vector<MapPoint*> KeyFrameViewer::GatherVisiblePoints(int nPointVis)
{
  std::vector<MapPoint*> vpPoints;
  
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mnUsing & nPointVis)  // bitfield AND to determine point visibility
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
    if(bSource)
    {
      spKeyFrames.insert(vpPoints[i]->mpPatchSourceKF);
    }
    else
    {
      std::set<KeyFrame*>& spMeasKeyFrames = vpPoints[i]->mMMData.spMeasurementKFs;
      spKeyFrames.insert(spMeasKeyFrames.begin(), spMeasKeyFrames.end());
    }
  }
  
  std::vector<std::pair<int, KeyFrame*> > vScoresAndKeyFrames;
  
  for(std::set<KeyFrame*>::iterator kf_it = spKeyFrames.begin(); kf_it != spKeyFrames.end(); ++kf_it)
  {
    KeyFrame* pKF = *kf_it;
    int nScore = 0;
    
    for(unsigned i=0; i < vpPoints.size(); ++i)
    {
      nScore += pKF->mmpMeasurements.count(vpPoints[i]);
    }
  
    vScoresAndKeyFrames.push_back(std::make_pair(nScore, pKF));
  }
  
  std::sort(vScoresAndKeyFrames.begin(), vScoresAndKeyFrames.end(), std::greater<std::pair<int, KeyFrame*> >());
  
  std::vector<KeyFrame*> vpKeyFrames;
  vpKeyFrames.reserve(vScoresAndKeyFrames.size());
  
  for(unsigned i=0; i < vScoresAndKeyFrames.size(); ++i)
  {
    vpKeyFrames.push_back(vScoresAndKeyFrames[i].second);
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
  
  std::vector<MapPoint*> vpPoints = GatherVisiblePoints(mnPointVis);
  mvpSourceKeyFrames = GatherKeyFrames(vpPoints, true);
  mvpTargetKeyFrames = GatherKeyFrames(vpPoints, false);
  
  //std::cout<<"Visible point measuring keyframes: "<<mvpSourceKeyFrames.size()<<std::endl;
  
  mnSourceIdx = mvpSourceKeyFrames.size() > 0 ? 0 : -1;
  mnTargetIdx = -1;
  mnTargetSearchDir = 1;
  
  //std::cout<<"mnSourceIdx: "<<mnSourceIdx<<std::endl;
  
  // Unselect all points
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mbSelected = false;
  }
  
  mSelectionMode = SINGLE;
  mSelectionStatus = READY;
}

CVD::Image<CVD::byte> KeyFrameViewer::ResizeImageToWindow(CVD::Image<CVD::byte> imOrig, double dWidthFrac)
{
  CVD::ImageRef irWindowSize = mGLWindow.size(); // mGLWindow.GetWindowSize();
  double dNewWidth = irWindowSize.x * dWidthFrac;
  double dResizeRatio = dNewWidth / imOrig.size().x;
  
  mm2Scale = TooN::Identity;
  mm2Scale *= dResizeRatio;
  
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
  GUI.ParseLine("Menu.ShowMenu KeyFrameViewer");
  
  mMessageForUser.str(""); // Wipe the user message clean
  
  if(mnSourceIdx == -1)
  {
    mMessageForUser << "No Source KFs to display! Make sure there are points on visible layer(s)";
    return;
  }
  
  KeyFrame* pKFSource = mvpSourceKeyFrames[mnSourceIdx];
  mimSource= ResizeImageToWindow(pKFSource->maLevels[0].image, 0.5);
  
  glRasterPos(mirSourceOffset);
  glDrawPixels(mimSource);
  
  mMessageForUser << "Viewing Source KF: " << mnSourceIdx + 1 << " / " << mvpSourceKeyFrames.size();
  
  MeasPtrMap mSelectedSourcePoints;
  
  mdPointRadius = (mGLWindow.size().x * mdPointSizeFrac) * 0.5;
  //std::cout<<"Point size: "<< dPointSize<<std::endl;
  
  // First draw all the points
  glPointSize(2*mdPointRadius); 
  glBegin(GL_POINTS);
  
  for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    if(point.mpPatchSourceKF != pKFSource)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
      
    if(point.mbDeleted)
      continue;
    
    TooN::Vector<3> v3Color = gavLevelColors[meas.nLevel];
    TooN::Vector<2> v2Point = mm2Scale*meas.v2RootPos + CVD::vec(mirSourceOffset);
    
    CVD::glColor(v3Color);
    CVD::glVertex(v2Point);
    
    if(point.mbSelected)
    {
      mSelectedSourcePoints[&point] = &meas;
    }
  }
  
  glEnd();
  
  // Now draw a circle around the ones that are selected
  std::vector<CVD::ImageRef> vCirclePoints = CVD::getCircle(mdPointRadius + 2);
        
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(mdPointRadius*0.3);
  
  for(MeasPtrMap::iterator meas_it = mSelectedSourcePoints.begin(); meas_it != mSelectedSourcePoints.end(); ++meas_it)
  {
    //MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    TooN::Vector<3> v3Color = gavLevelColors[meas.nLevel];
    TooN::Vector<2> v2Point = mm2Scale*meas.v2RootPos + CVD::vec(mirSourceOffset);
    
    CVD::glColor(v3Color);
    
    glBegin(GL_LINE_LOOP);
    for(unsigned i=0; i < vCirclePoints.size(); ++i)
    {
      CVD::glVertex(v2Point + CVD::vec(vCirclePoints[i]));
    }
    glEnd();
  }
  
  
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
  
  if(mSelectedSourcePoints.size() > 0)
  {
    // If we didn't have a valid target before, start searching at 0, otherwise start
    // at wherever we left off
    if(mnTargetIdx == -1)
      mnTargetIdx = 0;
      
     bool bFound = false;
      
    // Figure out which target KF to draw
    for(unsigned i=0; i < mvpTargetKeyFrames.size(); ++i, mnTargetIdx += mnTargetSearchDir)
    {
      if(mnTargetIdx >= (int)mvpTargetKeyFrames.size())
        mnTargetIdx = 0;
      if(mnTargetIdx < 0)
        mnTargetIdx = (int)mvpTargetKeyFrames.size() - 1;
        
      KeyFrame* pKFTarget = mvpTargetKeyFrames[mnTargetIdx];
      
      if(pKFTarget == pKFSource)
        continue;
        
      for(MeasPtrMap::iterator meas_it = mSelectedSourcePoints.begin(); meas_it != mSelectedSourcePoints.end(); ++meas_it)
      {
        MapPoint& point = *(meas_it->first);
        
        if(pKFTarget->mmpMeasurements.count(&point))
        {
          bFound = true;
          break;
        }
      }
      
      if(bFound)
        break;
    }
    
    // The target keyframes MUST contain a keyframe that measures a selected point, 
    // since every point has at least two measuring keyframes
    ROS_ASSERT(bFound);
  }
  else
  {
    mnTargetIdx = -1;
  }
  
  // Now actually draw target KF
  if(mnTargetIdx != -1)
  {
    KeyFrame* pKFTarget = mvpTargetKeyFrames[mnTargetIdx];
    CVD::Image<CVD::byte> imTarget = ResizeImageToWindow(pKFTarget->maLevels[0].image, 0.5);
    CVD::ImageRef irTargetOffset = mirSourceOffset + CVD::ImageRef(mimSource.size().x,0);
    
    glRasterPos(irTargetOffset);
    glDrawPixels(imTarget);
    
    mMessageForUser << std::endl << "Viewing Target KF: " << mnTargetIdx + 1 << " / " << mvpTargetKeyFrames.size();
    
    // Now draw the selected points in the target
    glPointSize(2*mdPointRadius); 
    glBegin(GL_POINTS);
    
    for(MeasPtrMap::iterator meas_it = pKFTarget->mmpMeasurements.begin(); meas_it != pKFTarget->mmpMeasurements.end(); ++meas_it)
    {
      MapPoint& point = *(meas_it->first);
      Measurement& meas = *(meas_it->second);
      
      if(!mSelectedSourcePoints.count(&point))
        continue;
      
      TooN::Vector<3> v3Color = gavLevelColors[meas.nLevel];
      TooN::Vector<2> v2Point = mm2Scale*meas.v2RootPos + CVD::vec(irTargetOffset);
      
      CVD::glColor(v3Color);
      CVD::glVertex(v2Point);
    }
    
    glEnd();
    
    // Draw circles around points
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(mdPointRadius*0.3);
    
    for(MeasPtrMap::iterator meas_it = pKFTarget->mmpMeasurements.begin(); meas_it != pKFTarget->mmpMeasurements.end(); ++meas_it)
    {
      MapPoint& point = *(meas_it->first);
      Measurement& meas = *(meas_it->second);
      
      if(!mSelectedSourcePoints.count(&point))
        continue;
      
      TooN::Vector<3> v3Color = gavLevelColors[meas.nLevel];
      TooN::Vector<2> v2Point = mm2Scale*meas.v2RootPos + CVD::vec(irTargetOffset);
      
      CVD::glColor(v3Color);
      
      glBegin(GL_LINE_LOOP);
      for(unsigned i=0; i < vCirclePoints.size(); ++i)
      {
        CVD::glVertex(v2Point + CVD::vec(vCirclePoints[i]));
      }
      glEnd();
    }
  }
  
  
  /*
  // Lock the map while we draw to make sure nobody deletes anything from under our nose
  // This won't affect the Tracker since the KeyFrameViewer is called in the same thread,
  // after the Tracker's done its thing
  boost::mutex::scoped_lock lock(mMap.mMutex); 
  
  if(mMap.mlpMultiKeyFrames.empty())
  {
    mMessageForUser << "MAP IS EMPTY! NO MKFs TO DRAW!";
    return;
  }
  
  // Check if mnSourceIdx is still valid, map could have shrunk with removal of MKFs
  if(mnSourceIdx >= (int)mMap.mlpMultiKeyFrames.size())
    mnSourceIdx = 0;
  
  // Get corresponding iterator dereference
  MultiKeyFrame& mkf = *(*std::next(mMap.mlpMultiKeyFrames.begin(), mnSourceIdx));  
  
  static gvar3<int> gvnDrawLevel("DrawLevel", 0, HIDDEN|SILENT);
  static gvar3<int> gvnDrawCandidates("DrawCandidates", 0, HIDDEN|SILENT);
  
  for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); kf_it++)
  {
    std::string camName = kf_it->first;
    CVD::ImageRef irOffset = mmDrawOffsets[camName];
    CVD::ImageRef irSize = mmSizes[camName];
    
    KeyFrame& kf = *(kf_it->second);
    CVD::Image<CVD::byte> imDraw = kf.maLevels[*gvnDrawLevel].image;
    
    if(imDraw.size() != irSize)  // only do resizing if necessary
    {
      CVD::Image<CVD::byte> imDrawResized(irSize);
      cv::Mat imDrawWrapped(imDraw.size().y, imDraw.size().x, CV_8U, imDraw.data(), imDraw.row_stride());
      cv::Mat imDrawResizedWrapped(imDrawResized.size().y, imDrawResized.size().x, CV_8U, imDrawResized.data(), imDrawResized.row_stride());
      cv::resize(imDrawWrapped, imDrawResizedWrapped, imDrawResizedWrapped.size(), 0, 0, cv::INTER_CUBIC);
      imDraw = imDrawResized;
    }
    
    glRasterPos(irOffset);
    glDrawPixels(imDraw);
    
    CVD::glColor(gavLevelColors[*gvnDrawLevel]);
    glPointSize(3*LevelScale(*gvnDrawLevel)); 
    glBegin(GL_POINTS);
    
    if(!*gvnDrawCandidates)   // Draw measurements from this KF
    {
      for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
      {
        Measurement& meas = *(meas_it->second);
        if(meas.eSource == Measurement::SRC_ROOT && meas.nLevel == *gvnDrawLevel)
        {
          CVD::glVertex(meas.v2RootPos + CVD::vec(irOffset));
        }
      }
    }
    else  // otherwise draw candidates from this KF
    {
      for(unsigned int j=0; j < kf.maLevels[*gvnDrawLevel].vCandidates.size(); ++j) 
      {
        CVD::ImageRef irLevelZero = CVD::ir_rounded(LevelZeroPos(kf.maLevels[*gvnDrawLevel].vCandidates[j].irLevelPos, *gvnDrawLevel));
        CVD::glVertex(irLevelZero + irOffset);
      }
    }
    
    glEnd();
  }

  */
  
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
    if(params == "s")
    {
      mnSourceIdx--;
      if(mnSourceIdx < 0)
        mnSourceIdx = (int)mvpSourceKeyFrames.size()-1;
        
      ToggleAllSourcePoints(true);
    }
    
    if(params == "f")
    {
      mnSourceIdx++;
      if(mnSourceIdx == (int)mvpSourceKeyFrames.size())
        mnSourceIdx = 0;
      
      ToggleAllSourcePoints(true);
    }
    
    if(params == "e")
    {
      mnTargetSearchDir = -1;
      mnTargetIdx--;
      if(mnTargetIdx < 0)
        mnTargetIdx = (int)mvpTargetKeyFrames.size()-1;
    }
    
    if(params == "d")
    {
      mnTargetSearchDir = 1;
      mnTargetIdx++;
      if(mnTargetIdx == (int)mvpTargetKeyFrames.size())
        mnTargetIdx = 0;
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
    else if(params == "a")
    {
      ToggleAllSourcePoints(false);
    }
    else if(params == "Delete")
    {
      std::shared_ptr<DeletePointsAction> pDeleteAction(new DeletePointsAction( GatherSelected() ));
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
  
  KeyFrame* pKFSource = mvpSourceKeyFrames[mnSourceIdx];
  irPixel -= mirSourceOffset;
  
  MapPoint* pClosestPoint = NULL;
  double dSmallestDiff = 1e10;
  TooN::Vector<2> v2Selected = CVD::vec(irPixel);
  
  for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    if(point.mpPatchSourceKF != pKFSource)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
      
    if(point.mbDeleted)
      continue;
      
    TooN::Vector<2> v2Point = mm2Scale*meas.v2RootPos;
    
    double dDiff = TooN::norm(v2Selected - v2Point);
    if(dDiff < (mdPointRadius + mdSelectionThresh)) // && dDiff < dSmallestDiff)
    {
      //dSmallestDiff = dDiff;
      //pClosestPoint = &point;
      point.mbSelected = !point.mbSelected;
    }
  }
  
  /*
  if(!pClosestPoint)
    return;
  
  pClosestPoint->mbSelected = !pClosestPoint->mbSelected;
  */
}

void KeyFrameViewer::ToggleAllSourcePoints(bool bForceUnselect)
{
  KeyFrame* pKFSource = mvpSourceKeyFrames[mnSourceIdx];
  
  // If any point is currently selected, unselect all
  // If no point is currently selected, select all
  
  bool bAnySelected = false;
  
  if(bForceUnselect)
  {
    bAnySelected = bForceUnselect;
  }
  else
  {
    for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
    {
      MapPoint& point = *(meas_it->first);
      //Measurement& meas = *(meas_it->second);
      
      if(point.mpPatchSourceKF != pKFSource)
        continue;
        
      if(!(point.mnUsing & mnPointVis))
        continue;
        
      if(point.mbDeleted)
        continue;
        
      if(point.mbSelected)
      {
        bAnySelected = true;
        break;
      }
    }
  }
  
  for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    //Measurement& meas = *(meas_it->second);
    
    if(point.mpPatchSourceKF != pKFSource)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
      
    if(point.mbDeleted)
      continue;
      
    point.mbSelected = !bAnySelected;
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
  
  KeyFrame* pKFSource = mvpSourceKeyFrames[mnSourceIdx];
  
  for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    if(point.mpPatchSourceKF != pKFSource)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
      
    if(point.mbDeleted)
      continue;
      
    TooN::Vector<2> v2Point = mm2Scale*meas.v2RootPos + CVD::vec(mirSourceOffset);
    
    if(util::PointInRectangle(v2Point, v2RectCorner,  v2RectExtents))
    {
      point.mbSelected = bSelected;
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

std::vector<MapPoint*> KeyFrameViewer::GatherSelected()
{
  std::vector<MapPoint*> vpPoints;

  KeyFrame* pKFSource = mvpSourceKeyFrames[mnSourceIdx];
  
  for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    //Measurement& meas = *(meas_it->second);
    
    if(point.mpPatchSourceKF != pKFSource)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
      
    if(point.mbDeleted)
      continue;
      
    if(point.mbSelected)
    {
      vpPoints.push_back(&point);
    }
  }
  
  return vpPoints;
}
