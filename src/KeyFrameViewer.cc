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
#include <gvars3/instances.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace GVars3;

KeyFrameViewer::KeyFrameViewer(Map &map, GLWindow2 &glw)
: mMap(map)
, mGLWindow(glw)
{
  mnSourceIdx = 0;
  mnVerticalDrawOffset = 100;
  mdPointSizeFrac = 1.0/300.0;
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
  
  //std::cout<<"mnSourceIdx: "<<mnSourceIdx<<std::endl;
  
  // Unselect all points
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mbSelected = false;
  }
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
    return;
  
  KeyFrame* pKFSource = mvpSourceKeyFrames[mnSourceIdx];
  CVD::Image<CVD::byte> imSource= ResizeImageToWindow(pKFSource->maLevels[0].image, 0.5);
  CVD::ImageRef irSourceOffset = CVD::ImageRef(0,mnVerticalDrawOffset);
  
  glRasterPos(irSourceOffset);
  glDrawPixels(imSource);
  
  mMessageForUser << "Viewing Source KF: " << mnSourceIdx + 1 << " / " << mvpSourceKeyFrames.size();
  
  double dPointSize = mGLWindow.size().x * mdPointSizeFrac;
  std::cout<<"Point size: "<< dPointSize<<std::endl;
  
  glPointSize(dPointSize); 
  glBegin(GL_POINTS);
  
  for(MeasPtrMap::iterator meas_it = pKFSource->mmpMeasurements.begin(); meas_it != pKFSource->mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    if(point.mpPatchSourceKF != pKFSource)
      continue;
      
    if(!(point.mnUsing & mnPointVis))
      continue;
    
    CVD::glColor(gavLevelColors[meas.nLevel]);
    CVD::glVertex(mm2Scale*meas.v2RootPos + CVD::vec(irSourceOffset));
  }
  
  glEnd();
  
  
  
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
  
  if(command=="KeyPress")
  {
    if(params == "a")
    {
      // Make sure index is valid before changing it
      if(mnSourceIdx != -1)
      {
        mnSourceIdx--;
        if(mnSourceIdx < 0)
          mnSourceIdx = (int)mvpSourceKeyFrames.size()-1;
      }
    }
    
    if(params == "d")
    {
      // Make sure index is valid before changing it
      if(mnSourceIdx != -1)
      {
        mnSourceIdx++;
        if(mnSourceIdx == (int)mvpSourceKeyFrames.size())
          mnSourceIdx = 0;
      }
    }
    
    bHandled = true;
  }
  
  return bHandled;
}
