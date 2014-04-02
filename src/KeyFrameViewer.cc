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
#include <mcptam/GLWindow2.h>
#include <mcptam/OpenGL.h>
#include <mcptam/LevelHelpers.h>
#include <gvars3/instances.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace GVars3;

KeyFrameViewer::KeyFrameViewer(Map &map, GLWindow2 &glw, ImageRefMap mDrawOffsets, ImageRefMap mSizes)
: mMap(map)
, mGLWindow(glw)
, mmDrawOffsets(mDrawOffsets)
, mmSizes(mSizes)
{
  nCurrentIdx = 0;
}

// Draw the current MultiKeyFrame
void KeyFrameViewer::Draw()
{
  mMessageForUser.str(""); // Wipe the user message clean
  
  // Lock the map while we draw to make sure nobody deletes anything from under our nose
  // This won't affect the Tracker since the KeyFrameViewer is called in the same thread,
  // after the Tracker's done its thing
  boost::mutex::scoped_lock lock(mMap.mMutex); 
  
  if(mMap.mlpMultiKeyFrames.empty())
  {
    mMessageForUser << "MAP IS EMPTY! NO MKFs TO DRAW!";
    return;
  }
  
  // Check if nCurrentIdx is still valid, map could have shrunk with removal of MKFs
  if(nCurrentIdx >= (int)mMap.mlpMultiKeyFrames.size())
    nCurrentIdx = 0;
  
  // Get corresponding iterator dereference
  MultiKeyFrame& mkf = *(*std::next(mMap.mlpMultiKeyFrames.begin(), nCurrentIdx));  
  
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
  
  mMessageForUser << "MKF : "<<nCurrentIdx+1<<" / "<<mMap.mlpMultiKeyFrames.size();
  
}

// Select the next MultiKeyFrame in the Map
void KeyFrameViewer::Next()
{
  nCurrentIdx++;
  boost::mutex::scoped_lock lock(mMap.mMutex); 
  
  if(nCurrentIdx >= (int)mMap.mlpMultiKeyFrames.size())
  {
    nCurrentIdx = 0;
  }
}

// Select the previous MultiKeyFrame in the Map
void KeyFrameViewer::Prev()
{
  nCurrentIdx--;
  
  if(nCurrentIdx < 0)
  {
    boost::mutex::scoped_lock lock(mMap.mMutex); 
    nCurrentIdx = mMap.mlpMultiKeyFrames.size() - 1;
  }
}

std::string KeyFrameViewer::GetMessageForUser()
{
  return mMessageForUser.str();
}
