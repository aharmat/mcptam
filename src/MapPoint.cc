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

//=========================================================================================
//
// Modifications
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/TrackerData.h>

using namespace TooN;

// Delete owned TrackerData pointers
MapPoint::~MapPoint()
{
  // Destructor should only delete resources allocated by the MapPoint
  // Since measurements of the MapPoint are owned by a KeyFrame, we shouldn't
  // automatically assume that upon destruction, the MapPoint should delete measurements
  // of itself from all KeyFrames
    
  // Need to delete the TrackerDatas that have been given to this map point
  for(std::map<std::string, TrackerData*>::iterator it = mmpTData.begin(); it != mmpTData.end(); ++it)
  {
    delete it->second;
  }
}

// Refreshes the cached vectors used for patch finding
void MapPoint::RefreshPixelVectors()
{
  KeyFrame &k = *mpPatchSourceKF;
  
  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Vector<3> v3PlanePoint_C = k.mse3CamFromWorld * mv3WorldPos;
  
  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  double dCamHeight = fabs(v3PlanePoint_C * mv3Normal_NC);

  double dPixelRate = fabs(mv3Center_NC * mv3Normal_NC);
  double dOneRightRate = fabs(mv3OneRightFromCenter_NC * mv3Normal_NC);
  double dOneDownRate = fabs(mv3OneDownFromCenter_NC * mv3Normal_NC);
  
  // Find projections onto plane
  Vector<3> v3CenterOnPlane_C = mv3Center_NC * dCamHeight / dPixelRate;
  Vector<3> v3OneRightOnPlane_C = mv3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  Vector<3> v3OneDownOnPlane_C = mv3OneDownFromCenter_NC * dCamHeight / dOneDownRate;
  
  // Find differences of these projections in the world frame
  mv3PixelRight_W = k.mse3CamFromWorld.get_rotation().inverse() * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  mv3PixelDown_W = k.mse3CamFromWorld.get_rotation().inverse() * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}  

// Calls the EraseMeasurementOfPoint() function of all keyframes that hold measurements of this point 
void MapPoint::EraseAllMeasurements()
{
  for(std::set<KeyFrame*>::iterator it = mMMData.spMeasurementKFs.begin(); it != mMMData.spMeasurementKFs.end(); ++it)
  {
    (*it)->EraseMeasurementOfPoint(this);
  }
  
  mMMData.spMeasurementKFs.clear();
  
}
