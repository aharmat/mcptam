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
  ROS_ASSERT(mbBad);
  
  for(std::set<KeyFrame*>::iterator it = mMMData.spMeasurementKFs.begin(); it != mMMData.spMeasurementKFs.end(); ++it)
  {
    // delete measurements immediately since this function only gets called when a point is being trashed,
    // in which case, both server and client will delete the point and there's no reason to keep a list of
    // deleted measurements to the point
    (*it)->EraseMeasurementOfPoint(this);  
  }
  
  mMMData.spMeasurementKFs.clear();
  
}

void MapPoint::EraseAllCrossCov()
{
  std::set<PointCrossCov*> spCrossCov;
  //boost::mutex::scoped_lock lock(mCrossCovMutex);
  
  // Need to gather PointCrossCov objects in a different container, because calling their 
  // EraseLinks function will remove itself from both points. 
  for(CrossCovPtrMap::iterator x_it = mmpCrossCov.begin(); x_it != mmpCrossCov.end(); ++x_it)
  {
    spCrossCov.insert(x_it->second);
  }
  
  //lock.unlock();
  
  for(std::set<PointCrossCov*>::iterator x_it = spCrossCov.begin(); x_it != spCrossCov.end(); ++x_it)
  {
    (*x_it)->EraseLinks();
    delete *x_it;
  }
  
  ROS_ASSERT(mmpCrossCov.empty());
}

bool MapPoint::CrossCov(MapPoint* pOther, TooN::Matrix<3>& m3CrossCov)
{
  //boost::mutex::scoped_lock lock(mCrossCovMutex);
  
  CrossCovPtrMap::iterator it = mmpCrossCov.find(pOther);
  
  if(it == mmpCrossCov.end())
    return false;
  
  m3CrossCov = it->second->GetCrossCov(this);
  return true;
}

// Updating happens from the mapper thread, so we need to lock the mutex
// in case the tracker thread is adding new entries to the crosscov map
void MapPoint::UpdateCrossCovPriorities(double dVal)
{
  ROS_ASSERT(dVal >= 0);
  boost::mutex::scoped_lock lock(mCrossCovMutex);
  
  for(CrossCovPtrMap::iterator x_it = mmpCrossCov.begin(); x_it != mmpCrossCov.end(); ++x_it)
  {
    PointCrossCov* pCrossCov = x_it->second;
    pCrossCov->mdPriority += dVal;
  }
}

// Adding new cross cov happens in the tracker thread, lock mutex so that
// there are no issues with mapper thread updating the priority
void MapPoint::AddCrossCov(MapPoint* pOther, PointCrossCov* pCrossCov)
{
  boost::mutex::scoped_lock lock(mCrossCovMutex);
  mmpCrossCov[pOther] = pCrossCov;
}

// -----------------------------------------------------------------------------------------------------------

PointCrossCov::PointCrossCov(MapPoint& point1, MapPoint& point2)
: mPoint1(point1)
, mPoint2(point2)
{
  mm3CrossCov = TooN::Zeros;
  mdPriority = 1e10;
}

// Row: point1, Col: point2
void PointCrossCov::SetCrossCov(const TooN::Matrix<3>& m3CrossCov)
{
  mm3CrossCov = m3CrossCov;
  mdPriority = 0;
}

/// point: the querying point
TooN::Matrix<3> PointCrossCov::GetCrossCov(MapPoint* pPoint)
{
  if(pPoint == &mPoint1)
    return mm3CrossCov;
  else if(pPoint == &mPoint2)
    return mm3CrossCov.T();
  
  ROS_BREAK();
  return TooN::Zeros;
}

void PointCrossCov::EraseLinks()
{
  //boost::mutex::scoped_lock lock1(mPoint1.mCrossCovMutex);
  CrossCovPtrMap::iterator it1 = mPoint1.mmpCrossCov.find(&mPoint2);
  ROS_ASSERT(it1 != mPoint1.mmpCrossCov.end());
  mPoint1.mmpCrossCov.erase(it1);
  //lock1.unlock();
  
  //boost::mutex::scoped_lock lock2(mPoint2.mCrossCovMutex);
  CrossCovPtrMap::iterator it2 = mPoint2.mmpCrossCov.find(&mPoint1);
  ROS_ASSERT(it2 != mPoint2.mmpCrossCov.end());
  mPoint2.mmpCrossCov.erase(it2);
}
