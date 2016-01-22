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

#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>

Map::Map()
{
  Reset();
}

Map::~Map()
{
  Reset();
}

// Clears all lists and deletes their contents, sets mbGood flag to false
void Map::Reset()
{
  EmptyTrash();

  boost::mutex::scoped_lock lock(mMutex);

  for (MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    while (point.mnUsing != 0)  // This should only be triggered once as points are released fast
    {
      ROS_INFO("Map: Resetting, waiting for point to be given up by tracker...");
      ros::Duration(0.1).sleep();
    }

    delete (*point_it);
  }

  mlpPoints.clear();

  for (MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    while (mkf.mnUsing != 0)  // This should only be triggered once as MKFs are released fast
    {
      ROS_INFO("Map: Resetting, waiting for MultiKeyFrame to be given up by tracker by releasing points...");
      ros::Duration(0.1).sleep();
    }

    delete (*mkf_it);
  }

  mlpMultiKeyFrames.clear();

  mbGood = false;
}

// Points marked bad are moved to the trash
std::set<MapPoint*> Map::MoveBadPointsToTrash()
{
  std::set<MapPoint*> sBadPoints;

  for (MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end();)
  {
    MapPoint& point = *(*point_it);

    if (point.mbBad)
    {
      boost::mutex::scoped_lock lock(mMutex);
      sBadPoints.insert(&point);
      point.EraseAllMeasurements();
      mlpPointsTrash.push_back(&point);
      mlpPoints.erase(point_it++);
    }
    else
      ++point_it;
  }

  return sBadPoints;
}

// MultiKeyFrames marked bad are moved to the trash
std::set<MultiKeyFrame*> Map::MoveBadMultiKeyFramesToTrash()
{
  std::set<MultiKeyFrame*> sBadMKFs;

  for (MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end();)
  {
    MultiKeyFrame& mkf = *(*mkf_it);

    if (mkf.mbBad)
    {
      boost::mutex::scoped_lock lock(mMutex);
      sBadMKFs.insert(&mkf);
      mkf.EraseBackLinksFromPoints();
      mkf.ClearMeasurements();
      mlpMultiKeyFramesTrash.push_back(&mkf);
      mlpMultiKeyFrames.erase(mkf_it++);
    }
    else
      ++mkf_it;
  }

  return sBadMKFs;
}

// Points marked deleted are moved to the trash
void Map::MoveDeletedPointsToTrash()
{
  for (MapPointPtrList::iterator point_it = mlpPoints.begin(); point_it != mlpPoints.end();)
  {
    MapPoint& point = *(*point_it);

    if (point.mbDeleted)
    {
      boost::mutex::scoped_lock lock(mMutex);
      point.mbBad = true;  // set bad flag to true, important
      point.EraseAllMeasurements();
      mlpPointsTrash.push_back(&point);
      mlpPoints.erase(point_it++);
    }
    else
      ++point_it;
  }
}

// Points marked deleted are moved to the trash
void Map::MoveDeletedMultiKeyFramesToTrash()
{
  for (MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFrames.begin(); mkf_it != mlpMultiKeyFrames.end();)
  {
    MultiKeyFrame& mkf = *(*mkf_it);

    if (mkf.mbDeleted)
    {
      boost::mutex::scoped_lock lock(mMutex);
      mkf.mbBad = true;  // set bad flag to true, important
      mkf.EraseBackLinksFromPoints();
      mkf.ClearMeasurements();
      mlpMultiKeyFramesTrash.push_back(&mkf);
      mlpMultiKeyFramesTrash.erase(mkf_it++);
    }
    else
      ++mkf_it;
  }
}

// Deletes entities that are in the trash
void Map::EmptyTrash()
{
  for (MapPointPtrList::iterator point_it = mlpPointsTrash.begin(); point_it != mlpPointsTrash.end();)
  {
    MapPoint& point = *(*point_it);

    // There should be no KeyFrames measuring this point by now, but just make sure in case
    // my logic was off
    if (point.mnUsing == 0 && point.mMMData.spMeasurementKFs.size() == 0)
    {
      delete (&point);
      mlpPointsTrash.erase(point_it++);
    }
    else
      ++point_it;
  }

  for (MultiKeyFramePtrList::iterator mkf_it = mlpMultiKeyFramesTrash.begin(); mkf_it != mlpMultiKeyFramesTrash.end();)
  {
    MultiKeyFrame& mkf = *(*mkf_it);

    // There should be no measurements left in this MultiKeyFrame, but just make sure
    // in case my logic was off
    if (mkf.mnUsing == 0 && mkf.NumMeasurements() == 0)
    {
      delete (&mkf);
      mlpMultiKeyFramesTrash.erase(mkf_it++);
    }
    else
      ++mkf_it;
  }
}
