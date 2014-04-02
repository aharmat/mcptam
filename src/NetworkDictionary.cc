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

#include <mcptam/NetworkDictionary.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/MapPoint.h>

#include <ros/ros.h>

template <class T>
Dictionary<T>::Dictionary(std::string prefix)
: mPrefix(prefix)
{
}

// Convert a pointer to a stringId
template <class T>
std::string Dictionary<T>::PtrToId(T* t, bool bCreate)
{
  typename std::map<T*, std::string>::iterator it = mmPtrToId.find(t);
  
  if(it == mmPtrToId.end())  // not in map...
  {
    if(!bCreate)  // ... and we don't want to create a new entry? uh oh....
    {
      std::string id_perma = mmPtrToId_Perma[t];
      
      ROS_ERROR_STREAM("Dictionary: Problem with pointer: "<<reinterpret_cast<uint64_t>(t));
      if(id_perma.empty())
      {
        ROS_ERROR("Dictionary: This pointer never had an id assigned!");
      }
      else
      {
        ROS_ERROR_STREAM("Dictionary: This pointer once had an id: "<<id_perma);
        
        typename std::map<T*, std::string>::iterator jit = mmRemoved.find(t);
        
        if(jit != mmRemoved.end())
          ROS_ERROR_STREAM("Dictionary: But it was removed by: "<<jit->second);
        else
          ROS_ERROR("Dictionary: But it somehow disappeared! REALLY REALLY bad, investigate now!");
        
      }
      
    }
    
    ROS_ASSERT(bCreate);
    
    std::stringstream ss;
    ros::Time stamp = ros::Time::now();
    // The id consists of the prefix, the pointer value, and the timestamp the conversion was made
    // which pretty much guarantees a unique id
    ss << mPrefix << reinterpret_cast<uint64_t>(t) << 'S' <<stamp.sec << 'N' << stamp.nsec;
    std::string id = ss.str();
    
    // Insert into maps
    mmPtrToId[t] = id;
    mmIdToPtr[id] = t;
    
    mmPtrToId_Perma[t] = id;
    mmIdToPtr_Perma[id] = t;
    
    return id;
  }
  else // in map
  {
    ROS_ASSERT(!bCreate);  // shouldn't want to create
    return it->second;
  }
    
}
    
// Convert a stringId to a pointer
template <class T>
T* Dictionary<T>::IdToPtr(std::string id, bool bCreate)
{
  typename std::map<std::string, T*>::iterator it;
  
  it = mmIdToPtr.find(id);
  
  if(it != mmIdToPtr.end()) // exists in map
  {
    // if create is true but we found id already in dictionary, we screwed up
    ROS_ASSERT(!bCreate); 
    
    return it->second;
  }
  else // not already in map
  {
    if(bCreate) 
    {
      T* ptr = new T;  // create a new object, needs default constructor
      mmIdToPtr[id] = ptr;
      mmPtrToId[ptr] = id;
      
      mmIdToPtr_Perma[id] = ptr;
      mmPtrToId_Perma[ptr] = id;
      return ptr;
    }
    else  // don't create.... in this case, the calling code will handle error checking logic
      return NULL;
  }
  
}

// Remove an object from the dictionary, with tracing
template <class T>
void Dictionary<T>::Remove(T* t, std::string who)
{
  typename std::map<T*, std::string>::iterator it = mmPtrToId.find(t);
  ROS_ASSERT(it != mmPtrToId.end());
  
  std::string id = it->second;
  
  mmPtrToId.erase(it);
  
  ROS_ASSERT(mmIdToPtr.count(id));
  mmIdToPtr.erase(id);
  
  mmRemoved[t] = who;  // record who removed this pointer
}

// Resets dictionary to a blank state
template <class T>
void Dictionary<T>::Clear()
{
  mmIdToPtr.clear();
  mmPtrToId.clear();
  
  mmIdToPtr_Perma.clear();
  mmPtrToId_Perma.clear();
  
  mmRemoved.clear();
}

// Sets the prefixes of the two dictionaries
NetworkDictionary::NetworkDictionary()
: mMapPointDict("MP")
, mMultiKeyFrameDict("MKF")
{
}

std::string NetworkDictionary::PtrToId(MultiKeyFrame* pMKF, bool bCreate)
{
  return mMultiKeyFrameDict.PtrToId(pMKF, bCreate);
}

std::string NetworkDictionary::PtrToId(MapPoint* pPoint, bool bCreate)
{
  return mMapPointDict.PtrToId(pPoint, bCreate);
}

MultiKeyFrame* NetworkDictionary::IdToMultiKeyFramePtr(std::string id, bool bCreate)
{
  return mMultiKeyFrameDict.IdToPtr(id, bCreate);
}

MapPoint* NetworkDictionary::IdToMapPointPtr(std::string id, bool bCreate)
{
  return mMapPointDict.IdToPtr(id, bCreate);
}

void NetworkDictionary::Remove(MultiKeyFrame* pMKF, std::string who)
{
  mMultiKeyFrameDict.Remove(pMKF, who);
}
    
void NetworkDictionary::Remove(MapPoint* mM, std::string who)
{
  mMapPointDict.Remove(mM, who);
}

void NetworkDictionary::Clear()
{
  mMultiKeyFrameDict.Clear();
  mMapPointDict.Clear();
}
