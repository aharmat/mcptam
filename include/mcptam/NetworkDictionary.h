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


/****************************************************************************************
 *
 * \file NetworkDictionary.h
 * \brief Declaration of (Network)Dictionary classes
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * NetworkDictionary is a class that helps NetworkManager communicate data between
 * mcptam's client and server. Dictionary is the generic dictionary that offers 
 * pointer-to-stringId and stringId-to-pointer conversion.
 *
 ****************************************************************************************/

#ifndef __NETWORK_DICTIONARY_H
#define __NETWORK_DICTIONARY_H

#include <map>
#include <string>

class MultiKeyFrame;
class MapPoint;

/** @brief A generic dictionary class that offers pointer-to-stringId and stringId-to-pointer
 *         conversion.
 * 
 * Objects and stringIds are added dynamically to the dictionary. 
 * The template class needs to have a default constructor in order to work. Contains some debugging
 * features that allows the tracing of data removals, which is often a source of errors while
 * writing higher level code in the various map makers. */
template <class T>
class Dictionary
{
  public:
    
    /** @param prefix The string that will start every stringId generated */
    Dictionary(std::string prefix);
    
    /** @brief Convert a pointer to a stringId
     * 
     *  If the pointer is not yet in the internal map, a new stringId is returned. If it is
     *  in the internal map, the stringId it got previously is returned.
     *  @param t Pointer to an object
     *  @param bCreate Used for error checking the dictionary lookup, indicates our expectation 
     *                 of the object already being in the internal map. See code for details. */
    std::string PtrToId(T* t, bool bCreate);
    
    /** @brief Convert a stringId to a pointer
     * 
     *  If the stringId is not yet in the internal map, a pointer to a new object is returned. The 
     *  ownership of this object should transfer to the calling code as Dictionary will make no
     *  attempts at freeing it later. If the stringId is found in the internal map, a pointer to
     *  the object that was created previously is returned.
     *  @param id The stringId
     *  @param bCreate Used for error checking the dictionary lookup, indicates our expectation 
     *                 of the stringId already being in the dictionary. See code for details. */
    T* IdToPtr(std::string id, bool bCreate);
    
    /** @brief Remove an object from the dictionary, with tracing
     * 
     *  When objects are deleted, they need to be removed from Dictionary otherwise all kinds of nasty
     *  pointer-related problems will occur.
     *  @param t Pointer to an object
     *  @param who A name for the code that called this function, used for tracing data removals in case of errors in PtrToId or IdToPtr */
    void Remove(T* t, std::string who);
    
    /// Resets dictionary to a blank state
    void Clear();
  
  private:
    std::string mPrefix;                     ///< The string prefix used for each id
    std::map<std::string, T*> mmIdToPtr;     ///< The map of stringIds to pointers
    std::map<T*, std::string> mmPtrToId;     ///< The map of pointers to stringIds
    
    // For debugging
    std::map<std::string, T*> mmIdToPtr_Perma;   ///< Same as mmIdToPtr, but entries are never removed, used for debugging
    std::map<T*, std::string> mmPtrToId_Perma;   ///< Same as mmPtrToId, but entries are never removed, used for debugging
    std::map<T*, std::string> mmRemoved;         ///< The map of pointers and who removed them, used for debugging
  
};

/** @brief A class implementing dictionaries for MapPoint and MultiKeyFrame objects
 * 
 *  Contains two Dictionary objects, one for each of MapPoint and MultiKeyFrame objects, 
 *  and functions that mirror those in Dictionary but for the specified types. It's debatable
 *  that this class is even necessary, as the calling code (NetworkManager) could just
 *  create two Dictionary objects itself, but here it is. */
class NetworkDictionary
{
  public:
  
    /// Sets the prefixes of the two dictionaries
    NetworkDictionary();
    
    /// Same as PtrToId in Dictionary but for a MultiKeyFrame
    std::string PtrToId(MultiKeyFrame* pMKF, bool bCreate);
    
    /// Same as PtrToId in Dictionary but for a MapPoint
    std::string PtrToId(MapPoint* pPoint, bool bCreate);
    
    /// Same as IdToPtr in Dictionary but for a MultiKeyFrame
    MultiKeyFrame* IdToMultiKeyFramePtr(std::string id, bool bCreate);
    
    /// Same as IdToPtr in Dictionary but for a MapPoint
    MapPoint* IdToMapPointPtr(std::string id, bool bCreate);
    
    /// Same as Remove in Dictionary but for a MultiKeyFrame
    void Remove(MultiKeyFrame* pMKF, std::string who);
    
    /// Same as Remove in Dictionary but for a MapPoint
    void Remove(MapPoint* pPoint, std::string who);
    
    /// Clears both dictionaries
    void Clear();
    
  private:
  
    Dictionary<MapPoint> mMapPointDict;            ///< The MapPoint Dictionary
    Dictionary<MultiKeyFrame> mMultiKeyFrameDict;  ///< The MultiKeyFrame Dictionary
  
};

#endif

