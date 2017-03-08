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
 * \file Dictionary.h
 * \brief Declaration of the Dictionary class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * Dictionary is the generic dictionary that offers
 * pointer-to-stringId and stringId-to-pointer conversion.
 *
 ****************************************************************************************/

#ifndef MCPTAM_DICTIONARY_H
#define MCPTAM_DICTIONARY_H

#include <map>
#include <string>
#include <ros/ros.h>

/** @brief A generic dictionary class that offers pointer-to-stringId and
 *stringId-to-pointer
 *         conversion.
 *
 * Objects and stringIds are added dynamically to the dictionary.
 * The template class needs to have a default constructor in order to work.
 *Contains some debugging
 * features that allows the tracing of data removals, which is often a source of
 *errors while
 * writing higher level code in the various map makers. */
template <class T>
class Dictionary
{
public:
  /** @param prefix The string that will start every stringId generated */
  explicit Dictionary(std::string prefix);

  /** @brief Convert a pointer to a stringId
   *
   *  If the pointer is not yet in the internal map, a new stringId is returned.
   *If it is
   *  in the internal map, the stringId it got previously is returned.
   *  @param t Pointer to an object
   *  @param bCreate Used for error checking the dictionary lookup, indicates
   *our expectation
   *                 of the object already being in the internal map. See code
   *for details. */
  std::string PtrToId(T *t, bool bCreate);

  /** @brief Convert a stringId to a pointer
   *
   *  If the stringId is not yet in the internal map, a pointer to a new object
   *is returned. The
   *  ownership of this object should transfer to the calling code as Dictionary
   *will make no
   *  attempts at freeing it later. If the stringId is found in the internal
   *map, a pointer to
   *  the object that was created previously is returned.
   *  @param id The stringId
   *  @param bCreate Used for error checking the dictionary lookup, indicates
   *our expectation
   *                 of the stringId already being in the dictionary. See code
   *for details. */
  T *IdToPtr(std::string id, bool bCreate);

  /** @brief Checks if we have an entry for the given pointer
   *  @param t Pointer to an object
   *  @return Whether we have an entry for the object */
  bool HasPtr(T *t);

  /** @brief Checks if we have an entry for the given id
   *  @param id The stringId
   *  @return Whether we have an entry for the id */
  bool HasId(std::string id);

  /** @brief Remove an object from the dictionary, with tracing
   *
   *  When objects are deleted, they need to be removed from Dictionary
   *otherwise all kinds of nasty
   *  pointer-related problems will occur.
   *  @param t Pointer to an object
   *  @param who A name for the code that called this function, used for tracing
   *data removals in case of errors in PtrToId or IdToPtr */
  void Remove(T *t, std::string who);

  /// Resets dictionary to a blank state
  void Clear();

  /// For debugging, prints who removed the object
  std::string WhoRemoved(T *t);

  /// For debugging, prints who removed the id
  std::string WhoRemoved(std::string id);

private:
  std::string mPrefix;                   ///< The string prefix used for each id
  std::map<std::string, T *> mmIdToPtr;  ///< The map of stringIds to pointers
  std::map<T *, std::string> mmPtrToId;  ///< The map of pointers to stringIds

  // For debugging
  std::map<std::string, T *> mmIdToPtr_Perma;  ///< Same as mmIdToPtr, but
  /// entries are never removed, used
  /// for debugging
  std::map<T *, std::string> mmPtrToId_Perma;  ///< Same as mmPtrToId, but
  /// entries are never removed, used
  /// for debugging
  std::map<T *, std::string> mmRemoved;  ///< The map of pointers and who removed
  /// them, used for debugging
};

// Define functions here because it's a template class

template <class T>
Dictionary<T>::Dictionary(std::string prefix)
  : mPrefix(prefix)
{
}

// Print who removed object
template <class T>
std::string Dictionary<T>::WhoRemoved(T *t)
{
  typename std::map<T *, std::string>::iterator rem_it = mmRemoved.find(t);

  if (rem_it != mmRemoved.end())
    return rem_it->second;
  else
    return "Nobody";
}

// Print who removed stringId
template <class T>
std::string Dictionary<T>::WhoRemoved(std::string id)
{
  if (mmIdToPtr_Perma.count(id))
    return WhoRemoved(mmIdToPtr_Perma[id]);
  else
    return "Never had given id";
}

// Convert a pointer to a stringId
template <class T>
std::string Dictionary<T>::PtrToId(T *t, bool bCreate)
{
  typename std::map<T *, std::string>::iterator it = mmPtrToId.find(t);

  if (it == mmPtrToId.end())  // not in map...
  {
    if (!bCreate)  // ... and we don't want to create a new entry? uh oh....
    {
      typename std::map<T *, std::string>::iterator perma_it = mmPtrToId_Perma.find(t);

      ROS_ERROR_STREAM("Dictionary: Problem with pointer: " << reinterpret_cast<uint64_t>(t));
      if (perma_it == mmPtrToId_Perma.end())
      {
        ROS_ERROR("Dictionary: This pointer never had an id assigned!");
      }
      else
      {
        ROS_ERROR_STREAM("Dictionary: This pointer once had an id: " << perma_it->second);

        typename std::map<T *, std::string>::iterator rem_it = mmRemoved.find(t);

        if (rem_it != mmRemoved.end())
          ROS_ERROR_STREAM("Dictionary: But it was removed by: " << rem_it->second);
        else
          ROS_ERROR("Dictionary: But it somehow disappeared! REALLY REALLY "
                    "bad, investigate now!");
      }
    }

    ROS_ASSERT(bCreate);

    std::stringstream ss;
    ros::Time stamp = ros::Time::now();
    // The id consists of the prefix, the pointer value, and the timestamp the
    // conversion was made
    // which pretty much guarantees a unique id
    ss << mPrefix << reinterpret_cast<uint64_t>(t) << 'S' << stamp.sec << 'N' << stamp.nsec;
    std::string id = ss.str();

    // Insert into maps
    mmPtrToId[t] = id;
    mmIdToPtr[id] = t;

    mmPtrToId_Perma[t] = id;
    mmIdToPtr_Perma[id] = t;

    return id;
  }
  else  // in map
  {
    if (bCreate)  // bad, print some diagnostics
    {
      std::stringstream ss;
      ros::Time stamp = ros::Time::now();
      // The id consists of the prefix, the pointer value, and the timestamp the
      // conversion was made
      // which pretty much guarantees a unique id
      ss << mPrefix << reinterpret_cast<uint64_t>(t) << 'S' << stamp.sec << 'N' << stamp.nsec;

      ROS_ERROR_STREAM("Found pointer in map with id: " << it->second << ", but you had the create flag set!");
      ROS_ERROR_STREAM("The new id would have been " << ss.str());
    }

    ROS_ASSERT(!bCreate);  // shouldn't want to create
    return it->second;
  }
}

// Convert a stringId to a pointer
template <class T>
T *Dictionary<T>::IdToPtr(std::string id, bool bCreate)
{
  typename std::map<std::string, T *>::iterator it;

  it = mmIdToPtr.find(id);

  if (it != mmIdToPtr.end())  // exists in map
  {
    // if create is true but we found id already in dictionary, we screwed up
    ROS_ASSERT(!bCreate);

    return it->second;
  }
  else  // not already in map
  {
    if (bCreate)
    {
      T *ptr = new T;  // create a new object, needs default constructor
      mmIdToPtr[id] = ptr;
      mmPtrToId[ptr] = id;

      mmIdToPtr_Perma[id] = ptr;
      mmPtrToId_Perma[ptr] = id;
      return ptr;
    }
    else  // don't create.... in this case, the calling code will handle error
          // checking logic
      return NULL;
  }
}

// Checks if we have an entry for the given pointer
template <class T>
bool Dictionary<T>::HasPtr(T *t)
{
  return mmPtrToId.count(t);
}

// Checks if we have an entry for the given id
template <class T>
bool Dictionary<T>::HasId(std::string id)
{
  return mmIdToPtr.count(id);
}

// Remove an object from the dictionary, with tracing
template <class T>
void Dictionary<T>::Remove(T *t, std::string who)
{
  typename std::map<T *, std::string>::iterator it = mmPtrToId.find(t);
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

#endif  // MCPTAM_DICTIONARY_H
