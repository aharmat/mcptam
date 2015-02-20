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
 * \file SystemBase.h
 * \brief Declaration of SystemBase interface
 *
 * Parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 * Base class for other SystemXYZ classes, sets up objects used by all child classes like
 * the OpenGL window, the video source, and the map.
 *
 ****************************************************************************************/

#ifndef __SYSTEM_BASE_H
#define __SYSTEM_BASE_H

#include <mcptam/Types.h>
#include <mcptam/GLWindow2.h>
#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>

class Map;
class VideoSourceMulti;

/** @brief Base class for other System(...) classes, sets up commonly used objects and provides
 *         some useful functions.
 * 
 * The initialized objects are the OpenGL window, the video source, and the map,
 * along with some other data structures. Implements a callback function
 * that can be used with a GVars-driven menu system to catch inputs. */
class SystemBase
{
public:

  /** @param windowName The name to be displayed in the OpenGL window's titlebar */
  SystemBase(std::string windowName, bool bFullSize, bool bDrawWindow);
  
  /// Frees dynamically allocated objects
  ~SystemBase();
  
  static bool sbLevelZeroPoints;  // This should really be in SystemFrontendBase, PoseCalibrator inheriting from SystemBase is the problem
  
protected:

  /// Used to save callback data in GUICommandCallBack
  struct Command 
  {
    std::string command; 
    std::string params; 
  };

  /** @brief This can be used with GUI.RegisterCommand to capture user input 
   * 
   *  Simply creates an instance of Command and pushes it to mqCommands
   *  @param ptr Pointer to an instance of the class that will save the captured input in mqCommands
   *  @param command The command that was received
   *  @param params Additional parameters that came with the command */
  static void GUICommandCallBack(void* ptr, std::string command, std::string params);
  
  /** @brief Calculates the average of the durations contained in the argument
   *  @param queue The queue holding the durations
   *  @return The average duration */
  double AverageDuration(std::deque<ros::Duration>& queue);
  
  /** @brief Creates a new GLWindow2 object
   *  @param windowName The name of the window
   *  @param bFullSize If the window size should be calculated based on the video stream's full size or current size
   *  @return Pointer to new window */
  GLWindow2* InitWindow(std::string windowName, bool bFullSize);
  
  /// Creates a new VideoSourceMulti object
  VideoSourceMulti* InitVideoSource();
  
  void SaveCamerasToFolder(std::string folder);
  
  void LoadCamerasFromFolder(std::string folder);
  
  /// Load a map of masks specified by the 'masks' and 'masks_dir' params
  void LoadLiveMasks();
  
  
  ros::NodeHandle mNodeHandle;      ///< ROS global node handle
  ros::NodeHandle mNodeHandlePriv;  ///< ROS private node handle

  VideoSourceMulti* mpVideoSourceMulti;  ///< Pointer to the VideoSourceMulti
  GLWindow2* mpGLWindow;  ///< The GL window
  
  Map *mpMap;   ///< Pointer to the Map
  
  ImageRefMap mmDrawOffsets;        ///< %Map of drawing offset coordinates
  TaylorCameraMap mmCameraModelsLive;   ///< The TaylorCamera models loaded from live cameras
  SE3Map mmPosesLive;                   ///< %Map of fixed relative camera poses loaded from live cameras
  
  TaylorCameraMap mmCameraModelsLoaded;   ///< The TaylorCamera models loaded from saved data
  SE3Map mmPosesLoaded;                   ///< %Map of fixed relative camera poses loaded from saved data
  
  TaylorCameraMap mmCameraModels;   ///< The TaylorCamera models
  SE3Map mmPoses;                   ///< %Map of fixed relative camera poses
  
  ImageBWMap mmMasksLive;
  
  std::string mSaveFolder;
  
  bool mbDone;              ///< Should I quit run loop?
  
  std::queue<Command> mqCommands;   ///< Queued commands received by GUICommandCallBack
  ros::CallbackQueue mCallbackQueueROS;         ///< Custom callback queue so we can spin just for our own callbacks instead of a node-wide spin

};

#endif

