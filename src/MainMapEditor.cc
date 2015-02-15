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
 * \file Main.cc
 * \brief Main entry point for mcptam
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the original PTAM, which are
 * Copyright 2008 Isis Innovation Limited
 *
 ****************************************************************************************/
 
 
#include <mcptam/MapEditor.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_editor");
  ros::NodeHandle nh;
  
  /*
  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  // Set the logger for this package to output all statements
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  */
  
  try
  {
    MapEditor ed;
    ed.Run();
  }
  catch(CVD::Exceptions::All e)
  {
    ROS_ERROR("Failed to run map_editor; got exception. ");
    ROS_ERROR("   Exception was: ");
    ROS_ERROR_STREAM(e.what);
  }
}










