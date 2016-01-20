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
 * \file MainClient.cc
 * \brief Main entry point for mcptam_client
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Parts of this code are from the original PTAM, which are
 * Copyright 2008 Isis Innovation Limited
 *
 ****************************************************************************************/

#include <iostream>
#include <mcptam/SystemClient.h>
#include <mcptam/LoadStaticParamsGeneral.h>
#include <mcptam/LoadStaticParamsClient.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mcptam_client");
  ros::NodeHandle nh;

  ROS_INFO("  Welcome to mcptam_client ");
  ROS_INFO("  ------------------------ ");
  ROS_INFO("  Multiple Camera Parallel Tracking and Mapping (MCPTAM");
  ROS_INFO("  Copyright 2014 Adam Harmat, McGill University");
  ROS_INFO("                 Michael Tribou, University of Waterloo");
  ROS_INFO("  ");
  ROS_INFO("  Multi-Camera Parallel Tracking and Mapping (MCPTAM) is free software:");
  ROS_INFO("  you can redistribute it and/or modify it under the terms of the GNU ");
  ROS_INFO("  General Public License as published by the Free Software Foundation,");
  ROS_INFO("  either version 3 of the License, or (at your option) any later");
  ROS_INFO("  version.");
  ROS_INFO("  ");
  ROS_INFO("  This program is distributed in the hope that it will be useful,");
  ROS_INFO("  but WITHOUT ANY WARRANTY; without even the implied warranty of");
  ROS_INFO("  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the");
  ROS_INFO("  GNU General Public License for more details.");
  ROS_INFO("  ");
  ROS_INFO("  You should have received a copy of the GNU General Public License");
  ROS_INFO("  along with this program.  If not, see <http://www.gnu.org/licenses/>.");
  ROS_INFO("  ");
  ROS_INFO("  Based on Parallel Tracking and Mapping (PTAM) software");
  ROS_INFO("  Copyright 2008 Isis Innovation Limited");
  ROS_INFO("  ");

  LoadStaticParamsGeneral();
  LoadStaticParamsClient();

  try
  {
    SystemClient sys;
    sys.Run();
  }
  catch (CVD::Exceptions::All e)
  {
    ROS_ERROR("Failed to run mcptam_client; got exception. ");
    ROS_ERROR("   Exception was: ");
    ROS_ERROR_STREAM(e.what);
  }
}
