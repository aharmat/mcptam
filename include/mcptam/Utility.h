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
 * \file Utility.h
 * \brief Declaration of utility functions
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 *
 * Some useful functions that are used in several places and didn't seem like they
 * belonged to any one class
 *
 ****************************************************************************************/

#ifndef __UTILITY_H
#define __UTILITY_H

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <opencv2/core/core.hpp>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <ros/assert.h>

// Don't pollute the global namespace
namespace util{

/** @brief Convert a received image to greyscale
 *  @param cvFrame The image in OpenCV format
 *  @param imBW The image in CVD::Image format */
inline void ConversionBW(const cv::Mat &cvFrame, CVD::Image<CVD::byte> &imBW)
{
  ROS_ASSERT(cvFrame.channels() == 3 || cvFrame.channels() == 1);
  imBW.resize(CVD::ImageRef(cvFrame.cols, cvFrame.rows));
  
  int nTotalsize = imBW.totalsize();
  CVD::byte* pImage = imBW.data();
  const uchar* pFrame = cvFrame.data;
  
  if(cvFrame.channels() == 3)  // source is in color
  {
    for(int i=0; i < nTotalsize; ++i, ++pImage, pFrame+=3)
    {
      *pImage = (pFrame[0] + pFrame[1] + pFrame[2]) / 3;  // do averaging
    }
  }
  else   // otherwise we can do a straight copy
  {
    memcpy(pImage, pFrame, nTotalsize);
  }
}

/** @brief Convert a received image to color RGB
 *  @param cvFrame The image in OpenCV format
 *  @param imRGB The image in CVD::Image format */
inline void ConversionRGB(const cv::Mat &cvFrame, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB)
{
  ROS_ASSERT(cvFrame.channels() == 3 || cvFrame.channels() == 1);
  imRGB.resize(CVD::ImageRef(cvFrame.cols, cvFrame.rows));
  
  int nTotalsize = imRGB.totalsize();
  CVD::Rgb<CVD::byte>* pImage = imRGB.data();
  const uchar* pFrame = cvFrame.data;
  
  if(cvFrame.channels() == 3)  // source is in color
  {
    for(int i=0; i < nTotalsize; ++i, ++pImage, pFrame+=3)
    {
      // Need to fill in the correct member variables for CVD::Rgb
      pImage->red = pFrame[2];
      pImage->green = pFrame[1];
      pImage->blue = pFrame[0]; 
    }
  }
  else
  {
    for(int i=0; i < nTotalsize; ++i, ++pImage, ++pFrame)
    {
      // Need to fill in the correct member variables for CVD::Rgb
      // Result will be a gray image but with 3 channels
      pImage->red = *pFrame;
      pImage->green = *pFrame;
      pImage->blue = *pFrame; 
    }
    
  }
}

/** @brief Convert a geometry_msgs::Pose message into TooN::SE3<>
 *  @param pose The pose to convert
 *  @return The same pose in SE3 format */
inline TooN::SE3<> PoseMsgToSE3(geometry_msgs::Pose pose)
{
  TooN::SE3<> se3Transform;
  
  // Fill in translation directly
  se3Transform.get_translation() = TooN::makeVector(pose.position.x, pose.position.y, pose.position.z);
  
  // The rotation component of a Pose message is in a quaternion, so instead of messing around
  // with quaternion math myself I just use the Bullet classes included in ROS tf package
  
  // Convert orientation message to Bullet quaternion
  tf::Quaternion btQuat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  
  // Convert to Bullet 3x3 matrix
  tf::Matrix3x3 btMat(btQuat);
  
  // Copy data into a TooN 3-Matrix
  TooN::Matrix<3> m3PoseRot;
  for(unsigned j=0; j < 3; j++)
  {
    for(unsigned i=0; i < 3; i++)
    {
      m3PoseRot[i][j] = btMat[i][j];
    }
  }
  
  // Set the rotation and we're done
  se3Transform.get_rotation() = m3PoseRot;
  return se3Transform;
}

/** @brief Convert a TooN::SE3<> message into geometry_msgs::Pose
 *  @param se3Transform The SE3 pose to convert
 *  @return The same pose in geometry_msg format */
inline geometry_msgs::Pose SE3ToPoseMsg(TooN::SE3<> se3Transform)
{
  TooN::Vector<3> v3Pos = se3Transform.get_translation();
  TooN::Matrix<3> m3Rot = se3Transform.get_rotation().get_matrix();
  
  // Similar to PoseMsgToSE3(..), use Bullet 3x3 matrix and quaternion as intermediaries
  
  // Initialize 3x3 matrix directly from TooN 3-matrix
  tf::Matrix3x3 btMat(m3Rot(0,0),m3Rot(0,1),m3Rot(0,2),m3Rot(1,0),m3Rot(1,1),m3Rot(1,2),m3Rot(2,0),m3Rot(2,1),m3Rot(2,2));
  tf::Quaternion btQuat;
  btMat.getRotation(btQuat);
  
  geometry_msgs::Pose pose;

  // Fill in position directly
  pose.position.x = v3Pos[0];
  pose.position.y = v3Pos[1];
  pose.position.z = v3Pos[2];
  
  // Get quaternion values from Bullet quat
  pose.orientation.x = btQuat.x();
  pose.orientation.y = btQuat.y();
  pose.orientation.z = btQuat.z();
  pose.orientation.w = btQuat.w();
  
  return pose;
}

/** @brief Test if a point is within the bounds of a rectangle
 * 
 *  Similar to STL begin/end tests, this function returns true if the point is on the 
 *  left/top edge of the rectangle but false if it's on the right/bottom edge. This way
 *  we can specify the true dimensions of the rectangle in irRectExtents without having to 
 *  subtract one from each 
 *  @param irPoint The point coordinates to test
 *  @param irRectCorner The top left corner of the rectangle
 *  @param irRectExtents The width and height of the rectangle
 *  @return Is the point inside the rectangle?  */
inline bool PointInRectangle(CVD::ImageRef irPoint, CVD::ImageRef irRectCorner, CVD::ImageRef irRectExtents)
{
  CVD::ImageRef irRectFarCorner = irRectCorner + irRectExtents;
  
  if(irPoint.x >= irRectCorner.x && irPoint.x < irRectFarCorner.x &&
     irPoint.y >= irRectCorner.y && irPoint.y < irRectFarCorner.y)
     return true;
  else
    return false;
}

/** @overload */
inline bool PointInRectangle(TooN::Vector<2> v2Point, TooN::Vector<2> v2RectCorner, TooN::Vector<2> v2RectExtents)
{
  TooN::Vector<2> v2RectFarCorner = v2RectCorner + v2RectExtents;
  
  if(v2Point[0] >= v2RectCorner[0] && v2Point[0] < v2RectFarCorner[0] &&
     v2Point[1] >= v2RectCorner[1] && v2Point[1] < v2RectFarCorner[1])
     return true;
  else
    return false;
}

/** @brief Test if a point is within the bounds of a rectangle, assumes (0,0) for top left corner
 * 
 *  Similar to STL begin/end tests, this function returns true if the point is on the 
 *  left/top edge of the rectangle but false if it's on the right/bottom edge. This way
 *  we can specify the true dimensions of the rectangle in irRectExtents without having to 
 *  subtract one from each 
 *  @param v2Point The point coordinates to test
 *  @param v2RectExtents The width and height of the rectangle
 *  @return Is the point inside the rectangle?  */
inline bool PointInRectangle(TooN::Vector<2> v2Point, TooN::Vector<2> v2RectExtents)
{
   if(v2Point[0] >= 0 && v2Point[0] < v2RectExtents[0] &&
     v2Point[1] >= 0 && v2Point[1] < v2RectExtents[1])
     return true;
  else
    return false;
}

/** @brief Converts a CVD::byte image to a sensor_msgs::Image message
 *  @param image The input image
 *  @param [out] image_msg The message to fill */
inline void ImageToMsg(const CVD::SubImage<CVD::byte>& image, sensor_msgs::Image& image_msg)
{
  CVD::ImageRef size = image.size();
  
  image_msg.width = size.x;
  image_msg.height = size.y;
  image_msg.encoding = "mono8";
  
  // Not sure if this works properly, haven't tested it on a bigendian system
#ifdef __BYTE_ORDER
  #if __BYTE_ORDER == __BIG_ENDIAN
    image_msg.is_bigendian = 1;
  #endif
#endif

  image_msg.step = image.row_stride(); // this is becase sizeof(CVD::byte) = 1
  
  int nTotalSize = image.totalsize();
  image_msg.data.resize(nTotalSize);
  
  const CVD::byte* pImage = image.data();
  
  // Copy data directly into image message
  memcpy(&image_msg.data[0], pImage, nTotalSize);
}

/** @brief Converts a sensor_msgs::Image message to a CVD::byte image
 *  @param image_msg The input message
 *  @param [out] image The image to fill */
inline void MsgToImage(const sensor_msgs::Image& image_msg, CVD::Image<CVD::byte>& image)
{
  if(image_msg.width == 0 || image_msg.height == 0)
    return;
  
  image.resize(CVD::ImageRef(image_msg.width, image_msg.height));
  CVD::byte* pImage = image.data();
  int nTotalSize = image.totalsize();
      
  // Copy data directly into image
  memcpy(pImage, &image_msg.data[0], nTotalSize);
}

inline void InvertMask(CVD::SubImage<CVD::byte>& mask)
{
  CVD::ImageRef irCurr;
  do
  {
    if(mask[irCurr])
      mask[irCurr] = 0;
    else
      mask[irCurr] = 255;
  }
  while(irCurr.next(mask.size()));
}

/** @brief Parses a ROS param XmlRpcValue into a vector of string vectors
 * 
 *  This is called by code that needs to load a ROS param indicating camera grouping
 *  @param camGroupList The XmlRpcValue containing camera names, grouped using parentheses. For example,  [[cam1, cam2],[cam3]] indicates
 *                      two camera groups, one with the cameras cam1 and cam2, and one with the camera cam3.
 *  @param [out] camGroupStrings A vector of string vectors, the outer vector indicating grouping
 *  @return Did the parse succeed? */
inline bool Parse(XmlRpc::XmlRpcValue camGroupList, std::vector<std::vector<std::string> >& camGroupStrings)
{
  camGroupStrings.clear();
  
  // input needs to be the right type
  if(camGroupList.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("CameraGroupsParser: Camera group parameter should be an array of camera arrays, indicating grouping with trigger camera in 1st position of each group");
    ROS_ERROR("CameraGroupsParser: e.g. [[cam1, cam2],[cam3]] where the trigger cameras are cam1 and cam3");
    return false;
  }
  
  unsigned numGroups = camGroupList.size();
  
  for(unsigned i=0; i < numGroups; ++i)
  {
    XmlRpc::XmlRpcValue camGroup = camGroupList[i];
   
    // input needs to be the right type 
    if(camGroup.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("CameraGroupsParser: Camera group parameter should be an array of camera arrays, indicating grouping with trigger camera in 1st position of each group");
      ROS_ERROR("CameraGroupsParser: e.g. [[cam1, cam2],[cam3]] where the trigger cameras are cam1 and cam3");
      return false;
    }
    
    std::vector<std::string> cameras;
    for(int j=0; j < camGroup.size(); ++j)
    {
      ROS_INFO_STREAM("CameraGroupsParser: Adding camera "<<std::string(camGroup[j])<<" to group "<<i);
      cameras.push_back(std::string(camGroup[j]));
    }
      
    camGroupStrings.push_back(cameras);
  }
  
  return true;
}
 
} // end namespace util
 
#endif

