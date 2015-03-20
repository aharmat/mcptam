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
 * \file RelocaliserFabMap.h
 * \brief Declares the RelocaliserFabMap class
 *
 * This code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Each KF stores a small, blurred version of itself;
 * Just compare a small, blurred version of the input frame to all the KFs,
 * choose the closest match, and then estimate a camera rotation by direct image
 * minimisation.
 *
 ****************************************************************************************/

#ifndef __RELOCALISER_FABMAP_H
#define __RELOCALISER_FABMAP_H

#include <mcptam/TaylorCamera.h>
#include <mcptam/Types.h>
#include <TooN/se3.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

class Map;
class GLWindow2;
class KeyFrame;
struct Level;

namespace cv {
	class BOWImgDescriptorExtractor;
  class DescriptorMatcher;
  class DescriptorExtractor;
  class Mat;
  class DMatch;
  class FeatureDetector;
  
  namespace of2 {
    class FabMap;
  }
}

/** @brief Attempts to find the best pose for a given KeyFrame by comparing to others in the Map.
 * 
 * Uses the SmallBlurryImage to find the best KeyFrame match, then computes the 2D transform between
 * them, then finally finds the 3D transform that would have generated that 2D transform. This class
 * is essentially directly from PTAM */
class RelocaliserFabMap
{
public:
  /** @brief Used when there is only one camera in the system
   *  @param map The Map being worked on
   *  @param camera The TaylorCamera model
   *  @param camName The name of the camera */
  RelocaliserFabMap(Map &map, GLWindow2& window, ImageRefMap offsets, TaylorCamera camera, std::string camName);
  
  /** @param map The Map being worked on
   *  @param cameras The TaylorCamera models */
  RelocaliserFabMap(Map &map, GLWindow2& window, ImageRefMap offsets, TaylorCameraMap cameras);
  
  ~RelocaliserFabMap();
  
  bool FindBestPose(MultiKeyFrame &mkfCurrent, bool bDraw);
  
  void Add(MultiKeyFrame &mkfCurrent);
  
  /** @brief Get the best pose found from the last AttemptRecovery
   *  @return The pose */
  TooN::SE3<> BestPose();
  
  void Reset();
  
  static double sdMinLoopProbability;   
  static int snMinFinalMatches;
  static double sdMaxPixelError;
  static int snNumFitTrials;
  
protected:

  void Init();
  
  void InitFromMap();
  
  void InitFromFiles();
  
  void ComputeBoW(Level& level);
  
  void ComputeFinalMatchDescriptors(KeyFrame& kf, cv::Mat& matDescriptors, std::vector<TooN::Vector<2> >& vRootPos);
  
  void GatherMeasurementDescriptors(KeyFrame& kf, cv::Mat& matDescriptors, std::vector<MapPoint*>& vpMapPoints);
  
  double OptimizePose(KeyFrame& kfSrc, const KeyFrame& kfTarget, const std::vector<TooN::Vector<2> >& vSourceRootPos, const std::vector<MapPoint*>& vpMapPoints, const std::vector<cv::DMatch>& vMatches);
  
  TooN::Vector<6> PoseUpdateStep(KeyFrame& kf, TrackerDataPtrVector& vTD, int nIter, double dOverrideSigma, double& dMeanWeightedErrorSquared);
  
  TooN::Vector<6> CalcPoseUpdate(TrackerDataPtrVector& vTD, double dOverrideSigma, double& dMeanWeightedErrorSquared);
  
  double EvaluateMatch(KeyFrame& kfSource, cv::Mat& matSourceDescriptors, std::vector<TooN::Vector<2> >& vSourceRootPos, KeyFrame& kfTarget, CVD::ImageRef irTargetOffset=CVD::ImageRef(-1,-1));
  
  
  Map &mMap;   ///< Reference to the Map
  GLWindow2& mGLWindow;
  ImageRefMap mmDrawOffsets;
  TaylorCameraMap mmCameraModels;   ///< All the camera models being used in the system
  TooN::SE3<> mse3Best;   ///< The 3D pose transform of the last attempted recovery
  
  cv::of2::FabMap* mpFabMap;
  
  int nFabMapSize;
  
  ros::NodeHandle mNodeHandlePriv;  ///< ROS node handle with private namespace
  
  std::map<int, KeyFrame*> mmFabMapToKeyFrame;
  
  cv::Ptr<cv::DescriptorMatcher> mpFinalMatcher;
  cv::Ptr<cv::DescriptorExtractor> mpExtractor;
  cv::Ptr<cv::FeatureDetector> mpDetector;
  cv::Ptr<cv::BOWImgDescriptorExtractor> mpBoWExtractor;
  
  boost::mutex mMutex;
  
  double mdMaxPixelErrorSquared;

};

#endif

