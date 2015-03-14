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

#include <mcptam/RelocaliserFabMap.h>
#include <mcptam/KeyFrame.h>
//#include <cvd/utility.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

// Static member
double RelocaliserFabMap::sdMinLoopProbability = 0.8;

RelocaliserFabMap::RelocaliserFabMap(Map &map, TaylorCamera camera, std::string camName)
: mMap(map)
, mNodeHandlePriv("~")
{
  mmCameraModels.insert(std::pair<std::string,TaylorCamera>(camName, camera));
  
  cv::initModule_nonfree();
  Init();
}

RelocaliserFabMap::RelocaliserFabMap(Map &map, TaylorCameraMap cameras)
  : mMap(map)
  , mmCameraModels(cameras)
  , mNodeHandlePriv("~")
{
  cv::initModule_nonfree();
  Init();
};


void RelocaliserFabMap::Init()
{
  std::string trainingDataPath;
  mNodeHandlePriv.param<std::string>("fabmap_training_data", trainingDataPath ,"");
  
  if(trainingDataPath.empty())
  {
    ROS_FATAL("RelocalizerFabMap: no FabMap training data given, must specify fabmap_training_data parameter!");
    ros::shutdown();
    return;
  }
  
  std::string vocabularyPath;
  mNodeHandlePriv.param<std::string>("fabmap_vocabulary", vocabularyPath ,"");
  
  if(vocabularyPath.empty())
  {
    ROS_FATAL("RelocalizerFabMap: no FabMap vocabularyPath given, must specify fabmap_vocabulary parameter!");
    ros::shutdown();
    return;
  }
  
  std::string treePath;
  mNodeHandlePriv.param<std::string>("fabmap_tree", treePath ,"");
  
  if(treePath.empty())
  {
    ROS_FATAL("RelocalizerFabMap: no FabMap Chow-Liu Tree given, must specify fabmap_tree parameter!");
    ros::shutdown();
    return;
  }
  
	cv::FileStorage fileStorage;
  
  // Load training data
  fileStorage.open(trainingDataPath, cv::FileStorage::READ);
  
  cv::Mat matTrainingData;
	fileStorage["BOWImageDescs"] >> matTrainingData;
	if (matTrainingData.empty()) 
  {
    ROS_ERROR_STREAM("RelocalizerFabMap: Training data not found!");
    ros::shutdown();
    return;
	}
  
  fileStorage.release();
  
  // Load vocabulary data
  fileStorage.open(vocabularyPath, cv::FileStorage::READ);
  
  cv::Mat matVocabulary;
	fileStorage["Vocabulary"] >> matVocabulary;
	if (matVocabulary.empty()) 
  {
		ROS_ERROR_STREAM("RelocalizerFabMap: Vocabulary not found!");
    ros::shutdown();
    return;
	}
  
	fileStorage.release();

	// Load the tree
	fileStorage.open(treePath, cv::FileStorage::READ);
  
	cv::Mat matTree;
	fileStorage["ChowLiuTree"] >> matTree;
	if (matTree.empty()) 
  {
		ROS_ERROR_STREAM("RelocalizerFabMap: Chow Liu tree not found!");
    ros::shutdown();
    return;
	}
  
	fileStorage.release();
	
	// Create FabMap object
	int nOptions = 0;
	nOptions |= cv::of2::FabMap::SAMPLED;
	nOptions |= cv::of2::FabMap::CHOW_LIU;
	mpFabMap = new cv::of2::FabMap2(matTree, 0.39, 0, nOptions);
	mpFabMap->addTraining(matTrainingData);
  
  cv::Ptr<cv::DescriptorExtractor> pExtractor = new cv::SURF(1000, 4, 2, false, true); //new cv::FREAK(true, true, 22.0f, 4);
	cv::Ptr<cv::DescriptorMatcher> pMatcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce"
	mpDescriptorExtractor = new cv::BOWImgDescriptorExtractor(pExtractor, pMatcher);
	mpDescriptorExtractor->setVocabulary(matVocabulary);
  
  nFabMapSize = 0;
  
};

RelocaliserFabMap::~RelocaliserFabMap()
{
  delete mpDescriptorExtractor;
  delete mpFabMap;
}

void RelocaliserFabMap::Reset()
{
  delete mpDescriptorExtractor;
  delete mpFabMap;
  
  Init();
}

TooN::SE3<> RelocaliserFabMap::BestPose()
{
  return mse3Best;
}

void RelocaliserFabMap::Add(MultiKeyFrame &mkfCurrent)
{
  for(KeyFramePtrMap::iterator kf_it = mkfCurrent.mmpKeyFrames.begin(); kf_it != mkfCurrent.mmpKeyFrames.end(); ++kf_it)
  {
    KeyFrame& kf = *(kf_it->second);
    Level& level = kf.maLevels[RELOC_LEVEL];
    
    ComputeBoW(level);
    
    if(level.matBoW.empty())
      continue;
      
    boost::mutex::scoped_lock lock(mMutex);
    mpFabMap->add(level.matBoW);
    mmFabMapToKeyFrame[nFabMapSize] = &kf;
    nFabMapSize++;
  }
}

void RelocaliserFabMap::ComputeBoW(Level& level)
{
  if(level.vCorners.empty())
    return;
  
  // Put all corners from level into a vector of OpenCV keypoints
  std::vector<cv::KeyPoint> vKeyPoints(level.vCorners.size());
  
  for(unsigned i=0; i < level.vCorners.size(); ++i)
  {
    cv::KeyPoint& keyPoint = vKeyPoints[i];
    
    keyPoint.pt.x = level.vCorners[i].x;
    keyPoint.pt.y = level.vCorners[i].y;
    keyPoint.size = 7.f;  // Got this value from the OpenCV implementation of FAST. If the detector type is changed in KeyFrame this should change too
  }
  
  cv::Mat imageWrapped(level.image.size().y, level.image.size().x, CV_8U, level.image.data(), level.image.row_stride());
  
  mpDescriptorExtractor->compute(imageWrapped, vKeyPoints, level.matBoW);
}

bool RelocaliserFabMap::FindBestPose(MultiKeyFrame &mkfCurrent)
{
  if(nFabMapSize == 0)
    return false;
  
  for(KeyFramePtrMap::iterator kf_it = mkfCurrent.mmpKeyFrames.begin(); kf_it != mkfCurrent.mmpKeyFrames.end(); ++kf_it)
  {
    KeyFrame& kf = *(kf_it->second);
    Level& level = kf.maLevels[RELOC_LEVEL];
    
    ComputeBoW(level);
    
    if(level.matBoW.empty())
      continue;
    
    std::vector<cv::of2::IMatch> vFabMapMatches;
    
    boost::mutex::scoped_lock lock(mMutex);
    mpFabMap->compare(level.matBoW, vFabMapMatches);
    lock.unlock();
    
    int nMatchingImageIdx = -1;
    
    for(unsigned i=0; i < vFabMapMatches.size(); ++i)
    {
      cv::of2::IMatch& match = vFabMapMatches[i];
      
      if(match.imgIdx < 0) // Not a match to an image in FabMap
        continue;
        
      if(match.match < RelocaliserFabMap::sdMinLoopProbability)  // Match probability too low
        continue;
        
      nMatchingImageIdx = match.imgIdx;
      break;
    }
    
    if(nMatchingImageIdx == -1)  // No match found for this KF
      continue;

    ROS_ASSERT(mmFabMapToKeyFrame.count(nMatchingImageIdx));
    KeyFrame* pKFTarget = mmFabMapToKeyFrame[nMatchingImageIdx];
    
    if(pKFTarget == NULL) // erased when MKF is deleted
      continue;
    
    std::cout<<"Matched current "<<kf.mCamName<<" to "<<pKFTarget->mpParent->mnID<<"::"<<pKFTarget->mCamName<<std::endl;
    return true;
    
    // Try to match kf and pKFTarget
  }
  
  std::cout<<"Could not match current MKF!"<<std::endl;
  
  return false;
}
