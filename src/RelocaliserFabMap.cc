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
#include <mcptam/PersistentFREAK.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/MapPoint.h>
#include <mcptam/Map.h>
#include <mcptam/TrackerData.h>
#include <mcptam/MEstimator.h>
#include <mcptam/GLWindow2.h>

#include <p3p/P3p.h>

#include <TooN/wls.h>
//#include <cvd/utility.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <cvd/gl_helpers.h>

// Static member
double RelocaliserFabMap::sdMinLoopProbability = 0.1;
int RelocaliserFabMap::snMinFinalMatches = 5;
double RelocaliserFabMap::sdMaxPixelError = 10.0;
int RelocaliserFabMap::snNumFitTrials = 600;


RelocaliserFabMap::RelocaliserFabMap(Map &map, GLWindow2& window, ImageRefMap offsets, TaylorCamera camera, std::string camName)
: mMap(map)
, mGLWindow(window)
, mmDrawOffsets(offsets)
, mNodeHandlePriv("~")
{
  mmCameraModels.insert(std::pair<std::string,TaylorCamera>(camName, camera));
  mdMaxPixelErrorSquared = RelocaliserFabMap::sdMaxPixelError * RelocaliserFabMap::sdMaxPixelError;
  
  cv::initModule_nonfree();
  Init();
}

RelocaliserFabMap::RelocaliserFabMap(Map &map, GLWindow2& window, ImageRefMap offsets, TaylorCameraMap cameras)
: mMap(map)
, mGLWindow(window)
, mmDrawOffsets(offsets)
, mmCameraModels(cameras)
, mNodeHandlePriv("~")
{
  mdMaxPixelErrorSquared = RelocaliserFabMap::sdMaxPixelError * RelocaliserFabMap::sdMaxPixelError;
  
  cv::initModule_nonfree();
  Init();
};


void RelocaliserFabMap::Init()
{
  mpDetector = cv::Ptr<cv::FeatureDetector>(new cv::StarFeatureDetector()); //(32, 10, 18, 18, 20);
  mpFinalMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher());
  mpExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SURF(1000, 4, 2, false, true)); //new cv::FREAK(true, true, 22.0f, 4);
  
  nFabMapSize = 0;
  mmFabMapToKeyFrame.clear();
  
  if(mMap.mbGood)
  {
    InitFromMap();
  }
  else
  {
    InitFromFiles();
  }
}

void RelocaliserFabMap::InitFromMap()
{
  std::cout<<"================================================"<<std::endl;
  std::cout<<"======= RelocaliserFabMap::InitFromMap ======="<<std::endl;
  std::cout<<"================================================"<<std::endl;
  
  cv::of2::BOWMSCTrainer trainer;
  
  std::map<KeyFrame*, std::vector<cv::KeyPoint> > mKeyFrameKeyPoints;
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      Level& level = kf.maLevels[RELOC_LEVEL];
      
      cv::Mat imageWrapped(level.image.size().y, level.image.size().x, CV_8U, level.image.data(), level.image.row_stride());
      std::vector<cv::KeyPoint>& vKeyPoints = mKeyFrameKeyPoints[&kf];
      mpDetector->detect(imageWrapped, vKeyPoints);
      
      if(vKeyPoints.empty())
        continue;
        
      cv::Mat matDescriptors;
      mpExtractor->compute(imageWrapped, vKeyPoints, matDescriptors);
      
      trainer.add(matDescriptors);
    }
  }
  
  cv::Mat matVocabulary = trainer.cluster();
  
	cv::Ptr<cv::DescriptorMatcher> pMatcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce"
	mpBoWExtractor = cv::Ptr<cv::BOWImgDescriptorExtractor>(new cv::BOWImgDescriptorExtractor(mpExtractor, pMatcher));
	mpBoWExtractor->setVocabulary(matVocabulary);
  
  cv::Mat matTrainData;
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      Level& level = kf.maLevels[RELOC_LEVEL];
      
      cv::Mat imageWrapped(level.image.size().y, level.image.size().x, CV_8U, level.image.data(), level.image.row_stride());
      ROS_ASSERT(mKeyFrameKeyPoints.count(&kf));
      std::vector<cv::KeyPoint>& vKeyPoints = mKeyFrameKeyPoints[&kf];
      
      if(vKeyPoints.empty())
        continue;
        
      mpBoWExtractor->compute(imageWrapped, vKeyPoints, level.matBoW);
      matTrainData.push_back(level.matBoW);
    }
  }
  
  cv::of2::ChowLiuTree treeBuilder;
  treeBuilder.add(matTrainData);
  cv::Mat matTree = treeBuilder.make();
  
  // Create FabMap object
	int nOptions = 0;
	nOptions |= cv::of2::FabMap::SAMPLED;
	nOptions |= cv::of2::FabMap::CHOW_LIU;
	mpFabMap = new cv::of2::FabMap2(matTree, 0.39, 0, nOptions);
	mpFabMap->addTraining(matTrainData);
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);
      Level& level = kf.maLevels[RELOC_LEVEL];
      
      if(level.matBoW.empty())
        continue;
        
      mpFabMap->add(level.matBoW);
      mmFabMapToKeyFrame[nFabMapSize] = &kf;
      nFabMapSize++;
    }
  }
  
  
}

void RelocaliserFabMap::InitFromFiles()
{
  std::cout<<"================================================"<<std::endl;
  std::cout<<"======= RelocaliserFabMap::InitFromFiles ======="<<std::endl;
  std::cout<<"================================================"<<std::endl;
  
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
  
	cv::Ptr<cv::DescriptorMatcher> pMatcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce"
	mpBoWExtractor = cv::Ptr<cv::BOWImgDescriptorExtractor>(new cv::BOWImgDescriptorExtractor(mpExtractor, pMatcher));
	mpBoWExtractor->setVocabulary(matVocabulary);
};

RelocaliserFabMap::~RelocaliserFabMap()
{
  delete mpFabMap;
}

void RelocaliserFabMap::Reset()
{
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
  /*
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
  */
  
  cv::Mat imageWrapped(level.image.size().y, level.image.size().x, CV_8U, level.image.data(), level.image.row_stride());
  std::vector<cv::KeyPoint> vKeyPoints;
  mpDetector->detect(imageWrapped, vKeyPoints);
  
	if(vKeyPoints.empty())
    return;
  
  mpBoWExtractor->compute(imageWrapped, vKeyPoints, level.matBoW);
}

void RelocaliserFabMap::ComputeFinalMatchDescriptors(KeyFrame& kf, cv::Mat& matDescriptors, std::vector<TooN::Vector<2> >& vRootPos)
{
  kf.MakeExtractor();
  
  Level& level = kf.maLevels[RELOC_LEVEL];
  
  // Put all corners from level into a vector of OpenCV keypoints
  std::vector<cv::KeyPoint> vKeyPoints(level.vCorners.size());
  
  for(unsigned i=0; i < level.vCorners.size(); ++i)
  {
    cv::KeyPoint& keyPoint = vKeyPoints[i];
    
    keyPoint.pt.x = level.vCorners[i].x;
    keyPoint.pt.y = level.vCorners[i].y;
    keyPoint.size = 7.f;  // Got this value from the OpenCV implementation of FAST. If the detector type is changed in KeyFrame this should change too
  }
    
  kf.mpExtractor->compute(vKeyPoints, matDescriptors);
  
  // Some keypoints might have been removed. Translate vKeyPoints back to vectors in level zero
  vRootPos.resize(vKeyPoints.size());
  for(unsigned i=0; i < vKeyPoints.size(); ++i)
  {
    vRootPos[i] = LevelZeroPos(TooN::makeVector(vKeyPoints[i].pt.x, vKeyPoints[i].pt.y), RELOC_LEVEL);
  }
}

void RelocaliserFabMap::GatherMeasurementDescriptors(KeyFrame& kf, cv::Mat& matDescriptors, std::vector<MapPoint*>& vpMapPoints)
{
  cv::Mat matTemp;
  int nInsertRow = 0;
  
  for(MeasPtrMap::iterator meas_it = kf.mmpMeasurements.begin(); meas_it != kf.mmpMeasurements.end(); ++meas_it)
  {
    MapPoint& point = *(meas_it->first);
    Measurement& meas = *(meas_it->second);
    
    if(point.mbBad) 
      continue;
      
    if(point.mbDeleted) 
      continue;
    
    if(meas.nLevel != RELOC_LEVEL)
      continue;
      
    // Need to build descriptors for those measurements that were gotten by the tracker
    // and handed to the map maker!
    
    if(meas.matDescriptor.empty())
      continue;
    
    if(matTemp.empty())
    {
      // Make the number of rows the maximum possible number
      matTemp = cv::Mat((int)kf.mmpMeasurements.size(), meas.matDescriptor.cols, meas.matDescriptor.type());
    }
    
    meas.matDescriptor.copyTo(matTemp.row(nInsertRow));
    vpMapPoints.push_back(&point);
    nInsertRow++;
  }
  
  // Chop off rows that were not used
  matTemp.resize(nInsertRow);
  matDescriptors = matTemp;
  
  ROS_ASSERT(matDescriptors.rows == (int)vpMapPoints.size());
}

double RelocaliserFabMap::EvaluateMatch(KeyFrame& kfSource, cv::Mat& matSourceDescriptors, std::vector<TooN::Vector<2> >& vSourceRootPos, KeyFrame& kfTarget, CVD::ImageRef irTargetOffset)
{
  cv::Mat matTargetDescriptors;
  std::vector<MapPoint*> vpTargetMapPoints;
  
  GatherMeasurementDescriptors(kfTarget, matTargetDescriptors, vpTargetMapPoints);
    
  std::vector<cv::DMatch> vFinalMatches;
  mpFinalMatcher->match(matTargetDescriptors, matSourceDescriptors, vFinalMatches);
  
  std::cout<<"Got "<<vFinalMatches.size()<<" correspondences!"<<std::endl;
  
  double dScore;
  
  if((int)vFinalMatches.size() < RelocaliserFabMap::snMinFinalMatches)
  {
    std::cout<<"Not enough, minimum is "<< RelocaliserFabMap::snMinFinalMatches<<std::endl;
    dScore = std::numeric_limits<double>::max();
  }
  else
  {
    dScore = OptimizePose(kfSource, kfTarget, vSourceRootPos, vpTargetMapPoints, vFinalMatches);
    std::cout<<"Optimization score: "<<dScore<<std::endl;
  }
  
  if(irTargetOffset.x >= 0 && irTargetOffset.y >= 0)
  {
    glRasterPos(irTargetOffset);
    glDrawPixels(kfTarget.maLevels[RELOC_LEVEL].image);
    
    // draw dots for measurements with descriptors in target KF
    glColor3f(0,1,0);  
    glPointSize(3); 
    glBegin(GL_POINTS);
    
    for(unsigned i=0; i < vpTargetMapPoints.size(); ++i)
    {
      MapPoint* pPoint = vpTargetMapPoints[i];
      
      if(!kfTarget.mmpMeasurements.count(pPoint))  // the measurement was deleted since vpTargetMapPoints was created
        continue;
      
      Measurement* pMeas = kfTarget.mmpMeasurements[pPoint];
      
      CVD::glVertex(CVD::vec(irTargetOffset) + LevelNPos(pMeas->v2RootPos, RELOC_LEVEL));
    }
    
    glEnd();
    
    mGLWindow.PrintString(irTargetOffset + CVD::ImageRef(8,8), std::to_string(dScore), 10);
  }
  
  return dScore;
}

bool RelocaliserFabMap::FindBestPose(MultiKeyFrame &mkfCurrent, bool bDraw)
{
  if(nFabMapSize == 0)
    return false;
    
  TooN::SE3<> se3Saved = mkfCurrent.mse3BaseFromWorld;  // in case we want to put it back
  
  std::vector<std::pair<double, TooN::SE3<> > > vMatchesEvaluated;
  
  for(KeyFramePtrMap::iterator kf_it = mkfCurrent.mmpKeyFrames.begin(); kf_it != mkfCurrent.mmpKeyFrames.end(); ++kf_it)
  {
    KeyFrame& kf = *(kf_it->second);
    Level& level = kf.maLevels[RELOC_LEVEL];
    
    if(!kf.mbActive)
      continue;
    
    ComputeBoW(level);
    
    if(level.matBoW.empty())
    {
      std::cerr<<"BoW empty, skipping "<<kf.mCamName<<std::endl;
      continue;
    }
    
    std::vector<cv::of2::IMatch> vFabMapMatches;
    
    boost::mutex::scoped_lock lock(mMutex);
    mpFabMap->compare(level.matBoW, vFabMapMatches);
    lock.unlock();
    
    std::vector<cv::of2::IMatch> vFabMapMatchesPruned;
    
    for(unsigned i=0; i < vFabMapMatches.size(); ++i)
    {
      cv::of2::IMatch& match = vFabMapMatches[i];
      
      if(match.imgIdx < 0) // Not a match to an image in FabMap
        continue;
        
      if(match.match < RelocaliserFabMap::sdMinLoopProbability)  // Match probability too low
        continue;
        
      vFabMapMatchesPruned.push_back(match);
    }
    
    if(vFabMapMatchesPruned.empty())  // No match found for this KF
    {
      std::cerr<<"No match found for "<<kf.mCamName<<std::endl;
      continue;
    }
      
    std::cout<<"Kept "<<vFabMapMatchesPruned.size()<<" matches out of "<<vFabMapMatches.size()<<std::endl;
    
    // Sort the matches in increasing order ( the < operator is already defined in IMatch)
    std::sort(vFabMapMatchesPruned.begin(), vFabMapMatchesPruned.end());
    
    // Look only at top matches (max 3). They will be at the end since the matches were sorted in ascending order.
    int nTopMatches = (int)vFabMapMatchesPruned.size() < 3 ? (int)vFabMapMatchesPruned.size() : 3;
    std::vector<cv::of2::IMatch> vTopMatches(vFabMapMatchesPruned.begin() + ((int)vFabMapMatchesPruned.size() - nTopMatches), vFabMapMatchesPruned.end());
    
    cv::Mat matSourceDescriptors;
    std::vector<TooN::Vector<2> > vSourceRootPos;
    ComputeFinalMatchDescriptors(kf, matSourceDescriptors, vSourceRootPos);
    
    CVD::ImageRef irSourceOffset = mmDrawOffsets[kf.mCamName];
    
    if(bDraw)
    {
      // top left: source KF
      glRasterPos(irSourceOffset);
      glDrawPixels(level.image);
      
      // Draw dots corners in source KF that made it into the descriptor Mat
      glColor3f(1,0,1);  
      glPointSize(3); 
      glBegin(GL_POINTS);
      
      for(unsigned i=0; i < vSourceRootPos.size(); ++i)
      {
        CVD::glVertex(CVD::vec(irSourceOffset) + LevelNPos(vSourceRootPos[i], RELOC_LEVEL));
      }
      
      glEnd();
      
      // Draw lines connecting target points to matching source points
      /*
      glLineWidth(2);
      glEnable(GL_POINT_SMOOTH);
      glEnable(GL_LINE_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glBegin(GL_LINES);
      
      for(unsigned i=0; i < vFinalMatches.size(); ++i)
      {
        const cv::DMatch& match = vFinalMatches[i];
        MapPoint* pPoint = vpTargetMapPoints[match.queryIdx];
        Measurement* pMeas = pKFTarget->mmpMeasurements[pPoint];
        
        glColor3f(0,1,0);
        CVD::glVertex(CVD::vec(irTargetOffset) + LevelNPos(pMeas->v2RootPos, RELOC_LEVEL));
        glColor3f(1,0,1);
        CVD::glVertex(CVD::vec(irSourceOffset) + LevelNPos(vSourceRootPos[match.trainIdx], RELOC_LEVEL));
      }
      
      glEnd();
      */ 
    }
    
    for(unsigned i=0; i < vTopMatches.size(); ++i)
    {
      cv::of2::IMatch& match = vTopMatches[i];
      
      ROS_ASSERT(mmFabMapToKeyFrame.count(match.imgIdx));
      KeyFrame* pKFTarget = mmFabMapToKeyFrame[match.imgIdx];
      
      if(pKFTarget == NULL) // erased when MKF is deleted
        continue;
        
      std::cout<<"Matched current "<<kf.mCamName<<" to "<<pKFTarget->mpParent->mnID<<"::"<<pKFTarget->mCamName<<std::endl;
      
      CVD::ImageRef irTargetOffset(-1,-1);
      
      if(bDraw)
      {
        if(i == 0)
          irTargetOffset = irSourceOffset + CVD::ImageRef(level.image.size().x, 0);
        else if(i == 1)
          irTargetOffset = irSourceOffset + CVD::ImageRef(0, level.image.size().y);
        else if(i == 2)
          irTargetOffset = irSourceOffset + CVD::ImageRef(level.image.size().x, level.image.size().y);
      }
      
      double dScore = EvaluateMatch(kf, matSourceDescriptors, vSourceRootPos, *pKFTarget, irTargetOffset);
      vMatchesEvaluated.push_back(std::make_pair(dScore, kf.mpParent->mse3BaseFromWorld));
    }
  }
  
  if(vMatchesEvaluated.empty())
  {
    std::cout<<"Could not match current MKF!"<<std::endl;
    mkfCurrent.mse3BaseFromWorld = se3Saved;
    mkfCurrent.UpdateCamsFromWorld();
    return false;
  }
  
  std::sort(vMatchesEvaluated.begin(), vMatchesEvaluated.end(), [](std::pair<double, TooN::SE3<> > a, std::pair<double, TooN::SE3<> > b)
                                                                  { return a.first < b.first; });
         
  std::cout<<"Best match score: "<<vMatchesEvaluated[0].first<<std::endl;
  std::cout<<"Best match pose: "<<std::endl<<vMatchesEvaluated[0].second<<std::endl;
  std::cout<<"Pose before match was: "<<std::endl<<se3Saved<<std::endl;
                                                                  
  // smallest score is best, so pick the first element
  mkfCurrent.mse3BaseFromWorld = vMatchesEvaluated[0].second;
  mkfCurrent.UpdateCamsFromWorld();
  return true;
}

double RelocaliserFabMap::OptimizePose(KeyFrame& kfSrc, const KeyFrame& kfTarget, const std::vector<TooN::Vector<2> >& vSourceRootPos, const std::vector<MapPoint*>& vpMapPoints, const std::vector<cv::DMatch>& vMatches)
{
  // The source descriptors correspond to the trainIdx, and the target descriptors correspond to the queryIdx
  // Build an optimization based on TrackerData structures, just like in Tracker
  // The initial pose of kfSrc will be equal to kfTarget, and we'll refine it from there
  
  TaylorCamera& camera = mmCameraModels[kfSrc.mCamName];
  
  //kfSrc.mse3CamFromWorld = kfTarget.mse3CamFromWorld;
  //kfSrc.mpParent->mse3BaseFromWorld = kfTarget.mpParent->mse3BaseFromWorld;
  //kfSrc.mpParent->UpdateCamsFromWorld();

  // To cache unprojections  
  std::vector<TooN::Vector<3> > vSourceUnProj(vSourceRootPos.size(), TooN::makeVector(0,0,0));

  P3p p3p;
  
  boost::mutex::scoped_lock lock(mMap.mMutex);
  
  // Do 300 MLESAC trials.
  double dBestError = std::numeric_limits<double>::max();
  TooN::SE3<> se3BestSolutionPose;
  for(int nR = 0; nR < RelocaliserFabMap::snNumFitTrials ; nR++)
  { 
    // Find set of three unique matches
    std::vector<int> vChosenIndices(3,-1);
    for(int i=0; i < 3; i++)
    {
      bool bIsUnique = false;
      int n;
      while(!bIsUnique)
      {
        n = rand() % vMatches.size();
        bIsUnique =true;
        for(int j=0; j<i && bIsUnique; j++)
        {
          if(vChosenIndices[j] == n)
            bIsUnique = false;
        }
      }
      vChosenIndices[i] = n;
    }
    
    TooN::Matrix<3,3> m3FeatureVectors;
    TooN::Matrix<3,3> m3WorldPoints;
    
    for(int i=0; i < 3; ++i)
    {
      const cv::DMatch& match = vMatches[vChosenIndices[i]];
      MapPoint* pPoint = vpMapPoints[match.queryIdx];
      
      TooN::Vector<3> v3UnProj;
      if(vSourceUnProj[match.trainIdx] == TooN::makeVector(0,0,0))
      {
        vSourceUnProj[match.trainIdx] = camera.UnProject(vSourceRootPos[match.trainIdx]);
      }
        
      v3UnProj = vSourceUnProj[match.trainIdx];
 
      m3FeatureVectors.T()[i] = v3UnProj;
      m3WorldPoints.T()[i] = pPoint->mv3WorldPos;
    }
    
    TooN::Matrix<3,16> mSolutions;
    int nRetVal = p3p.computePoses(m3FeatureVectors, m3WorldPoints, mSolutions);
    
    if(nRetVal < 0)
      continue;
      
    //std::cout<<"Solutions: "<<std::endl<<mSolutions<<std::endl;
    
    // Four possible poses, extract them from mSolutions
    for(int i=0; i < 4; ++i)
    {
      TooN::Vector<3> v3Translation = mSolutions.T()[i*4];
      TooN::Matrix<3,3> m3Rotation = mSolutions.slice(0,i*4+1,3,3);
      
      if(!TooN::isfinite(v3Translation))
        continue;
      
      // This pose transfers points from world to camera
      TooN::SE3<> se3SolutionPose = TooN::SE3<>(TooN::SO3<>(m3Rotation), v3Translation).inverse();
      
      double dError = 0.0;
      for(unsigned j=0; j < vMatches.size(); ++j)
      {
        const cv::DMatch& match = vMatches[j];
        MapPoint* pPoint = vpMapPoints[match.queryIdx];
        
        TooN::Vector<3> v3PointInCam = se3SolutionPose * pPoint->mv3WorldPos;
        TooN::Vector<2> v2Projected = camera.Project(v3PointInCam);
        TooN::Vector<2> v2Image = vSourceRootPos[match.trainIdx];
        TooN::Vector<2> v2PixelError = v2Image - v2Projected;
        
        double dSquaredError = v2PixelError * v2PixelError;
        if(dSquaredError > mdMaxPixelErrorSquared)
          dSquaredError = mdMaxPixelErrorSquared;
        
        dError += dSquaredError;
      }
      
      if(dError < dBestError)
      {
        se3BestSolutionPose = se3SolutionPose;
        dBestError = dError;
      }
    }
  }
  
  //testing
  double dMeanWeightedErrorSquared = dBestError;
  
  //kfSrc.mse3CamFromWorld = se3BestSolutionPose;
  kfSrc.mpParent->mse3BaseFromWorld = kfSrc.mse3CamFromBase.inverse() * se3BestSolutionPose;
  kfSrc.mpParent->UpdateCamsFromWorld();
  
  /*
  TrackerDataPtrVector vTD;
  
  for(unsigned i=0; i < vMatches.size(); ++i)
  {
    const cv::DMatch& match = vMatches[i];
    MapPoint* pPoint = vpMapPoints[match.queryIdx];
    
    // Ensure that this map point has an associated TrackerData struct.
    if(!pPoint->mmpTData.count(kfSrc.mCamName))
    {
      pPoint->mmpTData[kfSrc.mCamName] = new TrackerData(pPoint, camera.GetImageSize());
    }
    
    // Use boost intrusive_ptr to increment MapPoint's using counter, preventing its deletion while we're
    // holding onto this pointer
    boost::intrusive_ptr<TrackerData> pTData(pPoint->mmpTData[kfSrc.mCamName]);
    
    // Project according to current view. Don't skip if it's not in image, since se3CamFromWorld could be
    // arbitrarily wrong
    pTData->Project(kfSrc.mse3CamFromWorld,  camera); 
    
    // Calculate camera projection derivatives of this point.
    pTData->GetDerivsUnsafe(camera);
    
    pTData->mbFound = true;
    pTData->mdSqrtInvNoise = (1.0 / LevelScale(RELOC_LEVEL));
    pTData->mv2Found = vSourceRootPos[match.trainIdx];
    
    vTD.push_back(pTData);
  }
  
  double dMeanWeightedErrorSquared;
  for(int iter = 0; iter<20; iter++) // If so: do ten Gauss-Newton pose updates iterations.
  {
    PoseUpdateStep(kfSrc, vTD, iter, 5.0, dMeanWeightedErrorSquared);
  }
  */
  
  return dMeanWeightedErrorSquared;
}

TooN::Vector<6> RelocaliserFabMap::PoseUpdateStep(KeyFrame& kf, TrackerDataPtrVector& vTD, int nIter, double dOverrideSigma, double& dMeanWeightedErrorSquared)
{
  TaylorCamera& camera = mmCameraModels[kf.mCamName];
 
  if(nIter != 0)
  { // Re-project the points on all but the first iteration.
    for(unsigned int i=0; i < vTD.size(); i++)
    {
      vTD[i]->ProjectAndDerivs(kf.mse3CamFromWorld, camera);
    }
  }
  
  for(unsigned int i=0; i < vTD.size(); i++)
  {
    vTD[i]->CalcJacobian(kf.mpParent->mse3BaseFromWorld, kf.mse3CamFromBase);
  }
        
  // Force the MEstimator to be pretty brutal with outliers beyond the fifth iteration.
  if(nIter <= 5)
    dOverrideSigma = 0.0;
      
  // Calculate and apply the pose update...
  TooN::Vector<6> v6Update = CalcPoseUpdate(vTD, dOverrideSigma, dMeanWeightedErrorSquared);
  kf.mpParent->mse3BaseFromWorld = TooN::SE3<>::exp(v6Update) * kf.mpParent->mse3BaseFromWorld;
  
  // Update the KeyFrame cam-from-world poses
  kf.mpParent->UpdateCamsFromWorld();
    
  return v6Update;
}

TooN::Vector<6> RelocaliserFabMap::CalcPoseUpdate(TrackerDataPtrVector& vTD, double dOverrideSigma, double& dMeanWeightedErrorSquared)
{
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  std::vector<double> vErrorSquared;
  for(unsigned int j=0; j < vTD.size(); ++j)
  {
    TrackerData &td = *vTD[j];      
    td.mv2Error_CovScaled = td.mdSqrtInvNoise * (td.mv2Found - td.mv2Image);
    vErrorSquared.push_back(td.mv2Error_CovScaled * td.mv2Error_CovScaled);
  }
  
  // No valid measurements? Return null update.
  if(vErrorSquared.size() == 0)
    return TooN::makeVector(0,0,0,0,0,0);
  
  // Find the sigma squared that will be used in assigning weights
  double dSigmaSquared;
  if(dOverrideSigma > 0)
    dSigmaSquared = dOverrideSigma; 
  else
  {
    dSigmaSquared = Huber::FindSigmaSquared(vErrorSquared);
  }
  
  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  TooN::WLS<6> wls; //, wls_noweight;
  wls.add_prior(100.0); // Stabilising prior
  
  double dWeightedErrorSquaredSum = 0;
  
  for(unsigned int j=0; j < vTD.size(); ++j)
  {
    TrackerData &td = *vTD[j];
    TooN::Vector<2> &v2Error = td.mv2Error_CovScaled;
    double dErrorSq = v2Error * v2Error;
    double dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
    
    dWeightedErrorSquaredSum += dErrorSq * dWeight;
    
    TooN::Matrix<2,6> &m26Jac = td.mm26Jacobian;
    wls.add_mJ(v2Error[0], td.mdSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
    wls.add_mJ(v2Error[1], td.mdSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
  }
    
  wls.compute();
  
  dMeanWeightedErrorSquared = dWeightedErrorSquaredSum / (double)vTD.size();
  
  return wls.get_mu();
}
