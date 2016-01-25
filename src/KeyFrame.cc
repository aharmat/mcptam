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


// Copyright 2008 Isis Innovation Limited

//=========================================================================================
//
// Modifications
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#include <mcptam/KeyFrame.h>
#include <mcptam/ShiTomasi.h>
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/MapPoint.h>
#include <mcptam/MEstimator.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/MiniPatch.h>
#include <mcptam/Utility.h>
#include <cvd/vision.h>
#include <cvd/fast_corner.h>
#include <cvd/convolution.h>
#include <ros/ros.h>
#include <cmath>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cvd/vector_image_ref.h>
#include <cvd/draw.h>
#include <cvd/morphology.h>

using namespace TooN;

// ----------------------------------------------- KeyFrame ------------------------------------------------------

// Static members
double KeyFrame::sdCandidateThresh = 70;
double KeyFrame::sdDistanceMeanDiffFraction = 0.5;
//double KeyFrame::saThreshDerivs[LEVELS]  = {-600, -200, -50, -12};
double KeyFrame::sdCandidateTopFraction= 0.8;
std::string KeyFrame::ssCandidateType = "fast";
std::string KeyFrame::ssCandidateCriterion = "percent";
bool KeyFrame::sbAdaptiveThresh = true;
int Level::snNumPrev = 2;


KeyFrame::KeyFrame(MultiKeyFrame* pMKF, std::string name)
  : mCamName(name)
  , mpParent(pMKF)
{
  mpSBI = NULL;
  mbActive = false;
  mdSceneDepthMean = MAX_DEPTH;
  mdSceneDepthSigma = MAX_SIGMA;
  
}

void KeyFrame::AddMeasurement(MapPoint* pPoint, Measurement* pMeas)
{
  ROS_ASSERT(!mmpMeasurements.count(pPoint));
  if(mmpMeasurements.count(pPoint)) //safety: we've already added the measurements, don't add again!
	return;
  
  
  boost::mutex::scoped_lock lock(mMeasMutex);
  
  mmpMeasurements[pPoint] = pMeas;
 
 if(!(mpParent->isBufferMKF)) //This keyframe is NOT part of a buffer, add measurements to global points
 {
 	pPoint->mMMData.spMeasurementKFs.insert(this);
 }
 
}

// Erase all measurements
void KeyFrame::ClearMeasurements()
{
	
  boost::mutex::scoped_lock lock(mMeasMutex);
  
  for(MeasPtrMap::iterator it = mmpMeasurements.begin(); it != mmpMeasurements.end(); ++it)
  {
    delete it->second;  // delete the measurement pointers
  }
  
  mmpMeasurements.clear();
		  
}

KeyFrame::~KeyFrame()
{
  if(mpSBI)
    delete mpSBI;
    
  ClearMeasurements();
    
}

// Set the fixed mask to a given image
void KeyFrame::SetMask(CVD::Image<CVD::byte> &m)
{
  maLevels[0].mask = m;
  
  for(int i=1; i<LEVELS; i++)
  {
    Level &lev = maLevels[i];
    lev.mask.resize(maLevels[i-1].mask.size() / 2);
    CVD::halfSample(maLevels[i-1].mask, lev.mask);
  }
}

bool KeyFrame::NoImage()
{
  return maLevels[0].image.totalsize() == 0;
}

void KeyFrame::RemoveImage()
{
  for(int i=0; i<LEVELS; i++)
  {
    Level &lev = maLevels[i];
    
    lev.image = CVD::Image<CVD::byte>();
    lev.mask = CVD::Image<CVD::byte>();
  }
}

// Takes an image and calculates pyramid levels etc to fill the keyframe data structures with everything that's needed by the tracker.
std::tuple<double,double,double> KeyFrame::MakeKeyFrame_Lite(CVD::Image<CVD::byte> &im, bool bDeepCopy, bool bGlareMasking)
{
  // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  
  bool bPushBack = false;
  if(maLevels[0].image.totalsize() > 0)  // only save to list of previous if we currently are holding something
  {
    bPushBack = true;
    maLevels[0].imagePrev.push_back(maLevels[0].image);  // save it
  }
  
  ROS_DEBUG_STREAM("MakeKeyFrame_Lite: push back: "<<(bPushBack ? "yes" : "no"));
  
  if(bDeepCopy) 
  {
    maLevels[0].image.resize(im.size());
    CVD::copy(im, maLevels[0].image);
  }
  else  // Use CVD::Image's reference counting copy to avoid data copying
  {
    maLevels[0].image = im;
  }
  
  ros::WallTime startTime;
  double dFeatureTime = 0;
  double dDownsampleTime = 0;
  double dMaskTime = 0;
  
  // Then, for each level...
  for(int i=0; i<LEVELS; i++)
  {
    Level &lev = maLevels[i];
    
        
    if(i!=0)
    {  
      if(bPushBack)
        lev.imagePrev.push_back(lev.image);  // save image before it's overwritten with downsampled new image
        
      startTime = ros::WallTime::now();
      
      // .. make a half-size image from the previous level..
      lev.image.resize(maLevels[i-1].image.size() / 2);
      CVD::halfSample(maLevels[i-1].image, lev.image);
      dDownsampleTime += (ros::WallTime::now()-startTime).toSec();
    }
    
    if(bPushBack)  // also save the previous corners
    {
      lev.vCornersPrev.push_back(std::vector<CVD::ImageRef>());
      lev.vCornersPrev.back().swap(lev.vCorners);  // after this, lev.vCorners is empty
    }
    
    ROS_ASSERT(lev.vCorners.empty());
    lev.vCandidates.clear();
    lev.vScoresAndMaxCorners.clear();
    lev.vFastFrequency = TooN::Zeros;
    lev.nFastThresh = 0;
        
    startTime = ros::WallTime::now();
    
      
    // Use some OpenCV images because they provide an easy way to do dilation, threshold, and bitwise AND
    // We'll just use OpenCV Mat headers to wrap the underlying CVD Image data, so there's very little computational effort involved
    cv::Mat finalMask;
    cv::Mat glareMask;
    
    if(bGlareMasking)  // Generate a mask so that high value areas of the source image are blocked
    {
      cv::Mat img(lev.image.size().y, lev.image.size().x, CV_8U, lev.image.data(), lev.image.row_stride());
      cv::Mat dilated;
      cv::dilate(img, dilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1, -1)), cv::Point(-1, -1), 5, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
      cv::threshold(dilated, glareMask, 245, 255, cv::THRESH_BINARY_INV);
    }
    
    if(lev.mask.totalsize() > 0)  // assume internal mask is set if this is true
    {
      cv::Mat internalMask = cv::Mat(lev.mask.size().y, lev.mask.size().x, CV_8U, lev.mask.data(), lev.mask.row_stride());
        
      if(bGlareMasking)  // Combine the two masks
        cv::bitwise_and(internalMask, glareMask, finalMask);
      else              // The final mask is just the internal mask
        finalMask = internalMask;
    }
    else  // no internal mask set
    { 
      if(bGlareMasking)
        finalMask = glareMask;
      else   // have to generate a blank mask so that all regions are used
        finalMask = cv::Mat(lev.image.size().y, lev.image.size().x, CV_8U, 255);
    }
    
    // Do the opposite of wrapping OpenCV header around CVD Image data. Here we wrap a SubImage around the data of finalMask,
    // which is a cv::Mat object managing its own data
    CVD::SubImage<CVD::byte> mask(finalMask.ptr(), CVD::ImageRef(finalMask.cols, finalMask.rows), finalMask.step1());
    lev.lastMask.copy_from(mask);  // save the mask so it can be shown in the Tracker
    
    dMaskTime += (ros::WallTime::now()-startTime).toSec();
    startTime = ros::WallTime::now();
    
    if(KeyFrame::sbAdaptiveThresh)
    {
      // This version of adaptive thresholding works on the following principle: as the FAST threshold is decreased, the number
      // of detected corners goes up exponentially. Ideally, we want a FAST threshold that retains "good" corners without
      // allowing too many "bad" corners in, ie we want the signal to noise ratio to be high. But how do you define these
      // on real images? Quite simply I looked at many many different real images and played with the FAST threshold in order 
      // to see if there's something in the graph of # of corners vs FAST threshold that would let me select a good cutoff
      // point. What I noticed was that there is often a knee-point where if you lower the threshold further, the number
      // of points increases dramatically, ie there is more noise. Looking at the derivative of this curve I was able to
      // come up with some heuristics that let me select decently good thresholds.
      
      // Perform detection with minimum threshold. Later we'll get each corner's score which will allow us to make a historgram.
      fast_corner_detect_10(lev.image, lev.vCorners, MIN_FAST_THRESH);
      
      std::vector<int> vScores;
      fast_corner_score_10(lev.image, lev.vCorners, MIN_FAST_THRESH, vScores);  // Score them
      
      for(unsigned j=0; j < vScores.size(); ++j)
      {
        // For each score, increment all bins where score >= bin threshold value
        for(int t=MIN_FAST_THRESH; t <= MAX_FAST_THRESH; ++t)
        {
          if(vScores[j] >= t)
            lev.vFastFrequency[t]++;
          
          if(vScores[j] == t)  // reached the end, no point continuing
            break;
        }
      }
      
      // Find the derivative that will indicate the "knee point" we are looking for
      //double targetDeriv = KeyFrame::saThreshDerivs[i];  // use fixed thresholds on derivatives
      double targetDeriv = -1*(lev.image.size().x * lev.image.size().y) / 500.0;  // use thresholds calcualted from image size, could use improvement
      
      // Now we'll look for the threshold that corresponds to the knee point, starting at MIN_FAST_THRESH
      lev.nFastThresh = MIN_FAST_THRESH; 
      for(int t=MIN_FAST_THRESH; t <= MAX_FAST_THRESH; ++t)
      {
        double deriv;
        
        // Compuate the derivative, method depends on whether we are in the middle of the histogram and can use the 
        // midpoint method, or if we are at the end and have to use a forward or backward step
        if(t == MIN_FAST_THRESH)
          deriv = lev.vFastFrequency[t+1] - lev.vFastFrequency[t];
        else if(t == MAX_FAST_THRESH)
          deriv = lev.vFastFrequency[t] - lev.vFastFrequency[t-1];
        else
          deriv = (lev.vFastFrequency[t+1] - lev.vFastFrequency[t-1])/2.0;
          
        lev.nFastThresh = t;
          
        if(deriv > targetDeriv)  // Once we exceed target, we want to stop
          break;
      }
      
      std::vector<CVD::ImageRef> vCornersThreshed;
      for(unsigned j=0; j < vScores.size(); ++j)  // Go through all the corners' scores and keep those above the found threshold
      {
        if(mask[lev.vCorners[j]] < 255) // masked out, skip
          continue;
        
        if(vScores[j] < lev.nFastThresh)
          continue;
          
        vCornersThreshed.push_back(lev.vCorners[j]);
      }
      
      // Save the thresholded corners by swapping them into vCorners
      lev.vCorners.swap(vCornersThreshed);
    }
    else  // old style with fixed extraction thresholds
    {
      // I use a different threshold on each level; this is a bit of a hack
      // whose aim is to balance the different levels' relative feature densities.
    
      if(i == 0)
      {
        fast_corner_detect_10(lev.image, lev.vCorners, 10);
        lev.nFastThresh = 10;
		  }
      if(i == 1)
      {
        fast_corner_detect_10(lev.image, lev.vCorners, 15);
        lev.nFastThresh = 15;
      }
      if(i == 2)
      {
        fast_corner_detect_10(lev.image, lev.vCorners, 15);
        lev.nFastThresh = 15;
      }
      if(i == 3)
      {
        fast_corner_detect_10(lev.image, lev.vCorners, 10);
        lev.nFastThresh = 10;
      }
      
        //do nonmax suppression
  
	  std::vector<CVD::ImageRef> vMaxCornersTemp;
      CVD::fast_nonmax(lev.image, lev.vCorners, lev.nFastThresh, vMaxCornersTemp);
          
      std::vector<int> vMaxScoresTemp;
      CVD::fast_corner_score_10(lev.image, vMaxCornersTemp, lev.nFastThresh, vMaxScoresTemp);  
      
      ROS_ASSERT(vMaxScoresTemp.size() == vMaxCornersTemp.size());
      
      for(unsigned i=0; i < vMaxCornersTemp.size(); ++i)
      {
        if(!lev.image.in_image_with_border(vMaxCornersTemp[i], 10))
          continue;
          
        lev.vScoresAndMaxCorners.push_back(std::make_pair(vMaxScoresTemp[i], vMaxCornersTemp[i]));
      }
      
    }
    
    dFeatureTime += (ros::WallTime::now()-startTime).toSec();
    
    // Generate row look-up-table for the FAST corner points: this speeds up 
    // finding close-by corner points later on.
    unsigned int v=0;
    lev.vCornerRowLUT.clear();
    for(int y=0; y<lev.image.size().y; y++)
    {
      while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
        v++;
      lev.vCornerRowLUT.push_back(v);
    }
    
    ROS_DEBUG_STREAM("MakeKeyFrame_Lite: level: "<<i<<" image prev size: "<<lev.imagePrev.size());
    
  }
  

  
  return std::make_tuple(dDownsampleTime, dMaskTime, dFeatureTime);
}

void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  
  // First, nonmax suppression is done and the remaining points scored. This can be done based
  // on FAST score or Shi-Thomasi score. 
  
  // Next, candidates are generated from these maximal points. This can be done by taking the top
  // X percent of maximal points, or those points that exceed a certain threshold.
  
  // I found best: FAST score, percent
  // Original PTAM: Shi-Thomasi score, threshold
  
  std::string type = KeyFrame::ssCandidateType;    // "shi" or "fast"
  std::string criterion = KeyFrame::ssCandidateCriterion; // "percent" or "thresh"

  // For each level...
  for(int l=0; l<LEVELS; l++)
  {
    Level &lev = maLevels[l];
    
    // .. find those FAST corners which are maximal..

    std::vector<std::pair<double, CVD::ImageRef> > vScoresAndMaxCorners;
    
    if(type == "fast")
    {
      std::vector<CVD::ImageRef> vMaxCornersTemp;
      CVD::fast_nonmax(lev.image, lev.vCorners, lev.nFastThresh, vMaxCornersTemp);
      std::vector<int> vMaxScoresTemp;
      CVD::fast_corner_score_10(lev.image, vMaxCornersTemp, lev.nFastThresh, vMaxScoresTemp);  
      
      ROS_ASSERT(vMaxScoresTemp.size() == vMaxCornersTemp.size());
      
      for(unsigned i=0; i < vMaxCornersTemp.size(); ++i)
      {
        if(!lev.image.in_image_with_border(vMaxCornersTemp[i], 10))
          continue;
          
        lev.vScoresAndMaxCorners.push_back(std::make_pair(vMaxScoresTemp[i], vMaxCornersTemp[i]));
      }
    }
    else // type == "shi"
    {
      std::vector<CVD::ImageRef> vMaxCornersTemp;
      CVD::fast_nonmax(lev.image, lev.vCorners, lev.nFastThresh, vMaxCornersTemp);
      
      for(unsigned i=0; i < vMaxCornersTemp.size(); ++i)
      {
        if(!lev.image.in_image_with_border(vMaxCornersTemp[i], 10))
          continue;
          
        double dSTScore = FindShiTomasiScoreAtPoint(lev.image, 3, vMaxCornersTemp[i]);
        lev.vScoresAndMaxCorners.push_back(std::make_pair(dSTScore, vMaxCornersTemp[i]));
      }
    }
    
    if(criterion=="percent")
    {
      // Sort into descending order using reverse iterators
      std::sort(lev.vScoresAndMaxCorners.rbegin(), lev.vScoresAndMaxCorners.rend());
      
      double dUseTopFraction = KeyFrame::sdCandidateTopFraction;
      int nNumCorners = lev.vScoresAndMaxCorners.size() * dUseTopFraction;
      
      for(unsigned i=0; i < (unsigned)nNumCorners && i < lev.vScoresAndMaxCorners.size() ; ++i)
      {
        Candidate c;
        c.irLevelPos = lev.vScoresAndMaxCorners[i].second;
        c.dSTScore = lev.vScoresAndMaxCorners[i].first;
        lev.vCandidates.push_back(c);
      }
    }
    else  // criterion=="thresh"
    {
      double dThresh = KeyFrame::sdCandidateThresh;
      for(unsigned i=0; i < lev.vScoresAndMaxCorners.size(); ++i)
      {
        if(lev.vScoresAndMaxCorners[i].first > dThresh)
        {
          Candidate c;
          c.irLevelPos = lev.vScoresAndMaxCorners[i].second;
          c.dSTScore = lev.vScoresAndMaxCorners[i].first;
          lev.vCandidates.push_back(c);
        }
      }
    }
    
    ROS_DEBUG_STREAM("MakeKeyFrame_Rest: before imageprev stuff, level "<<l<<" num candidates: "<<lev.vCandidates.size());
    
    if(lev.imagePrev.size() > 0)
    {
      // We have a bunch of candidate points, but we only want to keep the most stable ones
      // Use the circular image and point buffers we have stored in order to do optical flow
      // at the candidate locations. If we can follow patch matches back a few frames, then
      // forward to the current frame and end up less than 2 pixels away from our starting point,
      // then we have a stable point. Otherwise discard it.
      MiniPatch pathPatch;
      
      std::vector<Candidate> vCandidatesPruned;
      
      for(unsigned i=0; i < lev.vCandidates.size(); ++i)
      {
        CVD::ImageRef irCurrCorner = lev.vCandidates[i].irLevelPos;
        pathPatch.SampleFromImage(irCurrCorner, lev.image);
        
        CVD::ImageRef irPrevCorner = irCurrCorner;
        bool bFound = true;
        /*
        // This version goes frame by frame backwards. Will produce more matches since the difference
        // between subsequent frames is small
        for(int j=lev.imagePrev.size()-1; j >= 0; --j)
        {
          bFound = pathPatch.FindPatch(irPrevCorner, lev.imagePrev[j], 10, lev.vCornersPrev[j]);
          if(!bFound)
            break;
          
          pathPatch.SampleFromImage(irPrevCorner, lev.imagePrev[j]);
        }
        */
        
        // This version goes directly to the earliest available frame. Will produce fewer matches
        // since the difference between the current frame and one several frames ago is larger
        // We'll use a search radius of 10 times the number of images we have in the history
        bFound = pathPatch.FindPatch(irPrevCorner, lev.imagePrev[0], lev.imagePrev.size()*10, lev.vCornersPrev[0]);

        if(!bFound)
          continue;
          
        pathPatch.SampleFromImage(irPrevCorner, lev.imagePrev[0]);
        
        CVD::ImageRef irNextCorner = irPrevCorner;
        /*
        // This version goes forward frame by frame. Once again, more matches.
        for(unsigned j=1; j < lev.imagePrev.size(); ++j)
        {
          bFound = pathPatch.FindPatch(irNextCorner, lev.imagePrev[j], 10, lev.vCornersPrev[j]);
          if(!bFound)
            break;
          
          pathPatch.SampleFromImage(irNextCorner, lev.imagePrev[j]);
        }
        
        if(!bFound)
          continue;
          
        bFound = pathPatch.FindPatch(irNextCorner, lev.image, 10, lev.vCorners);
        */
        
        // This version goes forward all the way to the current frame. Once again, fewer matches.
        bFound = pathPatch.FindPatch(irNextCorner, lev.image, lev.imagePrev.size()*10, lev.vCorners);  
        
        if(!bFound)
          continue;
          
        // Difference in location needs to be less than 2 pixels
        if((irNextCorner - irCurrCorner).mag_squared() > 2)
          continue;
          
        vCandidatesPruned.push_back(lev.vCandidates[i]);
      }
      
      lev.vCandidates.swap(vCandidatesPruned);
    }
    
    ROS_DEBUG_STREAM("MakeKeyFrame_Rest: after imageprev stuff, level "<<l<<" num candidates: "<<lev.vCandidates.size());
    
  } // end loop over levels
  
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  MakeSBI();
}

void KeyFrame::MakeSBI()
{
  if(!mpSBI)
    mpSBI = new SmallBlurryImage(*this);  
  // Relocaliser also wants the jacobians..
  mpSBI->MakeJacs();
}

// Calculates the distance of map points visible in a keyframe and their weights
// based on in/out-lier counts
void KeyFrame::GetPointDepthsAndWeights(std::vector<std::pair<double, double> >& vDepthsAndWeights)
{
  vDepthsAndWeights.clear();
  vDepthsAndWeights.reserve(mmpMeasurements.size());
  
  for(MeasPtrMap::iterator it = mmpMeasurements.begin(); it!=mmpMeasurements.end(); it++)
  {
    MapPoint &point = *(it->first);
    
    if(point.mbBad)
      continue;
      
    Vector<3> v3CamPos = mse3CamFromWorld * point.mv3WorldPos;
    
    ROS_ASSERT(point.mnMEstimatorInlierCount > 0); // should be set to 1 when point is created
    
    // The weight is simply the ratio of inlier to all observations
    double weight = point.mnMEstimatorInlierCount / (double)(point.mnMEstimatorInlierCount + point.mnMEstimatorOutlierCount);
    vDepthsAndWeights.push_back(std::make_pair(norm(v3CamPos), weight));
  }
  
}

// Gets the depths and weights of the visible map points and calls the robust scene depth calculator on them
void KeyFrame::RefreshSceneDepthRobust()
{
  std::vector<std::pair<double, double> > vDepthsAndWeights;
  GetPointDepthsAndWeights(vDepthsAndWeights);
  RefreshSceneDepthRobust(vDepthsAndWeights);
  
}

// Calculates the mean scene depth and scene depth variance based on the supplied depths and weights
// This version uses a robust M-estimator to calculate a weight for each point, which is combined
// with the supplied weight in the weighted mean estimation. This prevents large spurrious
// depths (coming from points that should be erased soon anyway) from completely corrupting the mean estimate
void KeyFrame::RefreshSceneDepthRobust(std::vector<std::pair<double, double> >& vDepthsAndWeights)
{
  if(vDepthsAndWeights.size() <= 3)  // This KF is really bad, should remove it
  {
    ROS_ERROR_STREAM("vDepthsAndWeights size is "<<vDepthsAndWeights.size()<<", something is probably wrong!");
    return;
  }
  
  // We'll use deviation from the median depth to compute weights
  
  std::sort(vDepthsAndWeights.begin(), vDepthsAndWeights.end());
  double dMedianDepth = vDepthsAndWeights[vDepthsAndWeights.size() / 2].first;
  
  std::vector<double> vDistanceSquaredFromMedian;
  vDistanceSquaredFromMedian.reserve(vDepthsAndWeights.size());
  
  for(unsigned i=0; i < vDepthsAndWeights.size(); ++i)
  {
    double d = vDepthsAndWeights[i].first - dMedianDepth;
    vDistanceSquaredFromMedian.push_back(d*d);    
  }
    
  // FindSigmaSquared sorts the input and we don't want to recompute distance squared, so make a copy here
  std::vector<double> vDistanceSquaredFromMedian_disposable = vDistanceSquaredFromMedian;
  double dSigmaSquared =  Huber::FindSigmaSquared(vDistanceSquaredFromMedian_disposable); 
  
  // Don't be too harsh
  if(dSigmaSquared < 0.4)
    dSigmaSquared = 0.4;
  
  double dSumWeightedDepth = 0.0;
  double dSumWeightedDepthSquared = 0.0;
  double dSumWeights = 0;
  
  for(unsigned i=0; i < vDistanceSquaredFromMedian.size(); ++i)
  {
    double dist_squared = vDistanceSquaredFromMedian[i];
    
    double tukey_weight = Huber::SquareRootWeight(dist_squared, dSigmaSquared);
    double combined_weight = vDepthsAndWeights[i].second * tukey_weight;
    double depth = vDepthsAndWeights[i].first;
    
    dSumWeightedDepth += combined_weight * depth;
    dSumWeightedDepthSquared += combined_weight * depth * depth;
    dSumWeights += combined_weight;
  }
  
  mdSceneDepthMean = dSumWeightedDepth / dSumWeights;
  mdSceneDepthSigma = sqrt((dSumWeightedDepthSquared / dSumWeights) - (mdSceneDepthMean) * (mdSceneDepthMean));
   
  if(!std::isfinite(mdSceneDepthMean))
  {
    ROS_FATAL_STREAM("dSigmaSquared: "<<dSigmaSquared);
    ROS_FATAL_STREAM("dSumWeightedDepth: "<<dSumWeightedDepth);
    ROS_FATAL_STREAM("dSumWeights: "<<dSumWeights);
    for(unsigned i=0; i < vDepthsAndWeights.size(); ++i)
      ROS_FATAL_STREAM("depth: "<<vDepthsAndWeights[i].first<<" weight: "<<vDepthsAndWeights[i].second);
      
    ROS_BREAK();
  }
}

// Gets the depths and weights of the visible map points and calls the simple scene depth calculator on them
void KeyFrame::RefreshSceneDepthMean()
{
  std::vector<std::pair<double, double> > vDepthsAndWeights;
  GetPointDepthsAndWeights(vDepthsAndWeights);
  
  RefreshSceneDepthMean(vDepthsAndWeights);
}

// Calculates the mean scene depth and scene depth variance based on the supplied depths and weights
// This version uses a straightforward weighted mean and variance algorithm, which means that a large outlier
// could, if its given weight is non-zero, corrupt the mean estimate. Should use the robust version above
// unless performance is suffering i.e. in the Tracker where this function is called at framerate
void KeyFrame::RefreshSceneDepthMean(std::vector<std::pair<double, double> >& vDepthsAndWeights)
{
  if(vDepthsAndWeights.size() == 0)
  {
    ROS_ERROR("vDepthsAndWeights size is 0, something is probably wrong!");
    return;
  }
  
  double dSumWeightedDepth = 0.0;
  double dSumWeightedDepthSquared = 0.0;
  double dSumWeights = 0;
  
  for(unsigned i=0; i < vDepthsAndWeights.size(); ++i)
  {
    double depth = vDepthsAndWeights[i].first;
    double weight = vDepthsAndWeights[i].second;
    
    dSumWeightedDepth += weight * depth;
    dSumWeightedDepthSquared += weight * depth * depth;
    dSumWeights += weight;
  }
  
  mdSceneDepthMean = dSumWeightedDepth / dSumWeights;
  mdSceneDepthSigma = sqrt((dSumWeightedDepthSquared / dSumWeights) - (mdSceneDepthMean) * (mdSceneDepthMean));
   
  if(!std::isfinite(mdSceneDepthMean))
  {
    ROS_FATAL_STREAM("dSumWeightedDepth: "<<dSumWeightedDepth);
    ROS_FATAL_STREAM("dSumWeights: "<<dSumWeights);
    for(unsigned i=0; i < vDepthsAndWeights.size(); ++i)
      ROS_FATAL_STREAM("depth: "<<vDepthsAndWeights[i].first<<" weight: "<<vDepthsAndWeights[i].second);
      
    ROS_BREAK();
  }
}

// Euclidean distance to the other keyframe
double KeyFrame::LinearDist(KeyFrame &other)
{
  Vector<3> v3KF1_CamPos = mse3CamFromWorld.inverse().get_translation();
  Vector<3> v3KF2_CamPos = other.mse3CamFromWorld.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dist = sqrt(v3Diff * v3Diff);
  return dist;
}

// A distance metric that combines the Euclidean distance to the other keyframe with the Euclidean distance between
// the points created from the mean scene depths (in the direction the keyframes are facing). 
// This is a pretty crude metric with no real rationale behind it other than trying to capture the fact that the 
// distance between keyframes should be low only if both the Euclidean distance between them is low, AND they both point
// in the same direction. For example, two keyframes back to back would have zero Euclidean distance but since they
// don't look at the same scene, it would be incorrect to say they are distance zero apart. 
//
// An alternative to this implementation would be to describe distance by the number of map points that both
// keyframes share compared to the total they observe, or something similar.
double KeyFrame::Distance(KeyFrame &other)
{
  SE3<> se3KF1_inv = mse3CamFromWorld.inverse();
  SE3<> se3KF2_inv = other.mse3CamFromWorld.inverse();
  
  Vector<3> v3KF1_CamPos = se3KF1_inv.get_translation();
  Vector<3> v3KF2_CamPos = se3KF2_inv.get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;     // Vector between keyframe bases
  
  Vector<3> v3KF1_MeanInCam = Zeros; v3KF1_MeanInCam[2] = mdSceneDepthMean;   // mean scene depth point in camera frame
  Vector<3> v3KF2_MeanInCam = Zeros; v3KF2_MeanInCam[2] = other.mdSceneDepthMean; // mean scene depth point in camera frame
  
  // Convert mean depth points to world frame
  Vector<3> v3KF1_MeanInWorld = se3KF1_inv * v3KF1_MeanInCam; 
  Vector<3> v3KF2_MeanInWorld = se3KF2_inv * v3KF2_MeanInCam;
  Vector<3> v3MeanDiff = v3KF2_MeanInWorld - v3KF1_MeanInWorld;  // Vector between mean depth points
   
  // The final distance is a weighted combination of these two Euclidean distances
  double dDist = sqrt(v3Diff * v3Diff) + sqrt(v3MeanDiff * v3MeanDiff) * KeyFrame::sdDistanceMeanDiffFraction;
  if(!std::isfinite(dDist) || dDist > 1e10)
  {
    ROS_FATAL_STREAM("dDist: "<<dDist);
    ROS_FATAL_STREAM("v3Diff: "<<v3Diff);
    ROS_FATAL_STREAM("v3MeanDiff: "<<v3MeanDiff);
    ROS_FATAL_STREAM("frac: "<<KeyFrame::sdDistanceMeanDiffFraction);
    ROS_FATAL_STREAM("mdSceneDepthMean: "<<mdSceneDepthMean);
    ROS_FATAL_STREAM("other.mdSceneDepthMean: "<<other.mdSceneDepthMean);
    
    ROS_BREAK();
  }
  
  return dDist;
}

// Erase the measurement object corresponding to its argument and remove it from the map of measurements
void KeyFrame::EraseMeasurementOfPoint(MapPoint* pPoint)
{
  boost::mutex::scoped_lock lock(mMeasMutex);
  
  MeasPtrMap::iterator it = mmpMeasurements.find(pPoint);
  ROS_ASSERT(it != mmpMeasurements.end());
  
  delete it->second;
  mmpMeasurements.erase(it);
}

// Erase, from all points measured, the pointer back to this KeyFrame
void KeyFrame::EraseBackLinksFromPoints()
{
  for(MeasPtrMap::iterator it = mmpMeasurements.begin(); it != mmpMeasurements.end(); ++it)
  {
    MapPoint& point = *(it->first);
    int nErased = point.mMMData.spMeasurementKFs.erase(this);
    ROS_ASSERT(nErased);
    // If this KeyFrame is the point's source, then mark it bad because it's about to
    // lose it. Alternatively, could transfer the role of source KF to another one in 
    // spMeasurementKFs, but that would need to be implemented and tested
    if(point.mpPatchSourceKF == this)
    {
      point.mbBad = true;
    }
  }
  
}

KeyFrame* KeyFrame::CopyKeyFramePartial(MultiKeyFrame* sourceMKF, std::string name)
{
	KeyFrame* returnKF = new KeyFrame(sourceMKF, name); //construct a new keyframe
	//copy stuff over
	returnKF->sdDistanceMeanDiffFraction=sdDistanceMeanDiffFraction;  ///< fraction of distance between mean scene depth points that is used in overall distance computation
	returnKF->ssCandidateType=ssCandidateType; ///< decide scoring type ("fast, "shi")
	returnKF->ssCandidateCriterion=ssCandidateCriterion;  ///< decide scoring criterion ("percent", "thresh")
	returnKF->sdCandidateThresh=sdCandidateThresh; ///< when using "thresh" criterion
	returnKF->sdCandidateTopFraction=sdCandidateTopFraction; ///< when using "percent" criterion
	returnKF->sbAdaptiveThresh=sbAdaptiveThresh;  ///< should we use an adaptive computation of the feature detection threshold?
	returnKF->mse3CamFromBase=mse3CamFromBase;   ///< The current pose in a base frame, which is the pose of the parent MultiKeyFrame
	returnKF->mse3CamFromWorld=mse3CamFromWorld;  ///< The current pose in the world frame, a product of mse3CamFromBase and the parent's mse3BaseFromWorld
  returnKF->mdSceneDepthMean=mdSceneDepthMean;  ///< The mean z-axis value of all the points visible from this keyframe
	returnKF->mdSceneDepthSigma=mdSceneDepthSigma; ///< The variance of the z-axis values of the points visible from this keyframe
	returnKF->mbActive=mbActive; 
	
		
	//deep copy the Levels info
	
	for(int i=0; i<LEVELS; i++)
	{
		
		if( maLevels[i].lastMask.size() != CVD::ImageRef(0,0)) //not empty
		{
			returnKF->maLevels[i].lastMask.resize( maLevels[i].lastMask.size());
			CVD::copy(maLevels[i].lastMask, returnKF->maLevels[i].lastMask);
		}
		
		
		if(maLevels[i].mask.size() != CVD::ImageRef(0,0))
		{
			returnKF->maLevels[i].mask.resize( maLevels[i].mask.size());
			CVD::copy(maLevels[i].mask, returnKF->maLevels[i].mask);
		}
		
		if(maLevels[i].image.size() != CVD::ImageRef(0,0))
		{
			returnKF->maLevels[i].image.resize( maLevels[i].image.size());
			CVD::copy(maLevels[i].image, returnKF->maLevels[i].image);
		}
		
		returnKF->maLevels[i].vCorners = maLevels[i].vCorners;  
		returnKF->maLevels[i].vCornerRowLUT = maLevels[i].vCornerRowLUT;   
		returnKF->maLevels[i].vCandidates = maLevels[i].vCandidates;      
		returnKF->maLevels[i].vScoresAndMaxCorners = maLevels[i].vScoresAndMaxCorners;  
		
		returnKF->maLevels[i].vFastFrequency = maLevels[i].vFastFrequency;    
		returnKF->maLevels[i].nFastThresh = maLevels[i].nFastThresh;               
 
		returnKF->maLevels[i].imagePrev = maLevels[i].imagePrev;  
		returnKF->maLevels[i].vCornersPrev = maLevels[i].vCornersPrev;  
		
		returnKF->maLevels[i].snNumPrev = maLevels[i].snNumPrev;  
		            
	}
	
	
	return returnKF;
	
}

//------------------------------------------ MultiKeyFrame  ------------------------------------------------------------------------

MultiKeyFrame::MultiKeyFrame()
{
  mbFixed = false;
  mdTotalDepthMean = MAX_DEPTH;
  mbBad = false;
  mbDeleted = false;
  mnUsing = 0;
  isBufferMKF = false; //initialize to NOT a bufferkeyframe
}

MultiKeyFrame::~MultiKeyFrame()
{
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {
    delete it->second;  // delete the keyframe
  }
  
  mmpKeyFrames.clear();
}

// Call ClearMeasurements on all owned KeyFrames
void MultiKeyFrame::ClearMeasurements()
{
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {
    it->second->ClearMeasurements();
  }
}

// Returns the number of measurements of all owned KeyFrames
int MultiKeyFrame::NumMeasurements()
{
  int nNumMeas = 0;
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {      
    nNumMeas += it->second->mmpMeasurements.size();
  }
  
  return nNumMeas;
}

bool MultiKeyFrame::NoImages()
{
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {      
    if(!it->second->NoImage())
      return false;
  }
  
  return true;
}

void MultiKeyFrame::RemoveImages()
{
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {      
    it->second->RemoveImage();
  }
}

// Call EraseBackLinksFromPoints on all owned KeyFrames
void MultiKeyFrame::EraseBackLinksFromPoints()
{
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {
    it->second->EraseBackLinksFromPoints();
  }
}

// Call RefreshSceneDepthRobust on all owned KeyFrames
void MultiKeyFrame::RefreshSceneDepthRobust()
{
  double dSumDepth = 0.0;
  int nNum = 0;
  
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);  
    if(!kf.mbActive)
      continue;
          
    kf.RefreshSceneDepthRobust();
    dSumDepth += kf.mdSceneDepthMean;
    nNum++;
  }
  
  ROS_ASSERT(nNum > 0);
  mdTotalDepthMean = dSumDepth / nNum;
}

// Call RefreshSceneDepthMean on all owned KeyFrames and recomputes total depth mean
void MultiKeyFrame::RefreshSceneDepthMean()
{
  double dSumDepth = 0.0;
  int nNum = 0;
  
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); it++)
  {
    KeyFrame& kf = *(it->second);
    if(!kf.mbActive)
      continue;
    
    kf.RefreshSceneDepthMean();
    dSumDepth += kf.mdSceneDepthMean;
    nNum++;
  }
  
  ROS_ASSERT(nNum > 0);
  mdTotalDepthMean = dSumDepth / nNum;
}

// Calculates the Euclidean linear distance between the current MultiKeyFrame and the argument
double MultiKeyFrame::LinearDist(MultiKeyFrame &other)
{
  Vector<3> v3KF1_BasePos = mse3BaseFromWorld.inverse().get_translation();
  Vector<3> v3KF2_BasePos = other.mse3BaseFromWorld.inverse().get_translation();
  Vector<3> v3Diff = v3KF1_BasePos - v3KF2_BasePos;
  double dist = sqrt(v3Diff * v3Diff);
  return dist;
}

// Calculates the distance between this MultiKeyFrame and the argument as the minimum distance between any of their KeyFrames
double MultiKeyFrame::Distance(MultiKeyFrame &other)
{
  double dMinDist = std::numeric_limits<double>::max();
  
  for(KeyFramePtrMap::iterator kf1_it = mmpKeyFrames.begin(); kf1_it != mmpKeyFrames.end(); ++kf1_it)
  {
    KeyFrame& kf1 = *(kf1_it->second);
    if(!kf1.mbActive)
      continue;
    
    for(KeyFramePtrMap::iterator kf2_it = other.mmpKeyFrames.begin(); kf2_it != other.mmpKeyFrames.end(); ++kf2_it)
    {
      KeyFrame& kf2 = *(kf2_it->second);
      if(!kf2.mbActive)
        continue;
      
      double dDist = kf1.Distance(kf2);
      if(dDist < dMinDist)
        dMinDist = dDist;
      
    }
  }
  
  return dMinDist;
}

//partially copies MKF
MultiKeyFrame* MultiKeyFrame::CopyMultiKeyFramePartial()
{
	MultiKeyFrame* returnMKF = new MultiKeyFrame();
	
	//copy stuff over
	
  returnMKF->mse3BaseFromWorld = mse3BaseFromWorld;  ///< The current pose in the world reference frame
  returnMKF->mbFixed=mbFixed; ///< Is the pose fixed? Generally only true for the first MultiKeyFrame added to the map
  returnMKF->mbBad=mbBad;  ///< Is it a dud? In that case it'll be moved to the trash soon.
  returnMKF->mbDeleted=mbDeleted; ///< Similar to mbBad, but used only in client/server code to allow immediate deletion of received MKF
  //returnMKF->mnUsing=mnUsing;  
  
  //returnMKF->mmpKeyFrames=mmpKeyFrames;  ///< %Map of camera names to KeyFrame pointers
  
  //copy each keyframe individually
  for(KeyFramePtrMap::iterator it = mmpKeyFrames.begin(); it != mmpKeyFrames.end(); ++it)
  {    
	  std::string cName = (it->first);
	  KeyFrame& kf = *(it->second); 
	  returnMKF->mmpKeyFrames.insert(std::make_pair(cName, kf.CopyKeyFramePartial(returnMKF,cName)));
  }
  
  returnMKF->mdTotalDepthMean=mdTotalDepthMean;  ///< The mean of all owned KeyFrames' mdSceneDepthMean values
  returnMKF->mnID=mnID;      ///< Used for identifying MultiKeyFrame when writing map to file   
  returnMKF->isBufferMKF = isBufferMKF;
  
  return returnMKF;     
	
}

// ------------------------------------------- Other stuff -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
    {
      if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
      else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
      else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
      else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 1.0);
      else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
    }
  }
};

static LevelHelpersFiller foo;
