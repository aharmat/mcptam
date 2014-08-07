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

#include <mcptam/Relocaliser.h>
#include <mcptam/SmallBlurryImage.h>
#include <mcptam/Map.h>
#include <mcptam/KeyFrame.h>
#include <cvd/utility.h>

using namespace TooN;

// Static member
double Relocaliser::sdRecoveryMaxScore = 1e5;//9e6;

Relocaliser::Relocaliser(Map &map, TaylorCamera camera, std::string camName)
: mMap(map)
{
  mmCameraModels.insert(std::pair<std::string,TaylorCamera>(camName, camera));
}

Relocaliser::Relocaliser(Map &map, TaylorCameraMap cameras)
  : mMap(map)
  , mmCameraModels(cameras)
{
  
};

SE3<> Relocaliser::BestPose()
{
  return mse3Best;
}

bool Relocaliser::AttemptRecovery(KeyFrame &kfCurrent)
{
  // Ensure the incoming frame has a SmallBlurryImage attached
  if(!kfCurrent.mpSBI)
    kfCurrent.mpSBI = new SmallBlurryImage(kfCurrent);
  else
    kfCurrent.mpSBI->MakeFromKF(kfCurrent);
  
  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(kfCurrent);
  
  if(mpBestKF == NULL)  // nothing found so bail out
    return false;

  // And estimate a camera rotation from a 3DOF image alignment
  std::pair<SE2<>, double> result_pair = kfCurrent.mpSBI->IteratePosRelToTarget(*(mpBestKF->mpSBI), 6);
  mse2 = result_pair.first;
  double dScore =result_pair.second;
  
  SE3<> se3KeyFramePos = mpBestKF->mse3CamFromWorld;
  
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mmCameraModels[kfCurrent.mCamName], mmCameraModels[mpBestKF->mCamName]) * se3KeyFramePos;
  
  if(dScore < Relocaliser::sdRecoveryMaxScore)
    return true;
  else 
    return false;
};

// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void Relocaliser::ScoreKFs(KeyFrame &kfCurrent)
{
  mdBestScore = std::numeric_limits<double>::max();
  mpBestKF = NULL;
  
  int nNumWithoutSBI = 0;
  int nNumTotal = 0;
  
  for(MultiKeyFramePtrList::iterator it = mMap.mlpMultiKeyFrames.begin(); it != mMap.mlpMultiKeyFrames.end(); ++it)
  {
    MultiKeyFrame& mkf = *(*it);
    for(KeyFramePtrMap::iterator jit = mkf.mmpKeyFrames.begin(); jit != mkf.mmpKeyFrames.end(); ++jit)
    {
      KeyFrame& kf = *(jit->second);
      if(kfCurrent.mCamName != kf.mCamName)  // only look at same camera
        continue;
        
      nNumTotal++;
        
      if(!kf.mpSBI)
      {
        ROS_WARN("KF doesn't have small blurry image! Skipping ...");
        nNumWithoutSBI++;
        continue;
      }
      
      double dSSD = kfCurrent.mpSBI->ZMSSD(*(kf.mpSBI));
      if(dSSD < mdBestScore)
      {
        mdBestScore = dSSD;
        mpBestKF = &kf;
      }
    }
  }
  
  ROS_INFO_STREAM("Relocalizer looked at "<<nNumTotal<<" SBIs. "<<nNumWithoutSBI<<" KeyFrames didn't have SBIs.");
}

