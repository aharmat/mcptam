#include <mcptam/FitGroundPlaneAction.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <TooN/TooN.h>
#include <TooN/SymEigen.h>

FitGroundPlaneAction::FitGroundPlaneAction(Map* pMap, std::vector<MapPoint*> vpPoints)
: mpMap(pMap)
, mvpPoints(vpPoints)
{
  mse3NewFromOld = CalcPlaneAligner();
}

void FitGroundPlaneAction::DoInternal()
{
  ApplyGlobalTransformationToMap(mse3NewFromOld);
}

void FitGroundPlaneAction::UndoInternal()
{
  ApplyGlobalTransformationToMap(mse3NewFromOld.inverse());
}
  
void FitGroundPlaneAction::ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld)
{
  for(MultiKeyFramePtrList::iterator mkf_it = mpMap->mlpMultiKeyFrames.begin(); mkf_it != mpMap->mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    mkf.mse3BaseFromWorld = mkf.mse3BaseFromWorld * se3NewFromOld.inverse();
    for(KeyFramePtrMap::iterator kf_it = mkf.mmpKeyFrames.begin(); kf_it != mkf.mmpKeyFrames.end(); ++kf_it)
    {
      KeyFrame& kf = *(kf_it->second);      
      kf.mse3CamFromWorld = kf.mse3CamFromBase * mkf.mse3BaseFromWorld;  // CHECK!! GOOD
    }
  }
  
  for(MapPointPtrList::iterator point_it = mpMap->mlpPoints.begin(); point_it != mpMap->mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    point.mv3WorldPos = se3NewFromOld * point.mv3WorldPos;
    point.RefreshPixelVectors();
  }
}

TooN::SE3<> FitGroundPlaneAction::CalcPlaneAligner()
{
  int nRansacs = 500;
  TooN::Vector<3> v3BestMean = TooN::Zeros;
  TooN::Vector<3> v3BestNormal = TooN::Zeros;
  double dBestDistSquared = 9999999999999999.9;
    
  unsigned int nPoints = mvpPoints.size();
  if(nPoints < 10)
  {
    ROS_INFO("MapEditor: CalcPlane: too few points to calc plane.");
    return TooN::SE3<>();
  }
  
  for(int i=0; i<nRansacs; i++)
  {
    int nA = rand()%nPoints;
    int nB = nA;
    int nC = nA;
    while(nB == nA)
      nB = rand()%nPoints;
      
    while(nC == nA || nC==nB)
      nC = rand()%nPoints;
    
    TooN::Vector<3> v3Mean = 0.33333333 * (mvpPoints[nA]->mv3WorldPos + 
             mvpPoints[nB]->mv3WorldPos + 
             mvpPoints[nC]->mv3WorldPos);
    
    TooN::Vector<3> v3CA = mvpPoints[nC]->mv3WorldPos  - mvpPoints[nA]->mv3WorldPos;
    TooN::Vector<3> v3BA = mvpPoints[nB]->mv3WorldPos  - mvpPoints[nA]->mv3WorldPos;
    TooN::Vector<3> v3Normal = v3CA ^ v3BA;
    
    if(v3Normal * v3Normal  == 0)
      continue;
      
    TooN::normalize(v3Normal);
    
    double dSumError = 0.0;
    for(unsigned int i=0; i<nPoints; i++)
    {
      TooN::Vector<3> v3Diff = mvpPoints[i]->mv3WorldPos - v3Mean;
      double dDistSq = v3Diff * v3Diff;
      
      if(dDistSq == 0.0)
        continue;
        
      double dNormDist = fabs(v3Diff * v3Normal);
      
      if(dNormDist > 0.05)
        dNormDist = 0.05;
        
      dSumError += dNormDist;
    }
    
    if(dSumError < dBestDistSquared)
    {
      dBestDistSquared = dSumError;
      v3BestMean = v3Mean;
      v3BestNormal = v3Normal;
    }
  }
  
  // Done the ransacs, now collect the supposed inlier set
  std::vector<TooN::Vector<3> > vInliers;
  for(unsigned int i=0; i<nPoints; i++)
  {
    TooN::Vector<3> v3Diff = mvpPoints[i]->mv3WorldPos - v3BestMean;
    double dDistSq = v3Diff * v3Diff;
    if(dDistSq == 0.0)
      continue;
      
    double dNormDist = fabs(v3Diff * v3BestNormal);
    if(dNormDist < 0.05)
      vInliers.push_back(mvpPoints[i]->mv3WorldPos);
  }
  
  // With these inliers, calculate mean and cov
  TooN::Vector<3> v3MeanOfInliers = TooN::Zeros;
  for(unsigned int i=0; i<vInliers.size(); i++)
    v3MeanOfInliers+=vInliers[i];
    
  v3MeanOfInliers *= (1.0 / vInliers.size());
  
  TooN::Matrix<3> m3Cov = TooN::Zeros;
  for(unsigned int i=0; i<vInliers.size(); i++)
  {
    TooN::Vector<3> v3Diff = vInliers[i] - v3MeanOfInliers;
    m3Cov += v3Diff.as_col() * v3Diff.as_row();
  }
  
  // Find the principal component with the minimal variance: this is the plane normal
  TooN::SymEigen<3> sym(m3Cov);
  TooN::Vector<3> v3Normal = sym.get_evectors()[0];
  
  // If mean of inliers Z is negative, we want positive plane normal to put camera above plane
  // If mean of inliers Z is positive, we want negative plane normal to put camera above plane
  if(v3MeanOfInliers[2] < 0 && v3Normal[2] < 0)
    v3Normal *= -1.0;
  else if(v3MeanOfInliers[2] > 0 && v3Normal[2] > 0)
    v3Normal *= -1.0;
  
  TooN::Matrix<3> m3Rot = TooN::Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  TooN::normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];
  
  TooN::SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  TooN::Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;
  
  return se3Aligner;
}
