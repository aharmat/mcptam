#include <mcptam/FlipAxisAction.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <TooN/TooN.h>
#include <TooN/SymEigen.h>

FlipAxisAction::FlipAxisAction(Map* pMap, int nDim)
: mpMap(pMap)
{
  // Flipping happens about either of other axes
  int nOtherDim = nDim + 1;
  if(nOtherDim > 2)
    nOtherDim = 0;
    
  TooN::Vector<6> v6Transform = TooN::Zeros;
  v6Transform[3+nOtherDim] = M_PI; 
  
  mse3NewFromOld = TooN::SE3<>::exp(v6Transform);
}

void FlipAxisAction::DoInternal()
{
  //std::cout<<"FlipAxisAction::DoInternal"<<std::endl;
  //std::cout<<"Applying transformation: "<<std::endl<<mse3NewFromOld<<std::endl;
  ApplyGlobalTransformationToMap(mse3NewFromOld);
}

void FlipAxisAction::UndoInternal()
{
  //std::cout<<"FlipAxisAction::UndoInternal"<<std::endl;
  //std::cout<<"Applying transformation: "<<std::endl<<mse3NewFromOld.inverse()<<std::endl;
  ApplyGlobalTransformationToMap(mse3NewFromOld.inverse());
}
  
void FlipAxisAction::ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld)
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
