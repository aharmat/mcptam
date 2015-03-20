#include <mcptam/MoveAxisAction.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <TooN/TooN.h>
#include <TooN/SymEigen.h>

MoveAxisAction::MoveAxisAction(Map* pMap, int nDim, double dDist)
: mpMap(pMap)
{
  TooN::Vector<6> v6Transform = TooN::Zeros;
  v6Transform[nDim] = dDist; 
  
  mse3NewFromOld = TooN::SE3<>::exp(v6Transform);
}

void MoveAxisAction::DoInternal()
{
  //std::cout<<"MoveAxisAction::DoInternal"<<std::endl;
  //std::cout<<"Applying transformation: "<<std::endl<<mse3NewFromOld<<std::endl;
  ApplyGlobalTransformationToMap(mse3NewFromOld);
}

void MoveAxisAction::UndoInternal()
{
  //std::cout<<"MoveAxisAction::UndoInternal"<<std::endl;
  //std::cout<<"Applying transformation: "<<std::endl<<mse3NewFromOld.inverse()<<std::endl;
  ApplyGlobalTransformationToMap(mse3NewFromOld.inverse());
}
  
void MoveAxisAction::ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld)
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
