#include <mcptam/FlipZAction.h>
#include <mcptam/Map.h>
#include <mcptam/MapPoint.h>
#include <TooN/TooN.h>
#include <TooN/SymEigen.h>

FlipZAction::FlipZAction(Map* pMap)
: mpMap(pMap)
{
  mse3NewFromOld = TooN::SE3<>::exp(TooN::makeVector(0,0,0,M_PI,0,0));
}

void FlipZAction::DoInternal()
{
  ApplyGlobalTransformationToMap(mse3NewFromOld);
}

void FlipZAction::UndoInternal()
{
  ApplyGlobalTransformationToMap(mse3NewFromOld.inverse());
}
  
void FlipZAction::ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld)
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
