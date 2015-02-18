#ifndef __FIT_GROUND_PLANE_ACTION_H
#define __FIT_GROUND_PLANE_ACTION_H

#include <mcptam/EditAction.h>
#include <TooN/se3.h>
#include <vector>

class Map;
class MapPoint;

class FitGroundPlaneAction : public EditAction
{
public:

  FitGroundPlaneAction(Map* pMap, std::vector<MapPoint*> vpPoints);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();
  
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);
  TooN::SE3<> CalcPlaneAligner();

  Map* mpMap;
  std::vector<MapPoint*> mvpPoints;
  
  TooN::SE3<> mse3NewFromOld;
  
};

#endif
