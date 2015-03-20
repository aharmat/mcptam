#ifndef __FIT_PLANE_ACTION_H
#define __FIT_PLANE_ACTION_H

#include <mcptam/EditAction.h>
#include <TooN/se3.h>
#include <vector>

class Map;
class MapPoint;

class FitPlaneAction : public EditAction
{
public:

  FitPlaneAction(Map* pMap, std::vector<MapPoint*> vpPoints, int nDim);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();
  
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);
  TooN::SE3<> CalcPlaneAligner(int nDim);

  Map* mpMap;
  std::vector<MapPoint*> mvpPoints;
  
  TooN::SE3<> mse3NewFromOld;
  
};

#endif
