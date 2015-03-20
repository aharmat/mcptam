#ifndef __MOVE_AXIS_ACTION_H
#define __MOVE_AXIS_ACTION_H

#include <mcptam/EditAction.h>
#include <TooN/se3.h>
#include <vector>

class Map;
class MapPoint;

class MoveAxisAction : public EditAction
{
public:

  MoveAxisAction(Map* pMap, int nDim, double dDist);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();
  
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);

  Map* mpMap;
  
  TooN::SE3<> mse3NewFromOld;
  
};

#endif
