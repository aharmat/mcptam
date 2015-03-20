#ifndef __FLIP_AXIS_ACTION_H
#define __FLIP_AXIS_ACTION_H

#include <mcptam/EditAction.h>
#include <TooN/se3.h>
#include <vector>

class Map;
class MapPoint;

class FlipAxisAction : public EditAction
{
public:

  FlipAxisAction(Map* pMap, int nDim);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();
  
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);

  Map* mpMap;
  
  TooN::SE3<> mse3NewFromOld;
  
};

#endif
