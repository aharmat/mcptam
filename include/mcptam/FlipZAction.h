#ifndef __FLIP_Z_ACTION_H
#define __FLIP_Z_ACTION_H

#include <mcptam/EditAction.h>
#include <TooN/se3.h>
#include <vector>

class Map;
class MapPoint;

class FlipZAction : public EditAction
{
public:

  FlipZAction(Map* pMap);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();
  
  void ApplyGlobalTransformationToMap(TooN::SE3<> se3NewFromOld);

  Map* mpMap;
  
  TooN::SE3<> mse3NewFromOld;
  
};

#endif
