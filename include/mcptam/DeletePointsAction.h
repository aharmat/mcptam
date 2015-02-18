#ifndef __DELETE_POINTS_ACTION_H
#define __DELETE_POINTS_ACTION_H

#include <mcptam/EditAction.h>
#include <vector>

class MapPoint;

class DeletePointsAction : public EditAction
{
public:

  DeletePointsAction(std::vector<MapPoint*> vpPoints);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();

  std::vector<MapPoint*> mvpPoints;
  
};

#endif
