#include <mcptam/DeletePointsAction.h>
#include <mcptam/MapPoint.h>

DeletePointsAction::DeletePointsAction(std::vector<MapPoint*> vpPoints)
: mvpPoints(vpPoints)
{
  
}

void DeletePointsAction::DoInternal()
{
  for(unsigned i=0; i < mvpPoints.size(); ++i)
  {
    mvpPoints[i]->mbDeleted = true;
    mvpPoints[i]->mbSelected = false;
  }
}

void DeletePointsAction::UndoInternal()
{
  for(unsigned i=0; i < mvpPoints.size(); ++i)
  {
    mvpPoints[i]->mbDeleted = false;
  }
}
  
