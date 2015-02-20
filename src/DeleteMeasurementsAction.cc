#include <mcptam/DeleteMeasurementsAction.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>

DeleteMeasurementsAction::DeleteMeasurementsAction(MeasPtrMap mpMeas)
: mmpMeasurements(mpMeas)
{
  
}

void DeleteMeasurementsAction::DoInternal()
{
  for(MeasPtrMap::iterator meas_it = mmpMeasurements.begin(); meas_it != mmpMeasurements.end(); ++meas_it)
  {
    MapPoint* pPoint = meas_it->first;
    Measurement* pMeas = meas_it->second;
    
    pMeas->bDeleted = true;
    
    if(pPoint->mMMData.GoodMeasCount() < 2)
    {
      pPoint->mbBad = true;
      pPoint->mbSelected = false;
    }
  }
}

void DeleteMeasurementsAction::UndoInternal()
{
  for(MeasPtrMap::iterator meas_it = mmpMeasurements.begin(); meas_it != mmpMeasurements.end(); ++meas_it)
  {
    MapPoint* pPoint = meas_it->first;
    Measurement* pMeas = meas_it->second;
    
    pMeas->bDeleted = false;
    
    if(pPoint->mMMData.GoodMeasCount() >= 2)
    {
      pPoint->mbBad = false;
    }
  }
}
  
