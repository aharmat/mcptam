#include <mcptam/DeleteMeasurementsAction.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <gvars3/instances.h>

using namespace GVars3;

DeleteMeasurementsAction::DeleteMeasurementsAction(MeasPtrMap mpMeas)
: mmpMeasurements(mpMeas)
{
  
}

void DeleteMeasurementsAction::DoInternal()
{
  static gvar3<int> gvnMinMeas("MinMeas", 2, HIDDEN|SILENT);
  
  for(MeasPtrMap::iterator meas_it = mmpMeasurements.begin(); meas_it != mmpMeasurements.end(); ++meas_it)
  {
    MapPoint* pPoint = meas_it->first;
    Measurement* pMeas = meas_it->second;
    
    pMeas->bDeleted = true;
    
    if(pPoint->mMMData.GoodMeasCount() < *gvnMinMeas)
    {
      pPoint->mbBad = true;
      pPoint->mbSelected = false;
    }
  }
}

void DeleteMeasurementsAction::UndoInternal()
{
  static gvar3<int> gvnMinMeas("MinMeas", 2, HIDDEN|SILENT);
  
  for(MeasPtrMap::iterator meas_it = mmpMeasurements.begin(); meas_it != mmpMeasurements.end(); ++meas_it)
  {
    MapPoint* pPoint = meas_it->first;
    Measurement* pMeas = meas_it->second;
    
    pMeas->bDeleted = false;
    
    if(pPoint->mMMData.GoodMeasCount() >= *gvnMinMeas)
    {
      pPoint->mbBad = false;
    }
  }
}
  
