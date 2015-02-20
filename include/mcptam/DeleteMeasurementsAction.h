#ifndef __DELETE_MEASUREMENTS_ACTION_H
#define __DELETE_MEASUREMENTS_ACTION_H

#include <mcptam/EditAction.h>
#include <mcptam/KeyFrame.h>
#include <vector>

class Measurement;

class DeleteMeasurementsAction : public EditAction
{
public:

  DeleteMeasurementsAction(MeasPtrMap mpMeas);
  
protected:

  virtual void DoInternal();
  virtual void UndoInternal();

  MeasPtrMap mmpMeasurements;
  
};

#endif
