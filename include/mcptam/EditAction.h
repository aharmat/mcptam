#ifndef __EDIT_ACTION_H
#define __EDIT_ACTION_H

class EditAction
{
public:

  void Do(){ DoInternal(); }
  void Undo(){ UndoInternal(); }
  
protected:
  
  virtual void DoInternal() = 0;
  virtual void UndoInternal() = 0;
  
};

#endif
