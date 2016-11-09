// This may look like C code, but it is really -*- C++ -*-

#ifndef _CHANGE_BATTERY_H
#define _CHANGE_BATTERY_H

#include "executor.h"

#include <string>

namespace Exec {

  class ChangeBattery : public virtual Executor {
  private:

  public:
    ChangeBattery (std::string ns, int id);
    virtual ~ChangeBattery ();

    virtual bool check ();
    
    bool prepare ();
    void start ();
  };

};

#endif
