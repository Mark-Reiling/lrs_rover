// This may look like C code, but it is really -*- C++ -*-

#ifndef _START_CHARGING_H
#define _START_CHARGING_H

#include "executor.h"

#include <string>

namespace Exec {

  class StartCharging : public virtual Executor {
  private:
    std::string sensor_type;

  public:
    StartCharging (std::string ns, int id);
    virtual ~StartCharging ();

    virtual bool check ();
    
    bool prepare ();
    void start ();
    bool abort ();

  };

};

#endif
