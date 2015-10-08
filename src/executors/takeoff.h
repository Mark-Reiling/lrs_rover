// This may look like C code, but it is really -*- C++ -*-

#ifndef _TAKEOFF_AT_H
#define _TAKEOFF_AT_H

#include "executor.h"

#include <string>

namespace Exec {

  class TakeOff : public virtual Executor {
  private:

  public:
    TakeOff (std::string ns, int id) : Executor (ns, id) {
      set_delegation_expandable(false);
    };

    virtual ~TakeOff () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
