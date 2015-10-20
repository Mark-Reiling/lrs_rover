// This may look like C code, but it is really -*- C++ -*-

#ifndef _DRIVETO_AT_H
#define _DRIVETO_AT_H

#include "executor.h"

#include <string>

namespace Exec {

  class DriveTo : public virtual Executor {
  private:

  public:
    DriveTo (std::string ns, int id);
    virtual ~DriveTo () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
