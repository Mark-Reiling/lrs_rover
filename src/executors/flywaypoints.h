// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORFLYWAYPOINTS_H
#define _EXECUTORFLYWAYPOINTS_H

#include "executor.h"

namespace Exec {

  class FlyWaypoints: public virtual Executor {
  private:

  public:
    FlyWaypoints (std::string ns, int id);
    virtual ~FlyWaypoints () {};

    bool prepare ();
    void start ();
    bool abort ();

  };

};

#endif
