// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORFLYWAYPOINTS_H
#define _EXECUTORFLYWAYPOINTS_H

#include "executor.h"

namespace Exec {

  class FlyWaypoints: public virtual Executor {
  private:
    double speed;

  public:
    FlyWaypoints (std::string ns, int id) : Executor (ns, id) {

    };

    virtual ~FlyWaypoints () {};

    bool prepare ();
    void start ();
    bool abort ();

    virtual bool get_constraints (std::vector<std::string> & cons);
    
  };

};

#endif
