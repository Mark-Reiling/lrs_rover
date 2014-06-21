// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORFLYFROMTO_H
#define _EXECUTORFLYFROMTO_H

#include "executor.h"

namespace Exec {

  class FlyFromTo : public virtual Executor {
  private:
    double x0, y0, z0;
    double x;
    double y;
    double z;
    double speed;

  public:
    FlyFromTo (std::string ns, int id) : Executor (ns, id) {

    };

    virtual ~FlyFromTo () {};

    bool prepare ();
    void start ();
    bool abort ();

    virtual bool get_constraints (std::vector<std::string> & cons);
    
  };

};

#endif
