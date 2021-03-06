// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORFLYTO_H
#define _EXECUTORFLYTO_H

#include "executor.h"

namespace Exec {

  class FlyTo : public virtual Executor {
  private:
    double x;
    double y;
    double z;
    double speed;

  public:
    FlyTo (std::string ns, int id);
    virtual ~FlyTo () {};

    bool prepare ();
    void start ();

  };

};

#endif
