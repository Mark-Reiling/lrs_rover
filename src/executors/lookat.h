// This may look like C code, but it is really -*- C++ -*-

#ifndef _LOOK_AT_H
#define _LOOK_AT_H

#include "executor.h"

#include <string>

namespace Exec {

  class LookAt : public virtual Executor {
  private:
    bool enough_requested;

  public:
    LookAt (std::string ns, int id);
    virtual ~LookAt () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


    virtual bool enough_execution ();

  };
};
#endif
