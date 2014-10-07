// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORMOVEARM_H
#define _EXECUTORMOVEARM_H

#include "executor.h"

namespace Exec {

  class MoveArm : public virtual Executor {
  private:

  public:
    MoveArm (std::string ns, int id) : Executor (ns, id) {

    };

    virtual ~MoveArm () {};

    bool prepare ();
    void start ();
    bool abort ();

    virtual bool get_constraints (std::vector<std::string> & cons);
    
  };

};

#endif
