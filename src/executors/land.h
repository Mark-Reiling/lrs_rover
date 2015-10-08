// This may look like C code, but it is really -*- C++ -*-

#ifndef _LAND_AT_H
#define _LAND_AT_H

#include "executor.h"

#include <string>

namespace Exec {

  class Land : public virtual Executor {
  private:

  public:
    Land (std::string ns, int id) : Executor (ns, id) {
      set_delegation_expandable(false);
    };

    virtual ~Land () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
