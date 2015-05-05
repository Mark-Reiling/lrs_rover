// This may look like C code, but it is really -*- C++ -*-

#ifndef _TAKE_TO_THE_AIR_H
#define _TAKE_TO_THE_AIR_H

#include "executor.h"

#include <string>

namespace Exec {

  class TakeToTheAir : public virtual Executor {
  private:

  public:
    TakeToTheAir (std::string ns, int id) : Executor (ns, id) {
      set_delegation_expandable(true);
    };

    virtual ~TakeToTheAir () {};

    virtual int expand (int free_id, std::vector<std::string> possible_units);

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
