// This may look like C code, but it is really -*- C++ -*-

#ifndef _IN_AIR_GOAL_H
#define _IN_AIR_GOAL_H

#include "executor.h"

#include <string>

namespace Exec {

  class InAirGoal : public virtual Executor {
  private:

  public:
    InAirGoal (std::string ns, int id) : Executor (ns, id) {
      set_delegation_expandable(true);
    };

    virtual ~InAirGoal () {};

    virtual int expand (int free_id, std::vector<std::string> possible_units);

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
