// This may look like C code, but it is really -*- C++ -*-

#ifndef _LEASHING_H
#define _LEASHING_H

#include "executor.h"

#include <string>

namespace Exec {

  class Leashing : public virtual Executor {
  private:
    bool enough_requested;

  public:
    Leashing (std::string ns, int id);
    virtual ~Leashing () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();

    virtual bool enough_execution ();

  };
};
#endif
