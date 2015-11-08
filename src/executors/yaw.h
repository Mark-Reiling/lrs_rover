// This may look like C code, but it is really -*- C++ -*-

#ifndef _YAW_H
#define _YAW_H

#include "executor.h"

#include <string>

namespace Exec {

  class Yaw : public virtual Executor {
  private:

  public:
    Yaw (std::string ns, int id);
    virtual ~Yaw () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
