// This may look like C code, but it is really -*- C++ -*-

#ifndef _OPERATOR_CONTROL_H
#define _OPERATOR_CONTROL_H

#include "executor.h"

namespace Exec {

  class OperatorControl : public virtual Executor {
  private:
    double x;
    double y;
    double z;
    double speed;

  public:
    OperatorControl (std::string ns, int id);
    virtual ~OperatorControl () {};

    bool prepare ();
    void start ();

  };

};

#endif
