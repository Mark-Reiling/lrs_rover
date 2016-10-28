// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORINAIRTEST_H
#define _EXECUTORINAIRTEST_H

#include "executor.h"
#include "lrs_msgs_tst/TSTNodeInfo.h"
#include <tf/transform_listener.h>



namespace Exec {

  class InAirTest : public virtual Executor {
  private:
    std::map<std::string, std::string> params;
    tf::TransformListener listener;

  public:
    InAirTest (std::string ns, int id);
    virtual ~InAirTest () {};

    bool prepare ();
    void start ();

  };

};

#endif
