// This may look like C code, but it is really -*- C++ -*-

#ifndef _START_COLLECT_DATA_H
#define _START_COLLECT_DATA_H

#include "executor.h"

#include <string>

namespace Exec {

  class StartCollectData : public virtual Executor {
  private:
    std::string sensor_type;

  public:
    StartCollectData (std::string ns, int id);
    virtual ~StartCollectData ();

    bool prepare ();
    void start ();
    bool abort ();

  };

};

#endif
