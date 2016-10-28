// This may look like C code, but it is really -*- C++ -*-

#ifndef _STOP_COLLECT_DATA_H
#define _STOP_COLLECT_DATA_H

#include "executor.h"

#include <string>

namespace Exec {

  class StopCollectData : public virtual Executor {
  private:
    std::string sensor_type;

  public:
    StopCollectData (std::string ns, int id);
    virtual ~StopCollectData ();

    bool prepare ();
    void start ();

  };

};

#endif
