// This may look like C code, but it is really -*- C++ -*-

#ifndef _STOP_VIDEO_RECORDING_H
#define _STOP_VIDEO_RECORDING_H

#include "executor.h"

#include <string>

namespace Exec {

  class StopVideoRecording : public virtual Executor {
  private:

  public:
    StopVideoRecording (std::string ns, int id);
    virtual ~StopVideoRecording () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();


  };
};
#endif
