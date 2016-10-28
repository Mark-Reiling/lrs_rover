// This may look like C code, but it is really -*- C++ -*-

#ifndef _START_VIDEO_RECORDING_H
#define _START_VIDEO_RECORDING_H

#include "executor.h"

#include <string>

namespace Exec {

  class StartVideoRecording : public virtual Executor {
  private:

  public:
    StartVideoRecording (std::string ns, int id);
    virtual ~StartVideoRecording () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();


  };
};
#endif
