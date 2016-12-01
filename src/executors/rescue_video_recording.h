// This may look like C code, but it is really -*- C++ -*-

#ifndef _RESCUE_VIDEO_RECORDING_H
#define _RESCUE_VIDEO_RECORDING_H

#include "executor.h"
#include <string>

namespace Exec {

  class RescueVideoRecording : public virtual Executor {
  private:
    ros::Publisher display_pub;
    
  public:
    RescueVideoRecording (std::string ns, int id);
    virtual ~RescueVideoRecording() {};

    bool prepare ();
    void start ();

    virtual bool get_constraints (std::vector<std::string> & cons);

  };

};

#endif
