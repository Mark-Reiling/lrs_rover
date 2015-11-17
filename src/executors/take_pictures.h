// This may look like C code, but it is really -*- C++ -*-

#ifndef _TAKE_PICTURES_H
#define _TAKE_PICTURES_H

#include "executor.h"

namespace Exec {

  class TakePictures : public virtual Executor {
  private:
    bool enough_requested;
    bool pause_requested;
    bool continue_requested;
    double x;
    double y;
    double z;
    double speed;

  public:
    TakePictures (std::string ns, int id);
    virtual ~TakePictures () {};

    bool prepare ();
    void start ();
    bool abort ();

    virtual bool enough_execution ();
    virtual bool request_pause ();
    virtual bool continue_execution ();

  };

};

#endif
