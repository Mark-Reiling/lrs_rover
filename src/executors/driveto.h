// This may look like C code, but it is really -*- C++ -*-

#ifndef _DRIVETO_AT_H
#define _DRIVETO_AT_H

#include "executor.h"
#include "geoconvertros.h"

#include <string>

namespace Exec {

  class DriveTo : public virtual Executor {
  private:
    GeoConvertRos geoconv;
    bool have_current_pose;
    geometry_msgs::PoseStamped current_pose;

  public:
    DriveTo (std::string ns, int id);
    virtual ~DriveTo () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);

  };
};
#endif
