// This may look like C code, but it is really -*- C++ -*-

#ifndef _SCAN_GROUND_SINGLE_H
#define _SCAN_GROUND_SINGLE_H

#include "executor.h"
#include "geoconvertros.h"

#include <string>

namespace Exec {

  class ScanGroundSingle : public virtual Executor {
  private:
    std::vector< std::map<std::string, double> > sensors_params;
    GeoConvertRos geoconv;
    std::string queue_uuid;

  public:
    ScanGroundSingle (std::string ns, int id);
    virtual ~ScanGroundSingle () {};

    virtual int expand (int free_id);
    virtual bool check ();

    bool prepare ();
    void start ();
    bool abort ();

    void push_command (ros::Publisher & pub, std::string sensor_type,
                       std::vector<geographic_msgs::GeoPoint> area);

  };

};

#endif
