// This may look like C code, but it is really -*- C++ -*-

#ifndef _SCAN_GROUND_H
#define _SCAN_GROUND_H

#include "executor.h"
#include "geoconvertros.h"

#include "lrs_msgs_common/PointArray.h"

#include <string>

bool get_partitioning (std::string ns, 
                       std::vector<geometry_msgs::Point> polygon, 
                       std::vector<geometry_msgs::Point> sites, 
                       std::vector<double> area_req, 
                       std::vector<lrs_msgs_common::PointArray> & areas);

namespace Exec {

  class ScanGround : public virtual Executor {
  private:
    std::vector< std::map<std::string, double> > sensors_params;
    GeoConvertRos geoconv;

  public:
    ScanGround (std::string ns, int id);
    virtual ~ScanGround () {};

    virtual int expand (int free_id, std::vector<std::string> possible_units, int expansion_try, 
			int & expansion_can_be_tried);
    virtual bool check ();

    bool prepare ();
    void start ();
    bool abort ();

  };

};

#endif
