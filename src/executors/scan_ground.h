// This may look like C code, but it is really -*- C++ -*-

#ifndef _SCAN_GROUND_H
#define _SCAN_GROUND_H

#include "executor.h"
#include "geoconvertros.h"

#include <string>

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
