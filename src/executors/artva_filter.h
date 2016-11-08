// This may look like C code, but it is really -*- C++ -*-

#ifndef _ARTVA_FILTER_H
#define _ARTVA_FILTER_H

#include "executor.h"
#include "std_msgs/Float64.h"

namespace Exec {

  class ArtvaFilter : public virtual Executor {
  private:
    double last_value;
    bool have_value;
    int enough_id;
    int abort_id;
    int pause_id;
    int continue_id;
    std::string global_tree_uuid;

  public:
    ArtvaFilter (std::string ns, int id);
    virtual ~ArtvaFilter () {};

    bool prepare ();
    void start ();

    void artva_callback(const std_msgs::Float64::ConstPtr& msg);
  };

};

#endif
