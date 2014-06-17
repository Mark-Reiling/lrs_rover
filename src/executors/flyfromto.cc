#include "flyfromto.h"

#include <algorithm>
#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace std;

bool Exec::FlyFromTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::FlyFromTo::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::FlyFromTo::start () {
  ROS_INFO ("Exec::FlyFromTo::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work ()) {
    return;
  }

  ROS_ERROR ("RMAX flyto world exex: %f %f %f -> %f %f %f - %f", x0, y0, z0, x, y, z, speed);

  sleep(5);
  
  ROS_INFO ("Exec::FlyTo: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyFromTo::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyFromTo::abort");
  return res;
}

bool Exec::FlyFromTo::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}


bool Exec::FlyFromTo::init_from_node_info () {
  bool res = true;
  if (get_parameters (node_ns, node_id, params)) {
    try {
      x0 = boost::lexical_cast<double>(params["x0"]);
      y0 = boost::lexical_cast<double>(params["y0"]);
      z0 = boost::lexical_cast<double>(params["z0"]);
      x = boost::lexical_cast<double>(params["x"]);
      y = boost::lexical_cast<double>(params["y"]);
      z = boost::lexical_cast<double>(params["z"]);
      speed = boost::lexical_cast<double>(params["speed"]);
    }
    catch(boost::bad_lexical_cast &) {
      ROS_ERROR("txt_exec flyto: BAD LEXICAL CAST");
      res = false;
    }
  } else {
    res = false;
  }
  return res;
}
