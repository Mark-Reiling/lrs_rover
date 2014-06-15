#include "flyfromto.h"

#include <algorithm>
#include <iostream>

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

  fetch_node_info ();

  if (!is_active ()) {
    return;
  }

  if (!wait_for_prework_conditions ()) {
    return;
  }

  if (!init_params ()) {
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
  //  mission->set_aborted (true);
  return res;
}

bool Exec::FlyFromTo::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;

  res = true;

  return res;
}


bool Exec::FlyFromTo::init_from_node_info () {
  if (get_parameters (node_ns, node_id, params)) {

    istringstream isx0 (params["x0"]);
    istringstream isy0 (params["y0"]);
    istringstream isz0 (params["z0"]);
    isx0 >> x0;
    isy0 >> y0;
    isz0 >> z0;

    istringstream isx (params["x"]);
    istringstream isy (params["y"]);
    istringstream isz (params["z"]);

    isx >> x;
    isy >> y;
    isz >> z;

    istringstream isspeed (params["speed"]);
    isspeed >> speed;

    return true;
  } else {
    return false;
  }
}
