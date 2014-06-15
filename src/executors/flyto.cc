#include "flyto.h"

#include <algorithm>
#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

using namespace std;


bool Exec::FlyTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::FlyTo::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::FlyTo::start () {
  ROS_INFO ("Exec::FlyTo::start: %s - %d", node_ns.c_str(), node_id);

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


  ROS_INFO ("Exec::Flyto: %f %f %f - %f", x, y, z, speed);

  sleep (5);

  ROS_INFO ("Exec::FlyTo: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyTo::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyTo::abort");
  //  mission->set_aborted (true);
  return res;
}

bool Exec::FlyTo::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;

  res = true;

  return res;
}


bool Exec::FlyTo::init_from_node_info () {
  if (get_parameters (node_ns, node_id, params)) {


    istringstream isx (params["x"]);
    istringstream isy (params["y"]);
    istringstream isz (params["z"]);
    istringstream isspeed (params["speed"]);

    isx >> x;
    isy >> y;
    isz >> z;
    isspeed >> speed;

    return true;
  } else {
    return false;
  }
}
