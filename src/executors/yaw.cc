#include "yaw.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;

Exec::Yaw::Yaw (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);

  add_resource_to_lock("fly");
}


bool Exec::Yaw::check () {
  ROS_INFO ("Yaw CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::Yaw::prepare () {
  bool res = true;
  ROS_INFO ("Exec::Yaw::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::Yaw::start () {
  ROS_INFO ("Exec::Yaw::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }


  double heading;
  if (!get_param ("heading", heading)) {
    fail ("yaw: heading parameter missing");
    return;
  }

  std::string qrate = "standard";
  get_param ("yaw-rate", qrate);

  ROS_INFO ("yaw: heading: %f", heading);
  ROS_INFO ("yaw: yaw-rate: %s", qrate.c_str());


  sleep (10);

  wait_for_postwork_conditions ();

}

bool Exec::Yaw::abort () {
  bool res = false;
  ROS_INFO("Exec::Yaw::abort");

  return res;
}




