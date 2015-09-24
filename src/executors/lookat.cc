#include "lookat.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;


bool Exec::LookAt::check () {
  ROS_INFO ("LookAt CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::LookAt::prepare () {
  bool res = true;
  ROS_INFO ("Exec::LookAt::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::LookAt::start () {
  ROS_INFO ("Exec::LookAt::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  sleep (10);

  wait_for_postwork_conditions ();

}

bool Exec::LookAt::abort () {
  bool res = false;
  ROS_INFO("Exec::LookAt::abort");

  return res;
}




