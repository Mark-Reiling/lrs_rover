#include "lookat.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;


Exec::LookAt::LookAt (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = false;
  einfo.can_be_enoughed = true;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}

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

  geographic_msgs::GeoPoint gp;
  if (get_param("p", gp)) {
    ROS_ERROR ("Look at: %f %f - %f", gp.latitude, gp.longitude, gp.altitude);
  } else {
    fail("Could not get parameter p");
    return;
  }

  for (int i=0; i<10000; i++) {
    usleep (1000);
    if (enough_requested ()) {
      break;
    }
  }

  wait_for_postwork_conditions ();

}

bool Exec::LookAt::abort () {
  bool res = false;
  ROS_INFO("Exec::LookAt::abort");

  return res;
}

