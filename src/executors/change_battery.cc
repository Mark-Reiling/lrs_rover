#include "change_battery.h"

#include <iostream>
#include <string>
#include <vector>

#include "wdbutil.h"
#include "executil.h"

using namespace std;


Exec::ChangeBattery::ChangeBattery (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);
}

Exec::ChangeBattery::~ChangeBattery () {

}


bool Exec::ChangeBattery::check () {
  ROS_INFO ("StartDataStream CHECK");
  bool res = true;
  string sensortype;

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  if (tni.delegation_ns == "/uav0") {
    res = false;
  }

  return res;
}


bool Exec::ChangeBattery::prepare () {
  bool res = true;
  ROS_INFO ("Exec::ChangeBattery::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::ChangeBattery::start () {
  ROS_INFO ("Exec::ChangeBattery::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  sleep(5);

  wait_for_postwork_conditions ();
}

