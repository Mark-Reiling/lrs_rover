#include "start_collect_data.h"

#include <iostream>
#include <string>
#include <vector>

#include "wdbutil.h"
#include "executil.h"

using namespace std;


Exec::StartCollectData::StartCollectData (std::string ns, int id) : Executor (ns, id) {

}

Exec::StartCollectData::~StartCollectData () {

}


bool Exec::StartCollectData::prepare () {
  bool res = true;
  ROS_INFO ("Exec::StartCollectData::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::StartCollectData::start () {
  ROS_INFO ("Exec::StartCollectData::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  string uuid = string_params["data-uuid"].value;

  sleep(5);

  wait_for_postwork_conditions ();
}

bool Exec::StartCollectData::abort () {
  bool res = false;
  ROS_INFO("Exec::StartCollectData::abort");

  return res;
}
