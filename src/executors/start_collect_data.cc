#include "start_collect_data.h"

#include <iostream>
#include <string>
#include <vector>

#include "wdbutil.h"
#include "executil.h"

using namespace std;


Exec::StartCollectData::StartCollectData (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);
}

Exec::StartCollectData::~StartCollectData () {

}


bool Exec::StartCollectData::check () {
  ROS_INFO ("StartDataStream CHECK");
  bool res = true;
  string sensortype;

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  if (!get_param("sensor-type", sensortype)) {
    ROS_ERROR ("StartDataStream: Could not get sensor_type parameter");
    return false;
  } else {
    // ROS_ERROR("scangroundsingle check - sensortype: %s - %s", sensortype.c_str(), tni.execution_ns.c_str());
  }

  if (sensortype == "pointcloud") {
    if (tni.delegation_ns != "/uav0") {
      res = false;
    }
  }

  if (sensortype == "IR+camera") {
    if (tni.delegation_ns != "/uav0") {
      res = false;
    }
  }

  if (sensortype == "artva") {
    if (tni.delegation_ns != "/uav1") {
      res = false;
    }
  }

  if (sensortype == "camera") {
    if ((tni.delegation_ns != "/uav2") && (tni.delegation_ns != "/uav3")) {
      res = false;
    }
  }
  

  return res;
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

