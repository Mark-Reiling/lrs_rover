#include "scan_ground.h"

#include "executil.h"

#include <iostream>
#include <string>

#include <uuid/uuid.h>

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;

Exec::ScanGround::ScanGround (std::string ns, int id) : Executor (ns, id) {

  add_resource_to_lock("fly");
  
  // Set to true if the executor should expand the node during delegation
  set_delegation_expandable(true);

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}

int Exec::ScanGround::expand (int free_id) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  ROS_INFO("expand: %s", ns.c_str());

  // Place code doing the expansion

  return free_id;
}

bool Exec::ScanGround::check () {
  bool res = true;
  string sensortype;

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("ScanGround check: init_params failed");
    return false;
  }

  if (!get_param("sensor-type", sensortype)) {
    ROS_ERROR ("ScanGround: Could not get sensor_type parameter");
    return false;
  } else {
    // ROS_ERROR("scan_ground check - sensortype: %s - %s", sensortype.c_str(), tni.execution_ns.c_str());
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


bool Exec::ScanGround::prepare () {
  bool res = true;
  ROS_INFO ("Exec::ScanGround::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}

void Exec::ScanGround::start () {
  ROS_INFO ("Exec::ScanGround::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;
  
  try {
    if (!do_before_work()) {
      return;
    }

    //
    // If the executor expands the node then the code below should be similar to the  
    // code for the sequence node.
    //
    // Otherwise the code below is the code doing the actual scan/mapping
    //

    bool sres = do_seq_work(tni, node_ns);
    if (!sres) {
      fail("Exec::ScanGround: Sequence work failed");
      return;
    }


    //
    // When we reach this point the node execution whould be finished.
    //
    
    ROS_INFO ("Exec::ScanGround: FINISHED");
    
    wait_for_postwork_conditions ();

  }
  catch (boost::thread_interrupted) {
    abort_fail ("scan_ground ABORTED");
    return;
  }
}

bool Exec::ScanGround::abort () {
  bool res = false;
  ROS_INFO("Exec::ScanGround::abort");

  ostringstream os;
  os << node_ns << "-" << node_id;
  if (threadmap.find (os.str()) != threadmap.end()) {
    ROS_INFO("EXECUTOR EXISTS: Sending interrupt to running thread");
    threadmap[os.str()]->interrupt();

    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }
  return res;
}


