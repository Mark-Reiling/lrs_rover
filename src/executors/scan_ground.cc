#include "scan_ground.h"

#include "tstutil.h"
#include "executil.h"

#include <iostream>
#include <string>

#include <uuid/uuid.h>

extern std::map<std::string, boost::thread *> threadmap;
extern boost::mutex thread_map_lock;

using namespace std;

Exec::ScanGround::ScanGround (std::string ns, int id) : Executor (ns, id) {

  // Set to true if the executor should expand the node during delegation
  set_delegation_expandable(true);

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}

int Exec::ScanGround::expand (int free_id, std::vector<std::string> possible_units,
                              int expansion_try, int & expansion_can_be_tried) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  ROS_INFO("expand: %s", ns.c_str());

  // Place code doing the expansion

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("ScanGround check: init_params failed");
    return false;
  }

  string sensortype;
  if (!get_param("sensor-type", sensortype)) {
    ROS_ERROR ("ScanGround: Could not get sensor_type parameter");
    return false;
  } else {
    // ROS_ERROR("scan_ground check - sensortype: %s - %s", sensortype.c_str(), tni.execution_ns.c_str());
  }

  std::vector<geographic_msgs::GeoPoint> area;
  if (!get_param("area", area)) {
    ROS_ERROR("scan_gropund: Parameter 'area' do not exist or is not set");
    return false;
  }

  unsigned int n_sub_tasks = 0;
  
  for (unsigned int i=0; i<possible_units.size(); i++) {
    ROS_ERROR("POSSIBLE UNIT: %s", possible_units[i].c_str());
    if (sensortype == "artva") {
      if (possible_units[i] == "/uav1") {
	n_sub_tasks++;
      }
    }
    if (sensortype == "camera") {
      if (possible_units[i] == "/uav2") {
	n_sub_tasks++;
      }
      if (possible_units[i] == "/uav3") {
	n_sub_tasks++;
      }
    }
  }
  



  int conc_id = create_child_node (node_ns, "conc", "conc", node_id);
  set_execution_unit(node_ns, conc_id, ns);
  set_parameter_int32(node_ns, conc_id, "unique_node_id", free_id++);

  if (sensortype == "artva") {
    int artva_trigger_id = create_child_node (node_ns, "artva-trigger", "artva-trigger", conc_id);
    set_execution_unit(node_ns, artva_trigger_id, ns);
    set_parameter_float64(node_ns, artva_trigger_id, "limit", 10.0);
    set_parameter_int32(node_ns, artva_trigger_id, "abort-unique-node-id", 1);
    set_parameter_int32(node_ns, artva_trigger_id, "unique_node_id", free_id++); 
 }

  for (unsigned int i=0; i<n_sub_tasks; i++) {
    int scan_id = create_child_node (node_ns, "scan-ground-single", "scan-ground-single", conc_id);
    set_parameter_geopoints(node_ns, scan_id, "area", area);
    set_parameter_string(node_ns, scan_id, "sensor-type", sensortype);
    set_parameter_int32(node_ns, scan_id, "unique_node_id", free_id++);
  }


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

#if 0
    while (true) {
      usleep(1000);
      boost::this_thread::interruption_point();
    }
#endif

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
  boost::mutex::scoped_lock lock(thread_map_lock);    
  bool res = false;
  ROS_INFO("Exec::ScanGround::abort");

  ostringstream os;
  os << node_ns << "-" << node_id;
  if (threadmap.find (os.str()) != threadmap.end()) {

    for (unsigned int i=0; i<tni.children.size(); i++) {
      ROS_INFO("SCAN GROUND ABORT CHILDREN %d: %d", i, tni.children[i]);
      set_abort_executor(node_ns, tni.children[i], true);
    }

    ROS_INFO("SCAN GROUND EXISTS: Sending interrupt to running thread");
    threadmap[os.str()]->interrupt();

    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }
  return res;
}


