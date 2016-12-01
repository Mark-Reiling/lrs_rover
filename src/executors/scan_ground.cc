#include "scan_ground.h"

#include "tstutil.h"
#include "executil.h"

#include <iostream>
#include <string>

#include <uuid/uuid.h>

#include "lrs_msgs_common/GetPartitioning.h"

extern std::map<std::string, boost::thread *> threadmap;
extern boost::mutex thread_map_lock;

using namespace std;


bool get_partitioning (std::string ns, std::vector<geometry_msgs::Point> polygon, 
		       std::vector<geometry_msgs::Point> sites, 
		       std::vector<double> area_req, 
		       std::vector<lrs_msgs_common::PointArray> & areas) {

  bool res = true;

  ros::NodeHandle n;
  ostringstream os;
  os << ns << "/partitioningserver/" << "get_partitioning";
  ros::ServiceClient client = n.serviceClient<lrs_msgs_common::GetPartitioning>(os.str());
  lrs_msgs_common::GetPartitioning srv;
  srv.request.polygon = polygon;
  srv.request.sites = sites;
  srv.request.area_request = area_req;

  if (client.call(srv)) {
    if (srv.response.success) {
      areas = srv.response.areas;
      res = true;
    } else {
      ROS_ERROR("Failed to get partitioning: %s - %d", ns.c_str(), srv.response.error);
      res = false;
    }
  } else {
    ROS_ERROR("Call to partitioning server failed: %s", ns.c_str());
    res = false;
  }
  return res;
}


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
    return free_id;
  } else {
    // ROS_ERROR("scan_ground check - sensortype: %s - %s", sensortype.c_str(), tni.execution_ns.c_str());
  }

  std::vector<geographic_msgs::GeoPoint> area;
  if (!get_param("area", area)) {
    ROS_ERROR("scan_gropund: Parameter 'area' do not exist or is not set");
    return free_id;
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

  if (n_sub_tasks == 0) {
    return free_id;
  }

  std::vector<lrs_msgs_common::PointArray> areas;
  if (n_sub_tasks == 1) {
    lrs_msgs_common::PointArray pa;
    for (unsigned int i=0; i<area.size(); i++) {
      geometry_msgs::PointStamped ps = geoconv.to_world (area[i]);
      geometry_msgs::Point p;
      p.x = ps.point.x;
      p.y = ps.point.y;
      p.z = 0.0;
      pa.points.push_back(p);
    }
    areas.push_back(pa);
  } else {
    std::vector<geometry_msgs::Point> polygon;
    std::vector<geometry_msgs::Point> sites;
    std::vector<double> area_req;
    for (unsigned int i=0; i<area.size(); i++) {
      ROS_ERROR("AREA %d: %f %f", i, area[i].latitude, area[i].longitude);
      geometry_msgs::PointStamped ps = geoconv.to_world (area[i]);
      geometry_msgs::Point p;
      p.x = ps.point.x;
      p.y = ps.point.y;
      p.z = 0.0;
      polygon.push_back(p);
      ROS_ERROR ("POLYGON POINT %d: %f %f", i, p.x, p.y);
    }
    for (unsigned int i=0; i<n_sub_tasks; i++) {
      area_req.push_back (1.0/n_sub_tasks);
      sites.push_back(polygon[i]);
    }
    if (get_partitioning(node_ns, polygon, sites, area_req, areas)) {

    } else {
      ROS_ERROR("Failed to get a partitioning");
      return free_id;
    }
  }


  int conc_id = create_child_node (node_ns, "conc", "conc", node_id);
  set_execution_unit(node_ns, conc_id, ns);
  set_parameter_int32(node_ns, conc_id, "unique_node_id", free_id++);

  if (sensortype == "artva") {
    int artva_filter_id = create_child_node (node_ns, "artva-filter", "artva-filter", conc_id);
    set_execution_unit(node_ns, artva_filter_id, ns);
    set_parameter_float64(node_ns, artva_filter_id, "limit", 10.0);

    int artva_trigger_id = create_child_node (node_ns, "artva-trigger", "artva-trigger", conc_id);
    set_execution_unit(node_ns, artva_trigger_id, ns);
    set_parameter_int32(node_ns, artva_trigger_id, "n_hits", 1);
    set_parameter_int32(node_ns, artva_trigger_id, "abort-unique-node-id", 1);
    set_parameter_int32(node_ns, artva_trigger_id, "unique_node_id", free_id++); 
 }

  for (unsigned int i=0; i<n_sub_tasks; i++) {
    int scan_id = create_child_node (node_ns, "scan-ground-single", "scan-ground-single", conc_id);
    std::vector<geographic_msgs::GeoPoint> a;
    for (unsigned int j=0; j<areas[i].points.size(); j++) {
      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "/world";
      ps.header.stamp = ros::Time::now();
      ps.point = areas[i].points[j];
      a.push_back(geoconv.to_geopoint(ps));
    }
    set_parameter_geopoints(node_ns, scan_id, "area", a);
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


