#include "scanfromabove.h"

//#include "delutil.h"

#include <iostream>
#include <string>

using namespace std;

int Exec::ScanFromAbove::expand (int free_id) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  ROS_INFO("expand: %s", ns.c_str());

  // Place code doing the expansion

  return free_id;
}

bool Exec::ScanFromAbove::check () {
  bool res = true;
  string sensortype;
  if (get_param("sensor-type", sensortype)) {
    ROS_INFO("scanfromabove check - sensortype: %s", sensortype.c_str());

    //
    // Check that we have this sensor. Return false if we do not have this sensor.
    //

  }
  return res;
}


bool Exec::ScanFromAbove::prepare () {
  bool res = true;
  ROS_INFO ("Exec::ScanFromAbove::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}

void Exec::ScanFromAbove::start () {
  ROS_INFO ("Exec::ScanFromAbove::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  //
  // If the executor expands the node then the code below should be similar to the  
  // code for the sequence node.
  //
  // Otherwise the code below is the code doing the actual scan/mapping
  //


  //
  // Replace the sleeps with useful work.
  //


  //
  // We assume that this node is a non-expanding node and that the datacollection
  // and processing is done in this node.
  //

  //
  // Code to preparefor the data collection
  //

  string datauuid = string_params["data-uuid"].value;
  ROS_INFO("scanfromabove preparing for data collection for uuid: %s", datauuid.c_str());

  sleep(5); // Replace with the real code

  //
  // Do the actual scan/mapping flying
  // 

  if (points_params.find("area") == points_params.end()) {
    ROS_ERROR("expand: Parmeter area do not exist");
    return;
  }

  std::vector<geometry_msgs::PointStamped> points;
  points = points_params["area"].value;

  for (unsigned int i=0; i<points.size(); i++) {
    ROS_INFO("Point %d: %s - %f %f %f", i, points[i].header.frame_id.c_str(),
	     points[i].point.x, points[i].point.y, points[i].point.z);
  }

  std::vector<string> uavs;
  uavs = strings_params["uavs"].value;

  for (unsigned int i=0; i<uavs.size(); i++) {
    ROS_INFO("UAV %d: %s", i, uavs[i].c_str());
  }


  sleep(20); // Replace with the real code

  //
  // The flying is done. Process the data if needed.
  //

  sleep(5); // Replace with the real code

  //
  // Put information in the world data base about the generated data
  //

#if 0
  lrs_msgs_common::DataInfo di;
  string filename = datauuid + ".bag";
  di.uuid = datauuid;
  di.url = "http://abc.def.com/" + filename;
  di.datatype = di.DATATYPE_LIDAR;
  di.filetype = di.FILETYPE_BAG;
#endif

  //
  // When we reach this point the node execution whould be finished.
  //
  
  ROS_INFO ("Exec::ScanFromAbove: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::ScanFromAbove::abort () {
  bool res = false;
  ROS_INFO("Exec::ScanFromAbove::abort");

  return res;
}

bool Exec::ScanFromAbove::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;

  return res;
}

