#include "scangroundsingle.h"

#include "wdbutil.h"

#include "executil.h"

#include <iostream>
#include <string>

using namespace std;

int Exec::ScanGroundSingle::expand (int free_id) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  ROS_INFO("expand: %s", ns.c_str());

  // Place code doing the expansion

  return free_id;
}

bool Exec::ScanGroundSingle::check () {
  bool res = true;
  string sensortype;

  fetch_node_info();

  if (init_params()) {
    if (get_param("sensor-type", sensortype)) {
      ROS_ERROR("scangroundsingle check - sensortype: %s - %s", sensortype.c_str(), tni.execution_ns.c_str());

      //
      // Check that we have this sensor. Return false if we do not have this sensor.
      //

      // Here a hard coded check for examples.

      if (sensortype == "laser") {
	if (tni.delegation_ns == "/uav0") {
	  return true;
	} else {
	  return false;
	}
      }

      if (sensortype == "camera") {
	if (tni.delegation_ns == "/uav1") {
	  return true;
	} else {
	  return false;
	}
      }
      
    }
  }
  return res;
}


bool Exec::ScanGroundSingle::prepare () {
  bool res = true;
  ROS_INFO ("Exec::ScanGroundSingle::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}

void Exec::ScanGroundSingle::start () {
  ROS_INFO ("Exec::ScanGroundSingle::start: %s - %d", node_ns.c_str(), node_id);

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

  std::vector<geographic_msgs::GeoPoint> points;
  if (!get_param("area", points)) {
    fail("Parameter 'area' do not exist or is not set");
    return;
  }

  //
  // Code to preparefor the data collection
  //

  string datauuid = string_params["data-uuid"].value;
  ROS_INFO("scangroundsingle preparing for data collection for uuid: %s", datauuid.c_str());

  sleep(5); // Replace with the real code

  //
  // Do the actual scan/mapping flying
  // 

  for (unsigned int i=0; i<points.size(); i++) {
    ROS_ERROR("GeoPoint %d: %f %f %f", i,
	      points[i].latitude, points[i].longitude, points[i].altitude);
  }

  //
  // To convert the points to different coordinate systems
  //

  sleep(20); // Replace with the real code

  //
  // The flying is done. Process the data if needed.
  //

  sleep(5); // Replace with the real code

  //
  // Put information in the world data base about the generated data
  //


  //
  // When we reach this point the node execution whould be finished.
  //
  
  ROS_INFO ("Exec::ScanGroundSingle: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::ScanGroundSingle::abort () {
  bool res = false;
  ROS_INFO("Exec::ScanGroundSingle::abort");

  return res;
}

