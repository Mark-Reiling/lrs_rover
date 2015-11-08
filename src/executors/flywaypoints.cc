#include "flywaypoints.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

using namespace std;


bool Exec::FlyWaypoints::prepare () {
  bool res = true;
  ROS_INFO ("Exec::FlyWaypoints::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::FlyWaypoints::start () {
  ROS_INFO ("Exec::FlyWaypoints::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work ()) {
    return;
  }

  std::vector<geographic_msgs::GeoPoint> geopoints;
  
  if (!get_param("waypoints", geopoints)) {
    fail ("flywaypoints: waypoints not set");
    return;
  }

  ROS_INFO("N WAYPOINTS: %zu", geopoints.size());
  
  bool segment_flag = false;
  bool any_order_flag = false;
  get_param ("segment-flag", segment_flag);
  get_param ("any_order_flag", any_order_flag);

  ROS_INFO ("Exec::FlyWaypoints: Execution unit: %s", tni.execution_ns.c_str());
  for (unsigned int i=0; i<geopoints.size(); i++) {
    geographic_msgs::GeoPoint p;
    p = geopoints[i];
    ROS_INFO ("Exec::Flywaypoints: %f %f %f", p.latitude, p.longitude, p.altitude);
    sleep (5);
  }

  ROS_INFO ("Exec::FlyWaypoints: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyWaypoints::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyWaypoints::abort");
  return res;
}

