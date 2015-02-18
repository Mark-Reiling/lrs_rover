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

  ROS_INFO("N WAYPOINTS: %zu", points_params["waypoints"].value.size());

  if (points_params["waypoints"].have_value) {
    ROS_INFO("WAYPOINTS HAVE VALUE");
  } else {
    ROS_ERROR("WAYPOINTS DO NOT HAVE VALUE");
  }

  if (points_params.find("waypoints") == points_params.end()) {
    ROS_ERROR("expand: Parmeter waypoints do not exist");
    set_succeeded_flag (node_ns, node_id, false);
    set_finished_flag (node_ns, node_id, true);
    return;
  }

  bool segment_flag = false;
  bool any_order_flag = false;

  if (int32_params["segment-flag"].have_value) {
    segment_flag = int32_params["segment-flag"].value;
  }

  if (int32_params["any-order-flag"].have_value) {
    any_order_flag = int32_params["any-order-flag"].value;
  }

  std::vector<geometry_msgs::PointStamped> waypoints;
  if (points_params["waypoints"].have_value) {
    waypoints = points_params["waypoints"].value;
  } else {
    ROS_ERROR ("flywaypoints: parameter waypoints do hot have a value");
    set_succeeded_flag (node_ns, node_id, false);
    set_finished_flag (node_ns, node_id, true);
    return;
  }

  ROS_INFO ("Exec::FlyWaypoints: Execution unit: %s", tni.execution_ns.c_str());
  for (unsigned int i=0; i<waypoints.size(); i++) {
    geometry_msgs::PointStamped p;
    p = waypoints[i];
    ROS_INFO ("Exec::Flywaypoints: %f %f %f %s", p.point.x, p.point.y, p.point.z, 
	      p.header.frame_id.c_str());
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

bool Exec::FlyWaypoints::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}
