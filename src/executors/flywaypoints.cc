#include "flywaypoints.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

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

  speed = 2.0;

  if (float64_params["speed"].have_value) {
    speed = float64_params["speed"].value;
  }

  std::vector<geometry_msgs::PointStamped> waypoints;
  if (point_params["waypoints"].have_value) {
    waypoints = points_params["waypoints"].value;
  } else {
    ROS_ERROR ("flywaypoints: parameter waypoints is missing");
    set_succeeded_flag (node_ns, node_id, false);
    set_finished_flag (node_ns, node_id, true);
    return;
  }

  ROS_INFO ("Exec::FlyWaypoints: Execution unit: %s", tni.execution_ns.c_str());
  for (unsigned int i=0; i<waypoints.size(); i++) {
    geometry_msgs::PointStamped p;
    p = waypoints[i];
    ROS_INFO ("Exec::Flywaypoints: %f %f %f %s - %f", p.point.x, p.point.y, p.point.z, 
	      p.header.frame_id.c_str(), speed);
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
