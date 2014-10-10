#include "flyto.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

using namespace std;


bool Exec::FlyTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::FlyTo::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::FlyTo::start () {
  ROS_INFO ("Exec::FlyTo::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work ()) {
    return;
  }

  if (float64_params["speed"].have_value) {
    speed = float64_params["speed"].value;
  }

  geometry_msgs::PointStamped p;
  if (point_params["p"].have_value) {
    p = point_params["p"].value;
  } else {
    ROS_ERROR ("flyto: parameter p is missing");
    set_succeeded_flag (node_ns, node_id, false);
    set_finished_flag (node_ns, node_id, true);
    return;
  }

  ROS_INFO ("Exec::Flyto: Execution unit: %s", tni.execution_ns.c_str());

  ROS_INFO ("Exec::Flyto: %f %f %f %s - %f", p.point.x, p.point.y, p.point.z, 
	    p.header.frame_id.c_str(), speed);

  //
  // Replace the sleep with useful work.
  //

  sleep (5);

  //
  // When we reach this point the node execution whould be finished.
  //

  ROS_INFO ("Exec::FlyTo: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyTo::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyTo::abort");
  return res;
}

bool Exec::FlyTo::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}
