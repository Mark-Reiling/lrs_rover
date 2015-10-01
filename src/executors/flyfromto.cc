#include "flyfromto.h"

#include <iostream>

#include "executil.h"

using namespace std;

bool Exec::FlyFromTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::FlyFromTo::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::FlyFromTo::start () {
  ROS_INFO ("Exec::FlyFromTo::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work ()) {
    return;
  }
  
  if (float64_params["commanded-speed"].have_value) {
    speed = float64_params["commanded-speed"].value;
  }

  geometry_msgs::PointStamped p;
  if (point_params["p"].have_value) {
    p = point_params["p"].value;
  } else {
    fail("parameter p is missing");
    return;
  }

  geometry_msgs::PointStamped p0;
  if (point_params["p0"].have_value) {
    p0 = point_params["p0"].value;
  } else {
    fail("parameter p0 is missing");
    return;
  }

  ROS_INFO ("txtexecutor flyfromto exec: %f %f %f %s -> %f %f %f %s - %f", 
	     p0.point.x, p0.point.y, p0.point.z, p0.header.frame_id.c_str(), 
	     p.point.x, p.point.y, p.point.z, p.header.frame_id.c_str(), 
	     speed);

  sleep(5);
  
  ROS_INFO ("Exec::FlyFromTo: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyFromTo::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyFromTo::abort");
  return res;
}

