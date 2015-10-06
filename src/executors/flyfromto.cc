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

  geographic_msgs::GeoPoint p;
  if (geo_point_params["p"].have_value) {
    p = geo_point_params["p"].value;
  } else {
    fail("parameter p is missing");
    return;
  }

  geographic_msgs::GeoPoint p0;
  if (geo_point_params["p0"].have_value) {
    p0 = geo_point_params["p0"].value;
  } else {
    fail("parameter p0 is missing");
    return;
  }

  ROS_INFO ("txtexecutor flyfromto exec: %f %f %f -> %f %f %f - %f", 
	    p0.latitude, p0.longitude, p0.altitude,
	    p.latitude, p.longitude, p.altitude, 
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

