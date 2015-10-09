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

  int follow_ground_flag = 0;
  get_param ("follow-ground-flag", follow_ground_flag);

  double follow_ground_altitude = 0.0;
  if (follow_ground_flag) {
    if (!get_param ("follow-ground-altitude", follow_ground_altitude)) {
      fail ("flyto: follow ground altitude must be specified");
      return;
    }
  }

  int must_be_near_from_position_flag = 0;
  get_param ("must-be-near-from-position-flag", must_be_near_from_position_flag);
  
  int fly_in_straight_line_flag = 0;
  get_param ("fly-in-a-straight-line-flag", fly_in_straight_line_flag);

  ROS_INFO ("txtexecutor flyfromto exec: %f %f %f -> %f %f %f - %f", 
	    p0.latitude, p0.longitude, p0.altitude,
	    p.latitude, p.longitude, p.altitude, 
	     speed);

  ROS_INFO ("Exec::Flyto: Follow ground: %d - %f", follow_ground_flag, follow_ground_altitude);

    ROS_INFO ("Exec::Flyto: must_be_near_from_position_flag: %d", must_be_near_from_position_flag);
    ROS_INFO ("Exec::Flyto: fly_in_straight_line_flag: %d", fly_in_straight_line_flag);

  sleep(5);
  
  ROS_INFO ("Exec::FlyFromTo: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyFromTo::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyFromTo::abort");
  return res;
}

