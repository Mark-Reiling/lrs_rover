#include "driveto.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::DriveTo::DriveTo (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  set_exec_info(ns, id, einfo);
}


bool Exec::DriveTo::check () {
  ROS_INFO ("DriveTo CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::DriveTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::DriveTo::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::DriveTo::start () {
  ROS_INFO ("Exec::DriveTo::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  try {

    if (!do_before_work()) {
      return;
    }

    double speed;

    if (!get_param("commanded-speed", speed)) {
      // Try to use the qualitative speed
      std::string qspeed;
      if (get_param("speed", qspeed)) {
	// Assign speed dependent on the value of qspeed
	if (qspeed == "slow") {
	  speed = 1.0;
	}
	if (qspeed == "standard") {
	  speed = 3.0;
	}
	if (qspeed == "fast") {
	  speed = 7.0;
	}
      } else {
	// Use default speed
	speed = 3.0;
      }
    } 

    geographic_msgs::GeoPoint p;
    if (get_param("p", p)) {
      ROS_ERROR ("FLYTO: %f %f - %f - %f", p.latitude, p.longitude, p.altitude, speed);
    } else {
      fail ("flyto: parameter p is missing");
      return;
    }

    ROS_INFO ("Exec::DriveTo (WGS84 Ellipsoid alt): %f %f - %f", 
	      p.latitude, p.longitude,  speed);


    sleep (10);


    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    ROS_ERROR("BOOST INTERUPTED IN driveto");
    set_succeeded_flag (node_ns, node_id, false);
    set_aborted_flag (node_ns, node_id, true);
    set_finished_flag (node_ns, node_id, true);
  }

}

bool Exec::DriveTo::abort () {
  bool res = false;
  ROS_INFO("Exec::DriveTo::abort");

  ostringstream os;
  os << node_ns << "-" << node_id;
  if (threadmap.find (os.str()) != threadmap.end()) {
    ROS_ERROR("EXECUTOR EXISTS: Sending interrupt to running thread");
    threadmap[os.str()]->interrupt();
    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }

  return res;
}




