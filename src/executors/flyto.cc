#include "flyto.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::FlyTo::FlyTo (std::string ns, int id) : Executor (ns, id) {

  add_resource_to_lock("fly");

  set_can_be_paused(true);
  set_can_be_enoughed(true);

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = true;
  einfo.can_be_paused = true;
  set_exec_info(ns, id, einfo);
}

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

  try {

    if (!do_before_work ()) {
      return;
    }


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

    bool follow_ground_flag = false;
    get_param ("follow-ground-flag", follow_ground_flag);

    double follow_ground_altitude = 0.0;
    if (follow_ground_flag) {
      if (!get_param ("follow-ground-altitude", follow_ground_altitude)) {
	fail ("flyto: follow ground altitude must be specified");
	return;
      }
    }

    ROS_INFO ("Exec::Flyto: Execution unit: %s", tni.execution_ns.c_str());

    ROS_INFO ("Exec::Flyto (WGS84 Ellipsoid alt): %f %f %f - %f", 
	      p.latitude, p.longitude, p.altitude, speed);

    ROS_INFO ("Exec::Flyto: Follow ground: %d - %f", follow_ground_flag, follow_ground_altitude);

    //
    // Replace the sleep with useful work.
    //

    bool paused = false;
    boost::this_thread::interruption_point();
    for (int i=0; i<10000; i++) {
      usleep(1000);
      boost::this_thread::interruption_point();
      if (enough_requested ()) {
	break;
      }
      if (pause_requested()) {
	paused = true;
	clear_pause_requested ();
	set_paused_flag (node_ns, node_id, true);
      }
      if (continue_requested ()) {
	paused = false;
	clear_continue_requested ();
	set_paused_flag (node_ns, node_id, false);
      }
      if (paused) {
	i--;
      }
    }

    //
    // When we reach this point the node execution whould be finished.
    //

    ROS_INFO ("Exec::FlyTo: FINISHED");

    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail ("flyto ABORTED");
    return;
  }
}

bool Exec::FlyTo::abort () {
  bool res = false;
  ROS_ERROR("Exec::FlyTo::abort");
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

