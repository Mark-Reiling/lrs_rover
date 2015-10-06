#include "flyto.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::FlyTo::FlyTo (std::string ns, int id) : Executor (ns, id) {
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
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

    if (float64_params["commanded-speed"].have_value) {
      speed = float64_params["commanded-speed"].value;
    }
    
    geographic_msgs::GeoPoint p;
    if (geo_point_params["p"].have_value) {
      p = geo_point_params["p"].value;
      ROS_ERROR ("FLYTO: %f %f - %f", p.latitude, p.longitude, p.altitude);
    } else {
      fail ("flyto: parameter p is missing");
      return;
    }

    ROS_INFO ("Exec::Flyto: Execution unit: %s", tni.execution_ns.c_str());


    // The frame_id for p should be wgs84 or utm-ZONE-[south|north].

    ROS_INFO ("Exec::Flyto (WGS84 Ellipsoid alt): %f %f %f - %f", 
	      p.latitude, p.longitude, p.altitude, speed);

    //
    // Replace the sleep with useful work.
    //

    boost::this_thread::interruption_point();
    for (int i=0; i<10000; i++) {
      usleep(1000);
      boost::this_thread::interruption_point();
    }

    //
    // When we reach this point the node execution whould be finished.
    //

    ROS_INFO ("Exec::FlyTo: FINISHED");

    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    ROS_ERROR("BOOST INTERUPTED IN flyto");
    set_succeeded_flag (node_ns, node_id, false);
    set_aborted_flag (node_ns, node_id, true);
    set_finished_flag (node_ns, node_id, true);
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
