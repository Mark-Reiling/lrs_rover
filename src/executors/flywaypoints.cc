#include "flywaypoints.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;

Exec::FlyWaypoints::FlyWaypoints (std::string ns, int id) : Executor (ns, id) {
  add_resource_to_lock("fly");

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

}


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


  try {

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
    for (int i=0; i<5000; i++) {
      boost::this_thread::interruption_point();
      usleep (1000);
    }

  }

  ROS_INFO ("Exec::FlyWaypoints: FINISHED");

  wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail ("flyto ABORTED");
    return;
  }
}

bool Exec::FlyWaypoints::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyWaypoints::abort");
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

