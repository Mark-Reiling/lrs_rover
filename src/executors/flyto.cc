#include "flyto.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

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

    boost::this_thread::interruption_point();
    for (int i=0; i<5000; i++) {
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

bool Exec::FlyTo::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}
