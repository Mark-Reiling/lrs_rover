#include "operator_control.h"

#include <iostream>

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

using namespace std;


Exec::OperatorControl::OperatorControl (std::string ns, int id) : Executor (ns, id) {

  add_resource_to_lock("fly");

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = true;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);    
}

bool Exec::OperatorControl::prepare () {
  bool res = true;
  ROS_INFO ("Exec::OperatorControl::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::OperatorControl::start () {
  ROS_INFO ("Exec::OperatorControl::start: %s - %d", node_ns.c_str(), node_id);

  try {

    if (!do_before_work ()) {
      return;
    }

    ROS_INFO ("Exec::OperatorControl");

    //
    // Replace the sleep with useful work.
    //

    boost::this_thread::interruption_point();
    while (true) {
      usleep(1000);
      boost::this_thread::interruption_point();
      if (enough_requested ()) {
        break;
      }
    }

    //
    // When we reach this point the node execution whould be finished.
    //

    ROS_INFO ("Exec::OperatorControl: FINISHED");

    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail ("OperatorControl ABORTED");
    return;
  }
}
