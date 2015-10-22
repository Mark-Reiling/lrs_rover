#include "leashing.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::Leashing::Leashing (std::string ns, int id) : Executor (ns, id), enough_requested(false) {
  set_delegation_expandable(false);
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = true;
  set_exec_info(ns, id, einfo);
}


bool Exec::Leashing::check () {
  ROS_INFO ("Leashing CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::Leashing::prepare () {
  bool res = true;
  ROS_INFO ("Exec::Leashing::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::Leashing::start () {
  ROS_INFO ("Exec::Leashing::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  try {

    if (!do_before_work()) {
      return;
    }

    double desired_distance_xy = 4.0;
    get_param("desired_distance_xy", desired_distance_xy);

    double desired_distance_z = 2.0;
    get_param("desired_distance_z", desired_distance_z);

    double heading = 0.0;
    get_param("heading", heading);

    int xy_control_mode = 0;
    get_param("xy_control_mode", xy_control_mode);

    int z_control_mode = 0;
    get_param("z_control_mode", z_control_mode);

    int heading_control_mode = 0;
    get_param("heading_control_mode", heading_control_mode);

    ROS_ERROR ("leashing: Desired distance XY: %f", desired_distance_xy);
    ROS_ERROR ("leashing: Desired distance Z: %f", desired_distance_z);
    ROS_ERROR ("leashing: Heading: %f", heading);
    ROS_ERROR ("leashing: xy_control_mode: %d", xy_control_mode);
    ROS_ERROR ("leashing: z_control_mode: %d", z_control_mode);
    ROS_ERROR ("leashing: heading_control_mode: %d", heading_control_mode);


    boost::this_thread::interruption_point();
    while (!enough_requested) {
      usleep (100000);
      boost::this_thread::interruption_point();
    }

    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    ROS_ERROR("BOOST INTERUPTED IN leashing");
    set_succeeded_flag (node_ns, node_id, false);
    set_aborted_flag (node_ns, node_id, true);
    set_finished_flag (node_ns, node_id, true);
  }

}

bool Exec::Leashing::abort () {
  bool res = false;
  ROS_INFO("Exec::Leashing::abort");

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

bool Exec::Leashing::enough_execution () {
  bool res = true;
  ROS_ERROR ("Exec::Sequence::enough_execution");
  enough_requested = true;
  return res;
}
