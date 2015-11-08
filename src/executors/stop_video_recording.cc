#include "stop_video_recording.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;

Exec::StopVideoRecording::StopVideoRecording (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);
}


bool Exec::StopVideoRecording::check () {
  ROS_INFO ("StopVideoRecording CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::StopVideoRecording::prepare () {
  bool res = true;
  ROS_INFO ("Exec::StopVideoRecording::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::StopVideoRecording::start () {
  ROS_INFO ("Exec::StopVideoRecording::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  sleep (3);

  wait_for_postwork_conditions ();

}

bool Exec::StopVideoRecording::abort () {
  bool res = false;
  ROS_INFO("Exec::StopVideoRecording::abort");

  return res;
}




