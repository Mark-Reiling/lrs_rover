#include "movearm.h"

#include <iostream>

#include "lrs_msgs_tst/ConfirmReq.h"

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

using namespace std;


bool Exec::MoveArm::prepare () {
  bool res = true;
  ROS_INFO ("Exec::MoveArm::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::MoveArm::start () {
  ROS_INFO ("Exec::MoveArm::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work ()) {
    return;
  }

  geometry_msgs::PoseStamped p;
  if (pose_params["p"].have_value) {
    p = pose_params["p"].value;
  } else {
    fail("move-arm: parameter p is missing");
    return;
  }

  ROS_INFO ("Exec::MoveArm: Execution unit: %s", tni.execution_ns.c_str());

  ROS_INFO ("Exec::MoveArm: %f %f %f %s", 
            p.pose.position.x, p.pose.position.y, p.pose.position.z, 
            p.header.frame_id.c_str());

  //
  // Replace the sleep with useful work.
  //

  sleep (5);

  //
  // When we reach this point the node execution whould be finished.
  //

  ROS_INFO ("Exec::MoveArm: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::MoveArm::abort () {
  bool res = false;
  ROS_INFO("Exec::MoveArm::abort");
  return res;
}

