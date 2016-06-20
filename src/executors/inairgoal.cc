#include "inairgoal.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;

int Exec::InAirGoal::expand (int free_id, std::vector<std::string> possible_units, int expansion_try, 
                             int & expansion_can_be_tried) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  expansion_can_be_tried = expansion_try;

  ROS_INFO("expand: %s", ns.c_str());

  //
  // Get node parameters
  //

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return 0;
  }

  //
  // Expand the tree
  //

  vector<string> vars;
  vector<string> cons;

#if 0
  int seqid = create_child_node (node_ns, "seq", "seq", node_id);
  ROS_INFO("seqid:%d", seqid);
  set_execution_unit(node_ns, seqid, ns);
  set_constraints(node_ns, seqid, vars, cons);
  set_parameter_int32(node_ns, seqid, "unique_node_id", free_id++);
#endif

  //
  // Tell operator
  //

  int cid = create_child_node (node_ns, "tell-operator", "tell-operator", node_id);
  set_constraints(node_ns, cid, vars, cons);
  set_parameter_string(node_ns, cid, "execunit", "/uav1");
  //  set_parameter_string(node_ns, cid, "execunitalias", "B");
  set_parameter_string(node_ns, cid, "content", "take-to-the-air");
  set_parameter_string(node_ns, cid, "interaction-uuid", "18");
  set_parameter_int32(node_ns, cid, "unique_node_id", free_id++);

  return free_id;
}

bool Exec::InAirGoal::check () {
  ROS_INFO ("InAirGoal CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::InAirGoal::prepare () {
  bool res = true;
  ROS_INFO ("Exec::InAirGoal::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::InAirGoal::start () {
  ROS_INFO ("Exec::InAirGoal::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;

  if (!do_before_work()) {
    return;
  }

  // Code from sequence executor here, do function call


  if (!do_seq_work(tni, node_ns)) {
    fail("do_seq_work() failed");
    return;
  }

  wait_for_postwork_conditions ();

}

bool Exec::InAirGoal::abort () {
  bool res = false;
  ROS_INFO("Exec::InAirGoal::abort");

  return res;
}




