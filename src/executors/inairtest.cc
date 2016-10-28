#include "inairtest.h"

#include "executil.h"
#include "tstutil.h"

#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace std;

Exec::InAirTest::InAirTest (std::string ns, int id) : Executor (ns, id) {

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = false;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}


bool Exec::InAirTest::prepare () {
  bool res = true;
  ROS_INFO ("Exec::InAirTest::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::InAirTest::start () {
  ROS_INFO ("Exec::InAirTest::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work()) {
    ROS_ERROR("do_before_work failed");
    return;
  }

  ROS_INFO ("TREE NS: %s", tni.ns.c_str());
  ROS_INFO ("THIS ID: %d", tni.id);
  ROS_INFO ("PARENT ID: %d", tni.parent_id);

  //  sleep (5);

  string answer = "unknown";
  answer = "false";
  ROS_ERROR("inairtest answer: %s", answer.c_str());


  if (set_parameter_string (tni.ns, tni.parent_id, "answer", answer)) {
    ROS_INFO ("answer set in parent");
  } else {
    fail ("Failed to set answer parameter in parent");
    return;
  }
  
  ROS_INFO ("Exec::InAirTest: finished");

  wait_for_postwork_conditions ();
}


