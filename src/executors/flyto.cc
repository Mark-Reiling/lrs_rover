#include "flyto.h"

#include <algorithm>
#include <iostream>
#include <boost/lexical_cast.hpp>

#include "lrs_msgs_tst/ConfirmReq.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

using namespace std;


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

  if (!do_before_work ()) {
    return;
  }

  ROS_INFO ("Exec::Flyto: %f %f %f - %f", x, y, z, speed);

  sleep (5);

  ROS_INFO ("Exec::FlyTo: FINISHED");

  wait_for_postwork_conditions ();
}

bool Exec::FlyTo::abort () {
  bool res = false;
  ROS_INFO("Exec::FlyTo::abort");
  return res;
}

bool Exec::FlyTo::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}


bool Exec::FlyTo::init_from_node_info () {
  bool res = true;
  if (get_parameters (node_ns, node_id, params)) {
    try {
      x = boost::lexical_cast<double>(params["x"]);
      y = boost::lexical_cast<double>(params["y"]);
      z = boost::lexical_cast<double>(params["z"]);
      speed = boost::lexical_cast<double>(params["speed"]);
    }
    catch(boost::bad_lexical_cast &) {
      ROS_ERROR("txt_exec flyto: BAD LEXICAL CAST");
      res = false;
    }
  } else {
    res = false;
  }
  return res;
}
