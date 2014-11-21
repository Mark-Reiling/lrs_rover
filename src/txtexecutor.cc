#include "ros/ros.h"

#include <boost/thread.hpp>

#include "std_msgs/String.h"

#include <iostream>
#include <sstream>
#include <queue>

#include "lrs_msgs_tst/ConfirmReq.h"

#include "lrs_srvs_tst/TSTCreateExecutor.h"

#include "concurrent.h"
#include "sequence.h"
#include "testif.h"
#include "confirm.h"
#include "wait.h"
#include "executors/flyto.h"
#include "executors/flywaypoints.h"
#include "executors/movearm.h"

#include "executil.h"

std::map<std::string, Executor *> execmap;
std::map<std::string, boost::thread *> threadmap;
ros::NodeHandle * global_nh;
ros::Publisher * global_confirm_pub;

boost::mutex mutex;

using namespace std;

bool create_executor (lrs_srvs_tst::TSTCreateExecutor::Request  &req,
		      lrs_srvs_tst::TSTCreateExecutor::Response &res ) {
  boost::mutex::scoped_lock mutex;
  ROS_INFO("quadexecutor: create_executor: %s %d - %d", req.ns.c_str(), req.id, req.run_prepare);

  ostringstream os;
  os << req.ns << "-" << req.id;
  if (execmap.find (os.str()) != execmap.end()) {
    ROS_INFO ("Executor already exists, overwrites it: %s %d", req.ns.c_str(), req.id);
  }

  string type = get_node_type (req.ns, req.id);
  
  bool found = false;
  if (type == "conc") {
    execmap[os.str()] = new Exec::Concurrent (req.ns, req.id);
    found = true;
  }

  if (type == "seq") {
    execmap[os.str()] = new Exec::Sequence (req.ns, req.id);
    found = true;
  }

  if (type == "test-if") {
    execmap[os.str()] = new Exec::TestIf (req.ns, req.id);
    found = true;
  }

  if (type == "confirm") {
    execmap[os.str()] = new Exec::Confirm (req.ns, req.id);
    found = true;
  }

  if (type == "wait") {
    execmap[os.str()] = new Exec::Wait (req.ns, req.id);
    found = true;
  }

  if (type == "fly-to") {
    execmap[os.str()] = new Exec::FlyTo (req.ns, req.id);
    found = true;
  }

  if (type == "fly-waypoints") {
    execmap[os.str()] = new Exec::FlyWaypoints (req.ns, req.id);
    found = true;
  }

  if (type == "move-arm") {
    execmap[os.str()] = new Exec::MoveArm (req.ns, req.id);
    found = true;
  }

  if (found) {
    res.success = true;
    res.error = 0;
    if (req.run_prepare) {
      bool prep = execmap[os.str()]->prepare ();
      if (!prep) {
	res.success = false;
	res.error = 2;
      }
    }
  } else {
    ROS_ERROR ("Could not create executor of type: %s", type.c_str());
    res.success = false;
    res.error = 1;
  }

  return true;
}



int main(int argc, char **argv) {

  //  ros::init(argc, argv, "quadexecutor", ros::init_options::AnonymousName);
  ros::init(argc, argv, "quadexecutor");

  ros::NodeHandle n;
  global_nh = &n;

  ros::Publisher confirm_pub = n.advertise<lrs_msgs_tst::ConfirmReq>("confirm_request", 1, true); // queue size 1 and latched
  global_confirm_pub = &confirm_pub;

  std::vector<ros::ServiceServer> services;

  std::string prefix = "tst_executor/";

  services.push_back (n.advertiseService(prefix + "create_executor", create_executor));
  services.push_back (n.advertiseService(prefix + "executor_expand", executor_expand));
  services.push_back (n.advertiseService(prefix + "executor_get_constraints", executor_get_constraints));
  services.push_back (n.advertiseService(prefix + "start_executor", start_executor));

  ROS_INFO("Ready to be an executor factory");

  ros::spin();
  
  return 0;
}

