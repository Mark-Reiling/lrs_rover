#include "driveto.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

using namespace std;


Exec::DriveTo::DriveTo (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  set_exec_info(ns, id, einfo);
  update_from_exec_info (einfo);
}


bool Exec::DriveTo::check () {
  ROS_INFO ("DriveTo CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::DriveTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::DriveTo::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::DriveTo::start () {
  ROS_INFO ("Exec::DriveTo::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("pose", 1, &Exec::DriveTo::pose_callback, 
                                         this);


  try {

    if (!do_before_work()) {
      return;
    }

    double speed;

    if (!get_param("commanded-speed", speed)) {
      // Try to use the qualitative speed
      std::string qspeed;
      if (get_param("speed", qspeed)) {
        // Assign speed dependent on the value of qspeed
        if (qspeed == "slow") {
          speed = 1.0;
        }
        if (qspeed == "standard") {
          speed = 3.0;
        }
        if (qspeed == "fast") {
          speed = 7.0;
        }
      } else {
        // Use default speed
        speed = 3.0;
      }
    } 

    geographic_msgs::GeoPoint gp;
    if (get_param("p", gp)) {
      ROS_INFO ("DRIVETO: %f %f - %f - %f", gp.latitude, gp.longitude, gp.altitude, speed);
    } else {
      fail ("driveto: parameter p is missing");
      return;
    }

    ROS_INFO ("Exec::DriveTo (WGS84 Ellipsoid alt): %f %f - %f", 
              gp.latitude, gp.longitude,  speed);

    geometry_msgs::PointStamped p = geoconv.to_world (gp);

    ROS_INFO ("Exec::DriveTo (/world): %f %f", p.point.x, p.point.y);

    // Code doing the actual work

    sleep (10);


    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail("driveto ABORT");
    return;
  }

}

void Exec::DriveTo::pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
  ROS_INFO ("DriveTo POSE CALLBACK: %f %f", msg->pose.position.x, msg->pose.position.y);
  current_pose = *msg;
  have_current_pose = true;
}
