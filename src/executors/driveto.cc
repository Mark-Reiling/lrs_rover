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
  // Internal check
  // check arm status string: TransportMode Busy
  // check rover

  return true;
}


bool Exec::DriveTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::DriveTo::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  // lock the arm and fail if it is not possible - using a service

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
          speed = 0.25;
        }
        if (qspeed == "standard") {
          speed = 0.5;
        }
        if (qspeed == "fast") {
          speed = 0.75;
        }
      } else {
        // Use default speed
        speed = 0.5;
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

    /*
     * here the GPS coordinate has to be translated to a NED and find the relative pose to the goal
     * and the result has to be sent to the driveTo action
     * hazard has to be check
     * in case of hazard DriveToObs has to be invoked
     *
     *
     * */
    ROS_INFO("Done");

    //Question1: what happens when I arrive? the node would finish?
    //Question2: Is there a timeout?
    //Question3: how can I report success or failure ?abort_fail("driveto ABORT");

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
