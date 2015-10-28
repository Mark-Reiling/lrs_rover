#include "leashing.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

#include "lrs_msgs_common/LeashingStatus.h"

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::Leashing::Leashing (std::string ns, int id) : Executor (ns, id), 
						    enough_requested(false), 
						    have_current_position(false), 
						    have_command(false),
						    horizontal_control_mode(0),
						    vertical_control_mode(0),
						    yaw_control_mode(0) {
  set_delegation_expandable(false);
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = true;
  set_exec_info(ns, id, einfo);

  ROS_ERROR ("Leashing constructor ns - id: %s - %d", ns.c_str(), id);
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
    ROS_ERROR ("TRY TO SET ACTIVE FLAG: %s - %d", node_ns.c_str(), node_id);
    if (node_id < 0) {
      return false;
    } else {
      set_active_flag (node_ns, node_id, true);
    }
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

    double desired_horizontal_distance = 4.0;
    get_param("desired_horizontal_distance", desired_horizontal_distance);

    double desired_vertical_distance = 2.0;
    get_param("desired_vertical_distance", desired_vertical_distance);

    ROS_ERROR ("leashing: Desired horizontal distance: %f", desired_horizontal_distance);
    ROS_ERROR ("leashing: Desired vertical distance: %f", desired_vertical_distance);

    string command_topic = "leashing_command";
    get_param("command_topic", command_topic);

    string status_topic = "status_command";
    get_param("status_topic", status_topic);

    string position_topic = "/rescuer/geopose";
    get_param("position_topic", position_topic);

    ROS_ERROR("leashing: Position topic: %s", position_topic.c_str());
    ROS_ERROR("leashing: Command  topic: %s", command_topic.c_str());
    ROS_ERROR("leashing: Status   topic: %s", status_topic.c_str());

    position_sub = n.subscribe(position_topic, 1, &Exec::Leashing::position_callback, this);
    command_sub = n.subscribe(command_topic, 1, &Exec::Leashing::command_callback, this);

    ros::Publisher status_pub;
    status_pub = n.advertise<lrs_msgs_common::LeashingStatus>(status_topic,1);

    horizontal_distance = desired_horizontal_distance;
    horizontal_heading = 0.0;
    horizontal_control_mode = 1; // KEEP DISTANCE
    distance_north = 2.0;
    distance_east = 0.0;
    yaw = 0.0;

    vertical_distance = desired_vertical_distance;

    double period = 0.1;
    while (!enough_requested) {
      //    usleep (1000000.0*period);
      usleep (100000);
      boost::this_thread::interruption_point();

      if (horizontal_control_mode == lrs_msgs_common::LeashingCommand::HORIZONTAL_CONTROL_MODE_DISTANCE_HEADING_VEL) {
	horizontal_distance += horizontal_distance_vel * period;
	horizontal_heading += horizontal_heading_vel*10.0 * period;
      }

      if (horizontal_control_mode == lrs_msgs_common::LeashingCommand::HORIZONTAL_CONTROL_MODE_NORTH_EAST_VEL) {
	distance_north += distance_north_vel * period;
	distance_east += distance_east_vel * period;
      }

      if (vertical_control_mode == lrs_msgs_common::LeashingCommand::VERTICAL_CONTROL_MODE_VEL) {
	vertical_distance += vertical_distance_vel * period;
      }

      if (yaw_control_mode == lrs_msgs_common::LeashingCommand::YAW_CONTROL_MODE_VEL) {
	yaw += yaw_vel*10*period;
      }


      lrs_msgs_common::LeashingStatus status;
      status.horizontal_control_mode = horizontal_control_mode;
      status.vertical_control_mode = vertical_control_mode;
      status.yaw_control_mode = yaw_control_mode;
      status.horizontal_distance = horizontal_distance;
      status.horizontal_heading = horizontal_heading;
      status.distance_north = distance_north;
      status.distance_east = distance_east;
      status.vertical_distance = vertical_distance;
      status.yaw = yaw;
      // statis.yawpoint...
      status_pub.publish(status);

      if (have_current_position) {
	// Control the vehicle
      }
    }


#if 0
    string fail_reason;

    if (!qc_reset(fail_reason)) {
      fail ("leashing: " + fail_reason);
      return;
    }

    double x = 13;
    double y = 19;

    if (!qc_leashing (desired_distance_xy, desired_distance_z, heading, x, y,
		      xy_control_mode, z_control_mode, heading_control_mode,
		      fail_reason)) {
      fail ("leashing: " + fail_reason);
      return;
    }

    if (!qc_run(fail_reason)) {
      fail ("flyto qc_run failed: " + fail_reason);
      return;
    }
    
    boost::this_thread::interruption_point();
    while (!enough_requested) {
      usleep (100000);
      boost::this_thread::interruption_point();
      if (qc_finished(last_status)) {
	fail ("abnormal operator override");
	return;
      }
    }

    ROS_ERROR ("BEFORE qc_stop");

    if (!qc_stop(fail_reason)) {
      fail ("flyto qc_stop failed: " + fail_reason);
      return;
    }
#endif


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
  ROS_ERROR ("Exec::Leashing::enough_execution");
  enough_requested = true;
  return res;
}

void Exec::Leashing::position_callback(const geographic_msgs::GeoPose::ConstPtr & msg) {
  //  ROS_ERROR ("POSE CALLBACK:");
  current_geopose = *msg;
  have_current_position = true;
}

void Exec::Leashing::command_callback(const lrs_msgs_common::LeashingCommand::ConstPtr & msg) {
  //  ROS_ERROR("COMMAND CALLBACK");
  if (msg->horizontal_control_mode > 0) {
    horizontal_control_mode = msg->horizontal_control_mode;
  }
  if (msg->vertical_control_mode > 0) {
    vertical_control_mode = msg->vertical_control_mode;
  }
  if (msg->yaw_control_mode > 0) {
    yaw_control_mode = msg->yaw_control_mode;
  }

  if (horizontal_control_mode == msg->HORIZONTAL_CONTROL_MODE_DISTANCE_HEADING_ABSOLUTE) {
    horizontal_distance = msg->horizontal_distance;
    horizontal_heading = msg->horizontal_heading;
  }

  if (horizontal_control_mode == msg->HORIZONTAL_CONTROL_MODE_DISTANCE_HEADING_VEL) {
    horizontal_distance_vel = msg->horizontal_distance_vel;
    horizontal_heading_vel = msg->horizontal_heading_vel;
  }

  if (horizontal_control_mode == msg->HORIZONTAL_CONTROL_MODE_NORTH_EAST_ABSOLUTE) {
    distance_north = msg->distance_north;
    distance_east = msg->distance_east;
  }

  if (horizontal_control_mode == msg->HORIZONTAL_CONTROL_MODE_NORTH_EAST_VEL) {
    distance_north_vel = msg->distance_north_vel;
    distance_east_vel = msg->distance_east_vel;
  }

  if (vertical_control_mode == msg->VERTICAL_CONTROL_MODE_ABSOLUTE) {
    vertical_distance = msg->vertical_distance;
  }

  if (vertical_control_mode == msg->VERTICAL_CONTROL_MODE_VEL) {
    vertical_distance_vel = msg->vertical_distance_vel;
  }

  if (yaw_control_mode == msg->YAW_CONTROL_MODE_VEL) {
    yaw_vel = msg->yaw_vel;
  }

}
