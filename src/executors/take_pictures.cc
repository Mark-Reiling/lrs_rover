#include "take_pictures.h"

#include <iostream>

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::TakePictures::TakePictures (std::string ns, int id) : Executor (ns, id), 
							    enough_requested(false),
							    pause_requested(false),
							    continue_requested(false) {

  add_resource_to_lock("camera");

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = true;
  einfo.can_be_paused = true;
  set_exec_info(ns, id, einfo);
}

bool Exec::TakePictures::prepare () {
  bool res = true;
  ROS_INFO ("Exec::TakePictures::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::TakePictures::start () {
  ROS_INFO ("Exec::TakePictures::start: %s - %d", node_ns.c_str(), node_id);

  try {

    if (!do_before_work ()) {
      return;
    }

    int n_pictures = 0;
    get_param("n", n_pictures);

    double delay = 1.0;
    get_param("delay_between_pictures_in_seconds", delay);

    ROS_INFO ("Exec::TakePictures: %d - %f", n_pictures, delay);


    //
    // Replace the sleep with useful work.
    //

    boost::this_thread::interruption_point();
    bool paused = false;
    int n = 0;
    double time = 0.0;
    while (true) {
      usleep(1000);
      boost::this_thread::interruption_point();
      if (enough_requested) {
	break;
      }
      if (pause_requested) {
	pause_requested = false;
	set_paused_flag (node_ns, node_id, true);
	paused = true;
      }
      if (continue_requested) {
	continue_requested = false;
	set_paused_flag (node_ns, node_id, false);
	paused = false;
      }
      if (!paused) {
	if (time > delay) {
	  time = 0.0;
	  n++;
	  ROS_INFO("TAKING PICTURE NUMBER: %d", n);
	}
      }
      if ((n != 0) && (n == n_pictures)) {
	break;
      }
      time += 0.001;
    }

    //
    // When we reach this point the node execution whould be finished.
    //

    ROS_INFO ("Exec::TakePictures: FINISHED");

    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail ("TakePictures ABORTED");
    return;
  }
}

bool Exec::TakePictures::abort () {
  bool res = false;
  ROS_ERROR("Exec::TakePictures::abort");
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

bool Exec::TakePictures::enough_execution () {
  bool res = true;
  ROS_ERROR ("Exec::TakePictures::enough_execution");
  enough_requested = true;
  return res;
}

bool Exec::TakePictures::request_pause () {
  bool res = true;
  ROS_ERROR ("Exec::TakePictures::request_pause");
  pause_requested = true;
  return res;
}

bool Exec::TakePictures::continue_execution () {
  bool res = true;
  ROS_ERROR ("Exec::TakePictures::request_continue");
  continue_requested = true;
  return res;
}
