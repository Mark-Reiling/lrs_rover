#include "artva_filter.h"

#include <iostream>

#include "executil.h"
#include "tstutil.h"

using namespace std;

Exec::ArtvaFilter::ArtvaFilter (std::string ns, int id) : Executor (ns, id),
							    last_value(0.0),
							    have_value(false),
                                                            enough_id(-1),
                                                            abort_id(-1),
                                                            pause_id(-1),
                                                            continue_id(-1) {

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);
  update_from_exec_info (einfo);      
}


bool Exec::ArtvaFilter::prepare () {
  bool res = true;
  ROS_INFO ("Exec::ArtvaFilter::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}

void Exec::ArtvaFilter::artva_callback(const std_msgs::Float64::ConstPtr& msg) {
  ROS_ERROR("artva_callback: %f", msg->data);
  have_value = true;
  last_value = msg->data;
}

void Exec::ArtvaFilter::start () {
  ROS_INFO ("Exec::ArtvaFilter::start: %s - %d", node_ns.c_str(), node_id);

  try {
    if (!do_before_work()) {
      return;
    }
    
    double limit;
    if (!get_param("limit", limit)) {
      fail("artva-filter: parameter limit is missing");
    }

    bool loop_flag = false;
    bool burst_flag = false;

    get_param("enough-unique-node-id", enough_id);
    get_param("pause-unique-node-id", pause_id);
    get_param("continue-unique-node-id", continue_id);
    get_param("abort-unique-node-id", abort_id);
    get_param("loop-flag", loop_flag);
    get_param("burst-flag", burst_flag);
    
    ros::NodeHandle n;
    std::string ns = ros::names::clean (ros::this_node::getNamespace());

    ros::Subscriber artvae_sub = n.subscribe("/artva", 10,
                                             &Exec::ArtvaFilter::artva_callback, this);
    ros::Subscriber fake_artva_sub = n.subscribe("/fake_artva", 10,
                                                 &Exec::ArtvaFilter::artva_callback, this);
    std::string fail_reason = "";

    if ((abort_id >= 0) || (enough_id >= 0)) {
      if (!get_global_tree_uuid_from_node(node_ns, node_id, global_tree_uuid)) {
        fail("Could not get uuid for tree");
        return;
      }
    }

    do {
      boost::this_thread::interruption_point();
      while (!enough_requested()) {
        usleep (10000);
        boost::this_thread::interruption_point();
        if (have_value) {
          if (last_value > limit) {
            break;
          }
        }
      }
      if (loop_flag) {
        have_value = false;
        if (!send_abort(global_tree_uuid, abort_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (!send_enough(global_tree_uuid, enough_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (!send_pause(global_tree_uuid, pause_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (!send_continue(global_tree_uuid, continue_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
      }
      if (enough_requested ()) {
        loop_flag = false;
      }
    } while (loop_flag);
      
    if (!enough_requested()) {
      ROS_ERROR("artva-filter FILTERED: Doing abort and enough.");
      do {
        if (!send_abort(global_tree_uuid, abort_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (!send_enough(global_tree_uuid, enough_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (!send_pause(global_tree_uuid, pause_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (!send_continue(global_tree_uuid, continue_id, fail_reason)) {
          fail(fail_reason);
          return;
        }
        if (burst_flag) {
          sleep (3);
        }
      } while (!enough_requested() && burst_flag);
    } else {
      ROS_ERROR("artva-filter was enoughed, doing nothing");
    }

    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail ("artva-filter ABORTED");
    return;
  }
}

