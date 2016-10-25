#include "rescue_video_recording.h"

#include <iostream>
#include <fstream>

#include "executil.h"
#include "wdbutil.h"
#include "tstutil.h"
#include "delutil.h"
#include "lrs_msgs_del/DelArgs.h"

using namespace std;

Exec::RescueVideoRecording::RescueVideoRecording (std::string ns, int id) : Executor (ns, id) {
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_enoughed = false;
  einfo.can_be_aborted = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);
  update_from_exec_info (einfo);    
}


bool Exec::RescueVideoRecording::prepare () {
  bool res = true;
  ROS_INFO ("Exec::RescueVideoRecording::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::RescueVideoRecording::start () {
  ROS_ERROR ("Exec::RescueVideoRecording::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work()) {
    return;
  }

  //  ROS_ERROR ("DelegateScanSpec doing work");

  ros::NodeHandle n;
  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  vector<string> possible_units = get_possible_units(tni.execution_ns);
  lrs_msgs_del::DelArgs delargs;
  for (auto & el : possible_units) {
    delargs.possible_units.push_back(el);
  }
  delargs.possible_units.push_back(node_ns);
  //    for (unsigned int i=0; i<delargs.possible_units.size(); i++) {
  //      ROS_ERROR("possible unit: %s", delargs.possible_units[i].c_str());
  //    }
  delargs.max_time = 3600;
  //    delargs.del_type = delargs.DELEGATION_NO_TIME_CONSTRAINTS;
  delargs.del_type = delargs.DELEGATION_FAST_EXECUTION_SCHEDULING;
  delargs.use_unit_prioritizer = false;
  
  int free_id = 1;
    
  int root_id = create_root_node (node_ns, "seq", "seq");
  if (root_id < 0) {
    fail("Failed to create root node");
    return;
  }
  set_parameter_int32(node_ns, root_id, "unique_node_id", free_id++);
  set_all_same(node_ns, root_id, "A");

  int seq_id = create_child_node (node_ns, "seq", "seq", root_id);
  if (seq_id < 0) {
    fail("Failed to create seq node");
    return;
  }
  set_parameter_int32(node_ns, seq_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, seq_id, "execunitalias", "A");
  std::vector<std::string> resnames;
  resnames.push_back("delexec17");
  set_del_exec_resources(node_ns, seq_id, resnames);


  string cid = delegate(node_ns, node_ns, root_id, delargs);

  //    ROS_ERROR("CID: %s", cid.c_str());

  bool proposed = false;
  bool finished = false;
  bool failure = false;
  int index = -1;
  lrs_msgs_del::DelLog dellog;
  while (!proposed) {
    //      cerr << "P";
    if (cnp_status(node_ns, cid, proposed, finished, failure, dellog)) {
      //        ROS_ERROR("proposed: %d finished: %d failure: %d", proposed, finished, failure);
      if (index == -1) {
	ROS_ERROR("DelLog Name: %s", dellog.name.c_str());
	index = 0;
      }
      for (;index<(int)dellog.logitems.size(); index++) {
	lrs_msgs_del::DelLogItem li = dellog.logitems[index];
	std::string listr = dellogitem_to_str(li);
	ROS_ERROR("DelLog Item %d: %s", index, listr.c_str());
      }
      if (proposed) {
	if (failure) {
	  ROS_ERROR("Delegation failed to generate a proposal");
	  wait_for_postwork_conditions ();
	  return;
	} else {
	  ROS_INFO("Delegation generated a proposal");
	}
      }
      if (finished && failure) {
	ROS_ERROR("Delegation failed with finish and failure");
	wait_for_postwork_conditions ();
	return;
      }
    } else {
      ROS_ERROR("Failed to call cnp_status");
      wait_for_postwork_conditions ();        
      return;
    }
    usleep(100000);
  }
  ROS_ERROR ("WE HAVE A PROPOSAL");

  // We have a proposal
  
  // set accept proposal
  set_accept_proposal(node_ns, cid, true);
  
  // wait for finished
  
  proposed = false;
  finished = false;
  failure = false;
  
  while (!finished) {
    if (cnp_status(node_ns, cid, proposed, finished, failure)) {
      if (finished) {
	if (failure) {
	  ROS_ERROR("Delegation failed in wait for finished");
	  wait_for_postwork_conditions ();            
	  return;
	  } else {
	  ROS_INFO("Delegation succeeded");
	}
      }
    } else {
      fail("Failed to call cnp_status");
      return;
    }
    usleep(100000);
  }

  if (!copy_del_to_exec(node_ns, root_id)) {
    fail("Failed to copy del_ns to execution_ns");
    return;
  }

  //
  // Parent id is not correct at this time so need to be fixed
  //
  
  if (!fix_tree_parent_id (node_ns, root_id)) {
    fail("Failed to fix parent id");
    return;
  }

#if 0
  std_msgs::Int32 data;
  data.data = root_id;
  display_pub.publish(data);
#endif

  string dotstr = get_tst_string(node_ns, root_id,
				 lrs_srvs_tst::TSTGetTreeString::Request::FORMAT_DOT);
  ofstream dotout("/tmp/rescue_video_recording.dot");
  dotout << dotstr << endl;
  
  ROS_ERROR("DELEGATION FINISHED - STARTING EXECUTION PHASE");

  if (!distribute_tree(node_ns, root_id)) {
    fail("Failed to distribute tree");
    return;
  }

  if (!set_tree_approved_flag(node_ns, root_id, true)) {
    fail("Failed to set approved");
    return;
  }

  if (!set_tree_start_time(node_ns, root_id, ros::Time::now())) {
    ostringstream os;
    os << "delegate_scan_spec.cc: Failed to set tree start time: " << root_id;
    fail(os.str());
    return;
  }

  lrs_msgs_tst::TSTNodeInfo ctni = get_node_info (ns, root_id);
  
  if (set_start_executor_flag(ns, root_id, true)) {
    
  } else {
    ROS_ERROR ("Exec::Sequence: Failed to set start executor flag: %d", root_id);
    fail("Exec::DelExec: Failed to create and start executor");
    return;
  }

  std::string puuid;
  if (get_global_tree_uuid_from_node(ns, tni.id, puuid)) {
    
  } else {
    fail("Failed to get global tree uuid for the parent");
    return;
  }
  
  std::string uuid;
  if (get_global_tree_uuid_from_node(ns, root_id, uuid)) {
    
  } else {
    fail("Failed to get global tree uuid");
    return;
  }
  

  if (register_executing_tree(ns, uuid, puuid)) {
    
  } else {
    fail("Failed to get register tree");
    return;
  }

  ROS_ERROR("EXEUTION STARTED");

  wait_for_postwork_conditions ();

  ROS_ERROR ("Exec::RescueVideoRecording: FINISHED");
}

bool Exec::RescueVideoRecording::abort () {
  bool res = false;
  ROS_INFO("Exec::RescueVideoRecording::abort");
  return res;
}

bool Exec::RescueVideoRecording::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}
