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
  ROS_INFO ("Exec::RescueVideoRecording::start: %s - %d", node_ns.c_str(), node_id);

  if (!do_before_work()) {
    return;
  }

  //  ROS_ERROR ("DelegateScanSpec doing work");

  ros::NodeHandle n;
  std::string ns = ros::names::clean (ros::this_node::getNamespace());
  display_pub = n.advertise<std_msgs::Int32>("/tst_display_id", 1000);
  sleep (3);

  geographic_msgs::GeoPoint lookat_gp;
  geographic_msgs::GeoPoint video_gp1;
  geographic_msgs::GeoPoint video_gp2;
  geographic_msgs::GeoPoint charge_gp1;
  geographic_msgs::GeoPoint charge_gp2;

  if (!get_param("lookat-gp", lookat_gp)) {
    fail("Could not get parameter lookat-gp");
    return;
  }

  if (!get_param("video-gp1", video_gp1)) {
    fail("Could not get parameter video-gp1");
    return;
  }

  if (!get_param("video-gp1", video_gp1)) {
    fail("Could not get parameter video-gp2");
    return;
  }

  if (!get_param("charge-gp1", charge_gp1)) {
    fail("Could not get parameter charge-gp1");
    return;
  }

  if (!get_param("charge-gp2", charge_gp2)) {
    fail("Could not get parameter charge-gp2");
    return;
  }

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
  set_parameter_string(node_ns, root_id, "execution_ns", node_ns);
  set_all_same(node_ns, root_id, "A");

  int conc_id = create_child_node (node_ns, "conc", "conc", root_id);
  if (conc_id < 0) {
    fail("Failed to create conc node");
    return;
  }
  set_parameter_int32(node_ns, conc_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, conc_id, "execunitalias", "A");

  int seq2_id = create_child_node (node_ns, "seq", "seq", conc_id);
  if (seq2_id < 0) {
    fail("Failed to create seq node");
    return;
  }
  set_parameter_int32(node_ns, seq2_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, seq2_id, "execunitalias", "A");

  int lowbatt_id = create_child_node (node_ns, "low-battery-trigger", "low-battery-trigger", seq2_id);
  if (lowbatt_id < 0) {
    fail("Failed to create low-battery-trigger for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, lowbatt_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, lowbatt_id, "execunitalias", "A");
  set_parameter_bool(node_ns, lowbatt_id, "burst-flag", false);
  set_parameter_float64(node_ns, lowbatt_id, "limit", 10.0);

  int rvr_id = create_child_node (node_ns, "rescue-video-recording", "rescue-video-recording", seq2_id);
  if (rvr_id < 0) {
    fail("Failed to create rescuevideorecording node");
    return;
  }
  set_parameter_int32(node_ns, rvr_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, rvr_id, "execunitalias", "A");
  set_parameter_geopoint(node_ns, rvr_id, "lookat-gp", lookat_gp);
  set_parameter_geopoint(node_ns, rvr_id, "video-gp1", video_gp2);
  set_parameter_geopoint(node_ns, rvr_id, "video-gp2", video_gp1);
  set_parameter_geopoint(node_ns, rvr_id, "charge-gp1", charge_gp2);
  set_parameter_geopoint(node_ns, rvr_id, "charge-gp2", charge_gp1);


  int seq_id = create_child_node (node_ns, "seq", "seq", conc_id);
  if (seq_id < 0) {
    fail("Failed to create seq node");
    return;
  }
  set_parameter_int32(node_ns, seq_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, seq_id, "execunitalias", "A");
  std::vector<std::string> resnames;
  resnames.push_back("delexec17");
  set_del_exec_resources(node_ns, seq_id, resnames);

  int flyto_video_id = create_child_node (node_ns, "fly-to", "fly-to", seq_id);
  if (flyto_video_id < 0) {
    fail("Failed to create flyto node for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, flyto_video_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, flyto_video_id, "execunitalias", "A");
  set_parameter_geopoint(node_ns, flyto_video_id, "p", video_gp1);

  int lookat_id = create_child_node (node_ns, "look-at", "look-at", seq_id);
  if (lookat_id < 0) {
    fail("Failed to create lookat node for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, lookat_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, lookat_id, "execunitalias", "A");
  set_parameter_geopoint(node_ns, lookat_id, "p", lookat_gp);

  int start_id = create_child_node (node_ns, "start-video-recording", "start-video-recording", seq_id);
  if (start_id < 0) {
    fail("Failed to create start-video-recording for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, start_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, start_id, "execunitalias", "A");

  int always_id = create_child_node (node_ns, "always-trigger", "always-trigger", seq_id);
  if (always_id < 0) {
    fail("Failed to create always-trigger for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, always_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, always_id, "execunitalias", "A");
  set_parameter_bool(node_ns, always_id, "burst-flag", false);
  set_parameter_string(node_ns, always_id, "global-tree-uuid-to-trigger", "dummyvalue-fix-me");

  int wait_id = create_child_node (node_ns, "wait", "wait", seq_id);
  if (wait_id < 0) {
    fail("Failed to create wait for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, wait_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, wait_id, "execunitalias", "A");
  set_parameter_int32(node_ns, wait_id, "duration", -1);

  int stop_id = create_child_node (node_ns, "stop-video-recording", "stop-video-recording", seq_id);
  if (stop_id < 0) {
    fail("Failed to create stop-video-recording for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, stop_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, stop_id, "execunitalias", "A");

  int flyto_charge_id = create_child_node (node_ns, "fly-to", "fly-to", seq_id);
  if (flyto_charge_id < 0) {
    fail("Failed to create flyto node for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, flyto_charge_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, flyto_charge_id, "execunitalias", "A");
  set_parameter_geopoint(node_ns, flyto_charge_id, "p", charge_gp1);

  int start_charging_id = create_child_node (node_ns, "start-charging", "start-charging", seq_id);
  if (wait_id < 0) {
    fail("Failed to create start_charging for rescuevideorecording");
    return;
  }
  set_parameter_int32(node_ns, start_charging_id, "unique_node_id", free_id++);
  set_parameter_string(node_ns, start_charging_id, "execunitalias", "A");

  // Sett puuid

  std::string puuid;
  if (!get_param("parent-uuid", puuid)) {
    if (!get_global_tree_uuid_from_node(ns, tni.id, puuid)) {
      fail("Failed to get global tree uuid for the parent");
      return;
    }
  }

  ROS_INFO("rescue_video_recording parent-uuid: %s", puuid.c_str());
  set_parameter_string(node_ns, rvr_id, "parent-uuid", puuid);  
  
  //  std_msgs::Int32 bdata;
  //  bdata.data = root_id;
  //  display_pub.publish(bdata);
  //  sleep(3);

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
	ROS_INFO("DelLog Name: %s", dellog.name.c_str());
	index = 0;
      }
      for (;index<(int)dellog.logitems.size(); index++) {
	lrs_msgs_del::DelLogItem li = dellog.logitems[index];
	std::string listr = dellogitem_to_str(li);
	ROS_INFO("DelLog Item %d: %s", index, listr.c_str());
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
  ROS_INFO ("WE HAVE A PROPOSAL");

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

  //  std_msgs::Int32 data;
  //  data.data = root_id;
  //  display_pub.publish(data);

  string dotstr = get_tst_string(node_ns, root_id,
				 lrs_srvs_tst::TSTGetTreeString::Request::FORMAT_DOT);
  ofstream dotout("/tmp/rescue_video_recording.dot");
  dotout << dotstr << endl;
  
  ROS_INFO("DELEGATION FINISHED - STARTING EXECUTION PHASE");

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
    os << "rescue_video_recording: Failed to set tree start time: " << root_id;
    fail(os.str());
    return;
  }

  lrs_msgs_tst::TSTNodeInfo ctni = get_node_info (ns, root_id);
  
  if (set_start_executor_flag(ns, root_id, true)) {
    
  } else {
    ROS_ERROR ("rescue_video_recording: Failed to set start executor flag: %d", root_id);
    fail("rescue_video_recording: Failed to create and start executor");
    return;
  }

  std::string uuid;
  if (!get_global_tree_uuid_from_node(ns, root_id, uuid)) {
    fail("Failed to get global tree uuid");
    return;
  }

  ROS_INFO("rescue_video_recording uuid: %s", uuid.c_str());
  

  if (!register_executing_tree(ns, uuid, puuid)) {
    fail("Failed to register new executing tree");
    return;
  }

  ROS_INFO("rescue_video_recording: EXECUTION STARTED");

  wait_for_postwork_conditions ();

  ROS_INFO ("Exec::RescueVideoRecording: FINISHED");
}


bool Exec::RescueVideoRecording::get_constraints (std::vector<std::string> & cons) {
  cons.clear ();
  bool res = false;
  res = true;
  return res;
}
