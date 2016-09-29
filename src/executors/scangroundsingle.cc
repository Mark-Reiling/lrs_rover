#include "scangroundsingle.h"

#include "wdbutil.h"

#include "executil.h"

#include <iostream>
#include <string>

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;

Exec::ScanGroundSingle::ScanGroundSingle (std::string ns, int id) : Executor (ns, id) {

  add_resource_to_lock("fly");
  
  // Set to true if the executor should expand the node during delegation
  set_delegation_expandable(false);

  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  einfo.can_be_enoughed = false;
  einfo.can_be_paused = false;
  set_exec_info(ns, id, einfo);

  update_from_exec_info (einfo);  
}

int Exec::ScanGroundSingle::expand (int free_id) {

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  ROS_INFO("expand: %s", ns.c_str());

  // Place code doing the expansion

  return free_id;
}

bool Exec::ScanGroundSingle::check () {
  bool res = true;
  string sensortype;

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("ScanGroundSingle check: init_params failed");
    return false;
  }

  if (!get_param("sensor-type", sensortype)) {
    ROS_ERROR ("ScanGroundSingle: Could not get sensor_type parameter");
    return false;
  } else {
    // ROS_ERROR("scangroundsingle check - sensortype: %s - %s", sensortype.c_str(), tni.execution_ns.c_str());
  }



  if (sensortype == "pointcloud") {
    if (tni.delegation_ns != "/uav0") {
      res = false;
    }
  }

  if (sensortype == "artva") {
    if (tni.delegation_ns != "/uav1") {
      res = false;
    }
  }

  if (sensortype == "camera") {
    if ((tni.delegation_ns != "/uav2") && (tni.delegation_ns != "/uav3")) {
      res = false;
    }
  }
  return res;
}


bool Exec::ScanGroundSingle::prepare () {
  bool res = true;
  ROS_INFO ("Exec::ScanGroundSingle::prepare");

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}

void Exec::ScanGroundSingle::start () {
  ROS_ERROR ("Exec::ScanGroundSingle::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;
  
  try {
    if (!do_before_work()) {
      return;
    }

    //
    // If the executor expands the node then the code below should be similar to the  
    // code for the sequence node.
    //
    // Otherwise the code below is the code doing the actual scan/mapping
    //

    
    //
    // Replace the sleeps with useful work.
    //


    //
    // We assume that this node is a non-expanding node and that the datacollection
    // and processing is done in this node.
    //

    std::vector<geographic_msgs::GeoPoint> points;
    if (!get_param("area", points)) {
      fail("Parameter 'area' do not exist or is not set");
      return;
    }

    //
    // Code to preparefor the data collection
    //
    
    string datauuid = string_params["data-uuid"].value;
    ROS_ERROR("scangroundsingle preparing for data collection for uuid: %s", datauuid.c_str());

    //
    // Do the actual scan/mapping flying
    // 

    for (unsigned int i=0; i<points.size(); i++) {
      ROS_ERROR("GeoPoint %d: %f %f %f", i,
                points[i].latitude, points[i].longitude, points[i].altitude);
    }

    //
    // To convert the points to different coordinate systems
    //

    if (tni.delegation_ns == "/uav0") {
      int duration = 120;
  
      boost::this_thread::interruption_point();
      for (int i=0; i<1000*duration; i++) {
        usleep(1000);
        boost::this_thread::interruption_point();
        
        if ((i % 10000) == 0) {
          
        }
      }
    }

    if (tni.delegation_ns == "/uav1") {
      int duration = 30;
      
      boost::this_thread::interruption_point();
      for (int i=0; i<1000*duration; i++) {
        usleep(1000);
        boost::this_thread::interruption_point();
      }
    }

    if (tni.delegation_ns == "/uav2") {
      int duration = 15;
      
      boost::this_thread::interruption_point();
      for (int i=0; i<1000*duration; i++) {
        usleep(1000);
        boost::this_thread::interruption_point();
      }
    }

    if (tni.delegation_ns == "/uav3") {
      int duration = 17;
      
      boost::this_thread::interruption_point();
      for (int i=0; i<1000*duration; i++) {
        usleep(1000);
        boost::this_thread::interruption_point();
      }
    }
    
    //
    // The flying is done. Process the data if needed.
    //
#if 0
    for (int i=0; i<5000; i++) {
      usleep(1000);
      boost::this_thread::interruption_point();
    }
#endif
    //
    // Put information in the world data base about the generated data
    //


    //
    // When we reach this point the node execution whould be finished.
    //
    
    ROS_ERROR ("Exec::ScanGroundSingle: FINISHED");
    
    wait_for_postwork_conditions ();

  }
  catch (boost::thread_interrupted) {
    abort_fail ("scangroundsingle ABORTED");
    return;
  }
}

bool Exec::ScanGroundSingle::abort () {
  bool res = false;
  ROS_INFO("Exec::ScanGroundSingle::abort");

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

