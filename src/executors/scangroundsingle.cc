#include "scangroundsingle.h"

#include "wdbutil.h"

#include "executil.h"

#include <iostream>
#include <string>

#include "lrs_msgs_common/ScanSpecCommand.h"

#include <uuid/uuid.h>

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

  if (sensortype == "IR+camera") {
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
  ROS_INFO ("Exec::ScanGroundSingle::start: %s - %d", node_ns.c_str(), node_id);

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

    uuid_t out;
    uuid_generate(out);
    char uuid[37];
    uuid_unparse(out, uuid);
    //    queue_uuid = std::string (uuid);
    queue_uuid = "uuid17";


    ros::NodeHandle n;
    ros::Publisher ssc_pub = n.advertise<lrs_msgs_common::ScanSpecCommand>("/scan_spec_command", 10);
    

    std::vector<geographic_msgs::GeoPoint> points;
    if (!get_param("area", points)) {
      fail("Parameter 'area' do not exist or is not set");
      return;
    }

    //
    // Code to preparefor the data collection
    //
    
    string datauuid = string_params["data-uuid"].value;
    ROS_INFO("scangroundsingle preparing for data collection for uuid: %s", datauuid.c_str());

    //
    // Do the actual scan/mapping flying
    // 

    for (unsigned int i=0; i<points.size(); i++) {
      ROS_INFO("GeoPoint %d: %f %f %f", i,
                points[i].latitude, points[i].longitude, points[i].altitude);
    }

    //
    // To convert the points to different coordinate systems
    //

    if (tni.delegation_ns == "/uav0") {
      int duration = 45;

      
      boost::this_thread::interruption_point();
      for (int i=0; i<1000*duration; i++) {
        usleep(1000);
        boost::this_thread::interruption_point();
        if (i == 5000) {
          std::vector<geographic_msgs::GeoPoint> area;
          area.push_back(geoconv.wtogp(0, 2, 0));
          area.push_back(geoconv.wtogp(2, 2, 0));
          area.push_back(geoconv.wtogp(2, 4, 0));
          area.push_back(geoconv.wtogp(0, 4, 0));
          push_command (ssc_pub, "artva", area);
        }
        if (i == 10000) {
          std::vector<geographic_msgs::GeoPoint> area;
          area.push_back(geoconv.wtogp(0, -2, 0));
          area.push_back(geoconv.wtogp(2, -2, 0));
          area.push_back(geoconv.wtogp(2, -4, 0));
          area.push_back(geoconv.wtogp(0, -4, 0));
          push_command (ssc_pub, "camera", area);
        }
        if (i == 15000) {
          std::vector<geographic_msgs::GeoPoint> area;
          area.push_back(geoconv.wtogp(-2, 0, 0));
          area.push_back(geoconv.wtogp(-2, 2, 0));
          area.push_back(geoconv.wtogp(-4, 2, 0));
          area.push_back(geoconv.wtogp(-4, 0, 0));
          push_command (ssc_pub, "camera", area);
        }
      }
    }

    if (tni.delegation_ns == "/uav1") {
      int duration = 35;

      boost::this_thread::interruption_point();
      for (int i=0; i<1000*duration; i++) {
        usleep(1000);
        boost::this_thread::interruption_point();
        
        if ((i % 10000) == 0) {
          
        }
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
      int duration = 10;

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
    
    ROS_INFO ("Exec::ScanGroundSingle: FINISHED");
    
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
    ROS_INFO("EXECUTOR EXISTS: Sending interrupt to running thread");
    threadmap[os.str()]->interrupt();

    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }
  return res;
}


void Exec::ScanGroundSingle::push_command (ros::Publisher & pub, std::string sensor_type,
                                           std::vector<geographic_msgs::GeoPoint> area) {
  ROS_INFO ("push scan spec command: %s", sensor_type.c_str());
  
  lrs_msgs_common::ScanSpec ss;
  uuid_t out;
  uuid_generate(out);
  char uuid[37];
  uuid_unparse(out, uuid);
  ss.uuid = std::string (uuid);
  ss.sensor_type = ss.SCAN_SPEC_SENSOR_TYPE_UNSPECIFIED;
  if (sensor_type == "artva") {
    ss.sensor_type = ss.SCAN_SPEC_SENSOR_TYPE_ARTVA;
  }
  if (sensor_type == "camera") {
    ss.sensor_type = ss.SCAN_SPEC_SENSOR_TYPE_CAMERA;
  }
  ss.area = area;

  lrs_msgs_common::ScanSpecCommand ssc;

  ssc.command = ssc.SCAN_SPEC_COMMAND_QUEUE_INSERT;
  ssc.queue_uuid = queue_uuid;
  ssc.scan_spec =ss;

  pub.publish(ssc);
  
}
