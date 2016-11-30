#include "driveto.h"
#include <math.h>
#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

#include "Gpoint.h"


using namespace std;
/*
class Gpoint{
public:
  Gpoint(double a_, double b_, string type_)
  {
    set_home();
    if (type_.compare("GPS") == 0)
    {
        lat = a_;
        lng = b_;
        is_GPS_set = true;
        Cal_xy();
    }
    else if(type_.compare("local"))
    {
        x = a_;
        y = b_;
        is_local_set = true;
        Cal_GPS();
    }
    else ROS_ERROR_STREAM("The point type is" << type_ << "which does not match neither GPS nor local");

  }
  Gpoint(geographic_msgs::GeoPoint G)
  {
      set_home();
      lat = G.latitude;
      lng = G.longitude;
      is_GPS_set = true;
      Cal_xy();
  }

  ~Gpoint()
  {

  }
  void update_local(double x_,double y_)
  {
    x = x_;
    y = y_;
    Cal_GPS();
  }
  void update_GPS(double lat_,double lng_)
  {
    lat = lat_;
    lng = lng_;
    Cal_xy();
  }

  void update_home(double lat_, double lng_)
  {
    if(!(is_GPS_set||is_local_set))
    {
      ROS_ERROR("insufficient coordinate data");
      return;
    }
    lat_home = lat_;
    lng_home = lng_;
    is_home_set = true;
    Cal_R();
    Cal_GPS();
    Cal_xy();
  }
  void update_home(geographic_msgs::GeoPoint G)
  {
    if(!(is_GPS_set||is_local_set))
    {
      ROS_ERROR("insufficient coordinate data");
      return;
    }
    lat_home = G.latitude;
    lng_home = G.longitude;
    is_home_set = true;
    Cal_R();
    Cal_GPS();
    Cal_xy();
  }

private:
  double const A =  6378137;    //major semiaxis
  double const B = 6356752.3124;    //minor semiaxis
  double const e = 0.0816733743281685;// first eccentricity
  double x;
  double y;
  double lat;
  double lng;
  double lat_home;
  double lng_home;
  bool is_home_set = false;
  bool is_local_set = false;
  bool is_GPS_set = false;
  double R;

  //member functions
  void Cal_R()
  {
    R = A/sqrt(1-pow(e,2)*pow(sin(lat_home*M_PI/180.0),2));
  }

  void set_home()
  {
    lat_home = 52.240677;
    lng_home = 6.853642;
    is_home_set = true;
    Cal_R();
  }



  void Cal_GPS()
  {
    if(!is_local_set)
    {
      ROS_ERROR("local coordinate data does not exist");
      return;
    }
    lat = y/R*180.0/M_PI + lat_home;
    lng = x/R*180.0/M_PI*cos(lat*M_PI/180.0) + lng_home;

  }
  void Cal_xy()
  {
     if(!is_GPS_set)
     {
       ROS_ERROR("GPS coordinate data does not exist");
       return;
     }
     x = (lat-lat_home)*M_PI/180.00*R;
     y =-(lng - lng_home)*M_PI/180.00*R*cos(lat*M_PI/180.0);
  }

};*/


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
    Gpoint target(gp);

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

bool Exec::DriveTo::check_node(string node_) {
  bool res = false;
  vector< string > node_list;
  if(!ros::master::getNodes(node_list))
  {
    ROS_ERROR("Failed to get node list");
    return res;
  }

  for(size_t i=0; i < node_list.size();i++)
  {
    if(node_.compare(node_list[i]) == 0 )
    {
      res = true;
      //ROS_INFO("Node %s found",node_);
    }
  }
  return res;
}
