// This may look like C code, but it is really -*- C++ -*-

#ifndef _LEASHING_H
#define _LEASHING_H

#include "executor.h"

#include "geoconvertros.h"

#include <tf/transform_listener.h>
#include "geographic_msgs/GeoPose.h"
#include "lrs_msgs_common/LeashingCommand.h"

#include <string>

namespace Exec {

  class Leashing : public Executor {
  private:
    tf::TransformListener listener;
    ros::Subscriber position_sub;
    bool have_current_position;
    geographic_msgs::GeoPose current_geopose;

    ros::Subscriber command_sub;
    bool have_command;
    lrs_msgs_common::LeashingCommand command;

    int horizontal_control_mode;
    int vertical_control_mode;
    int yaw_control_mode;

    double horizontal_distance;
    double horizontal_heading;

    double horizontal_distance_vel;
    double horizontal_heading_vel;

    double distance_north;
    double distance_east;

    double distance_north_vel;
    double distance_east_vel;

    double vertical_distance;
    double vertical_distance_vel;

    double yaw;
    double yaw_vel;

  public:
    Leashing (std::string ns, int id);
    virtual ~Leashing () {};

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();

    void position_callback(const geographic_msgs::GeoPose::ConstPtr & msg);
    void command_callback(const lrs_msgs_common::LeashingCommand::ConstPtr & msg);
  };
};
#endif
