#ifndef GPOINT_H
#define GPOINT_H

#include <math.h>
#include <iostream>
#include <string>
#include <ros/ros.h>

#include<geographic_msgs/GeoPoint.h>

class Gpoint
{
  public:
    Gpoint(double a_, double b_, std::string type_);

    Gpoint(geographic_msgs::GeoPoint G);

    ~Gpoint();

    void update_local(double x_,double y_);
    void update_GPS(double lat_,double lng_);
    void update_home(double lat_, double lng_);
    void update_home(geographic_msgs::GeoPoint G);
    double getX();
    double getY();
    double getLAT();
    double getLGN();
    geographic_msgs::GeoPoint getGeoPoint();
    geographic_msgs::GeoPoint getHomeGeoPoint();


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
    void Cal_R();
    void set_home();
    void Cal_GPS();
    void Cal_xy();

};

#endif // GPOINT_H
