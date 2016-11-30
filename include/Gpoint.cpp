#include "Gpoint.h"


Gpoint::Gpoint(double a_, double b_, std::string type_)
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

Gpoint::Gpoint(geographic_msgs::GeoPoint G)
{
    set_home();
    lat = G.latitude;
    lng = G.longitude;
    is_GPS_set = true;
    Cal_xy();
}

Gpoint::~Gpoint()
{

}

void Gpoint::update_local(double x_,double y_)
{
  x = x_;
  y = y_;
  Cal_GPS();
}

void Gpoint::update_GPS(double lat_,double lng_)
{
  lat = lat_;
  lng = lng_;
  Cal_xy();
}

void Gpoint::update_home(double lat_, double lng_)
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

void Gpoint::update_home(geographic_msgs::GeoPoint G)
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

double Gpoint::getX()
{
  return x;
}

double Gpoint::getY()
{
  return y;
}

double Gpoint::getLAT()
{
  return lat;
}

double Gpoint::getLGN()
{
  return lng;
}

geographic_msgs::GeoPoint Gpoint::getGeoPoint()
{
  geographic_msgs::GeoPoint msg;
  msg.latitude = lat;
  msg.longitude = lng;
  return msg;
}

geographic_msgs::GeoPoint Gpoint::getHomeGeoPoint()
{
  geographic_msgs::GeoPoint msg;
  msg.latitude = lat_home;
  msg.longitude = lng_home;
  return msg;
}

void Gpoint::Cal_R()
{
  R = A/sqrt(1-pow(e,2)*pow(sin(lat_home*M_PI/180.0),2));
}

void Gpoint::set_home()
{
  lat_home = 52.240677;
  lng_home = 6.853642;
  is_home_set = true;
  Cal_R();
}

void Gpoint::Cal_GPS()
{
  if(!is_local_set)
  {
    ROS_ERROR("local coordinate data does not exist");
    return;
  }
  lat = y/R*180.0/M_PI + lat_home;
  lng = x/R*180.0/M_PI*cos(lat*M_PI/180.0) + lng_home;

}

void Gpoint::Cal_xy()
{
   if(!is_GPS_set)
   {
     ROS_ERROR("GPS coordinate data does not exist");
     return;
   }
   x = (lat-lat_home)*M_PI/180.00*R;
   y =-(lng - lng_home)*M_PI/180.00*R*cos(lat*M_PI/180.0);
}
