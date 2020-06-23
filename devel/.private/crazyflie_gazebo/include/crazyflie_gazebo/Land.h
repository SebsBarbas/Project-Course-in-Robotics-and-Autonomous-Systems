// Generated by gencpp from file crazyflie_gazebo/Land.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_GAZEBO_MESSAGE_LAND_H
#define CRAZYFLIE_GAZEBO_MESSAGE_LAND_H

#include <ros/service_traits.h>


#include <crazyflie_gazebo/LandRequest.h>
#include <crazyflie_gazebo/LandResponse.h>


namespace crazyflie_gazebo
{

struct Land
{

typedef LandRequest Request;
typedef LandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Land
} // namespace crazyflie_gazebo


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crazyflie_gazebo::Land > {
  static const char* value()
  {
    return "b665b6c83a196e4774268cc26329b159";
  }

  static const char* value(const ::crazyflie_gazebo::Land&) { return value(); }
};

template<>
struct DataType< ::crazyflie_gazebo::Land > {
  static const char* value()
  {
    return "crazyflie_gazebo/Land";
  }

  static const char* value(const ::crazyflie_gazebo::Land&) { return value(); }
};


// service_traits::MD5Sum< ::crazyflie_gazebo::LandRequest> should match
// service_traits::MD5Sum< ::crazyflie_gazebo::Land >
template<>
struct MD5Sum< ::crazyflie_gazebo::LandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_gazebo::Land >::value();
  }
  static const char* value(const ::crazyflie_gazebo::LandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_gazebo::LandRequest> should match
// service_traits::DataType< ::crazyflie_gazebo::Land >
template<>
struct DataType< ::crazyflie_gazebo::LandRequest>
{
  static const char* value()
  {
    return DataType< ::crazyflie_gazebo::Land >::value();
  }
  static const char* value(const ::crazyflie_gazebo::LandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crazyflie_gazebo::LandResponse> should match
// service_traits::MD5Sum< ::crazyflie_gazebo::Land >
template<>
struct MD5Sum< ::crazyflie_gazebo::LandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_gazebo::Land >::value();
  }
  static const char* value(const ::crazyflie_gazebo::LandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_gazebo::LandResponse> should match
// service_traits::DataType< ::crazyflie_gazebo::Land >
template<>
struct DataType< ::crazyflie_gazebo::LandResponse>
{
  static const char* value()
  {
    return DataType< ::crazyflie_gazebo::Land >::value();
  }
  static const char* value(const ::crazyflie_gazebo::LandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRAZYFLIE_GAZEBO_MESSAGE_LAND_H