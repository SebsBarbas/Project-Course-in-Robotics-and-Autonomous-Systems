// Generated by gencpp from file crazyflie_gazebo/StartTrajectory.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_GAZEBO_MESSAGE_STARTTRAJECTORY_H
#define CRAZYFLIE_GAZEBO_MESSAGE_STARTTRAJECTORY_H

#include <ros/service_traits.h>


#include <crazyflie_gazebo/StartTrajectoryRequest.h>
#include <crazyflie_gazebo/StartTrajectoryResponse.h>


namespace crazyflie_gazebo
{

struct StartTrajectory
{

typedef StartTrajectoryRequest Request;
typedef StartTrajectoryResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StartTrajectory
} // namespace crazyflie_gazebo


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crazyflie_gazebo::StartTrajectory > {
  static const char* value()
  {
    return "74e2cf5224bc82fcc8d9c7dd3865d912";
  }

  static const char* value(const ::crazyflie_gazebo::StartTrajectory&) { return value(); }
};

template<>
struct DataType< ::crazyflie_gazebo::StartTrajectory > {
  static const char* value()
  {
    return "crazyflie_gazebo/StartTrajectory";
  }

  static const char* value(const ::crazyflie_gazebo::StartTrajectory&) { return value(); }
};


// service_traits::MD5Sum< ::crazyflie_gazebo::StartTrajectoryRequest> should match
// service_traits::MD5Sum< ::crazyflie_gazebo::StartTrajectory >
template<>
struct MD5Sum< ::crazyflie_gazebo::StartTrajectoryRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_gazebo::StartTrajectory >::value();
  }
  static const char* value(const ::crazyflie_gazebo::StartTrajectoryRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_gazebo::StartTrajectoryRequest> should match
// service_traits::DataType< ::crazyflie_gazebo::StartTrajectory >
template<>
struct DataType< ::crazyflie_gazebo::StartTrajectoryRequest>
{
  static const char* value()
  {
    return DataType< ::crazyflie_gazebo::StartTrajectory >::value();
  }
  static const char* value(const ::crazyflie_gazebo::StartTrajectoryRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crazyflie_gazebo::StartTrajectoryResponse> should match
// service_traits::MD5Sum< ::crazyflie_gazebo::StartTrajectory >
template<>
struct MD5Sum< ::crazyflie_gazebo::StartTrajectoryResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_gazebo::StartTrajectory >::value();
  }
  static const char* value(const ::crazyflie_gazebo::StartTrajectoryResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_gazebo::StartTrajectoryResponse> should match
// service_traits::DataType< ::crazyflie_gazebo::StartTrajectory >
template<>
struct DataType< ::crazyflie_gazebo::StartTrajectoryResponse>
{
  static const char* value()
  {
    return DataType< ::crazyflie_gazebo::StartTrajectory >::value();
  }
  static const char* value(const ::crazyflie_gazebo::StartTrajectoryResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRAZYFLIE_GAZEBO_MESSAGE_STARTTRAJECTORY_H
