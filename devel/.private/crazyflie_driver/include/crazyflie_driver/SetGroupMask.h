// Generated by gencpp from file crazyflie_driver/SetGroupMask.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_SETGROUPMASK_H
#define CRAZYFLIE_DRIVER_MESSAGE_SETGROUPMASK_H

#include <ros/service_traits.h>


#include <crazyflie_driver/SetGroupMaskRequest.h>
#include <crazyflie_driver/SetGroupMaskResponse.h>


namespace crazyflie_driver
{

struct SetGroupMask
{

typedef SetGroupMaskRequest Request;
typedef SetGroupMaskResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetGroupMask
} // namespace crazyflie_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crazyflie_driver::SetGroupMask > {
  static const char* value()
  {
    return "d44d7e9aa94d069ed5834dbd7329e1bb";
  }

  static const char* value(const ::crazyflie_driver::SetGroupMask&) { return value(); }
};

template<>
struct DataType< ::crazyflie_driver::SetGroupMask > {
  static const char* value()
  {
    return "crazyflie_driver/SetGroupMask";
  }

  static const char* value(const ::crazyflie_driver::SetGroupMask&) { return value(); }
};


// service_traits::MD5Sum< ::crazyflie_driver::SetGroupMaskRequest> should match
// service_traits::MD5Sum< ::crazyflie_driver::SetGroupMask >
template<>
struct MD5Sum< ::crazyflie_driver::SetGroupMaskRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_driver::SetGroupMask >::value();
  }
  static const char* value(const ::crazyflie_driver::SetGroupMaskRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_driver::SetGroupMaskRequest> should match
// service_traits::DataType< ::crazyflie_driver::SetGroupMask >
template<>
struct DataType< ::crazyflie_driver::SetGroupMaskRequest>
{
  static const char* value()
  {
    return DataType< ::crazyflie_driver::SetGroupMask >::value();
  }
  static const char* value(const ::crazyflie_driver::SetGroupMaskRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crazyflie_driver::SetGroupMaskResponse> should match
// service_traits::MD5Sum< ::crazyflie_driver::SetGroupMask >
template<>
struct MD5Sum< ::crazyflie_driver::SetGroupMaskResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_driver::SetGroupMask >::value();
  }
  static const char* value(const ::crazyflie_driver::SetGroupMaskResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_driver::SetGroupMaskResponse> should match
// service_traits::DataType< ::crazyflie_driver::SetGroupMask >
template<>
struct DataType< ::crazyflie_driver::SetGroupMaskResponse>
{
  static const char* value()
  {
    return DataType< ::crazyflie_driver::SetGroupMask >::value();
  }
  static const char* value(const ::crazyflie_driver::SetGroupMaskResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_SETGROUPMASK_H
