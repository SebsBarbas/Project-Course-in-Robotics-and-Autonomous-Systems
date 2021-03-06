// Generated by gencpp from file crazyflie_driver/GoTo.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_GOTO_H
#define CRAZYFLIE_DRIVER_MESSAGE_GOTO_H

#include <ros/service_traits.h>


#include <crazyflie_driver/GoToRequest.h>
#include <crazyflie_driver/GoToResponse.h>


namespace crazyflie_driver
{

struct GoTo
{

typedef GoToRequest Request;
typedef GoToResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GoTo
} // namespace crazyflie_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crazyflie_driver::GoTo > {
  static const char* value()
  {
    return "82856b48a972d6af023d961a655bcf75";
  }

  static const char* value(const ::crazyflie_driver::GoTo&) { return value(); }
};

template<>
struct DataType< ::crazyflie_driver::GoTo > {
  static const char* value()
  {
    return "crazyflie_driver/GoTo";
  }

  static const char* value(const ::crazyflie_driver::GoTo&) { return value(); }
};


// service_traits::MD5Sum< ::crazyflie_driver::GoToRequest> should match
// service_traits::MD5Sum< ::crazyflie_driver::GoTo >
template<>
struct MD5Sum< ::crazyflie_driver::GoToRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_driver::GoTo >::value();
  }
  static const char* value(const ::crazyflie_driver::GoToRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_driver::GoToRequest> should match
// service_traits::DataType< ::crazyflie_driver::GoTo >
template<>
struct DataType< ::crazyflie_driver::GoToRequest>
{
  static const char* value()
  {
    return DataType< ::crazyflie_driver::GoTo >::value();
  }
  static const char* value(const ::crazyflie_driver::GoToRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crazyflie_driver::GoToResponse> should match
// service_traits::MD5Sum< ::crazyflie_driver::GoTo >
template<>
struct MD5Sum< ::crazyflie_driver::GoToResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_driver::GoTo >::value();
  }
  static const char* value(const ::crazyflie_driver::GoToResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_driver::GoToResponse> should match
// service_traits::DataType< ::crazyflie_driver::GoTo >
template<>
struct DataType< ::crazyflie_driver::GoToResponse>
{
  static const char* value()
  {
    return DataType< ::crazyflie_driver::GoTo >::value();
  }
  static const char* value(const ::crazyflie_driver::GoToResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_GOTO_H
