// Generated by gencpp from file crazyflie_driver/AddCrazyflie.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_ADDCRAZYFLIE_H
#define CRAZYFLIE_DRIVER_MESSAGE_ADDCRAZYFLIE_H

#include <ros/service_traits.h>


#include <crazyflie_driver/AddCrazyflieRequest.h>
#include <crazyflie_driver/AddCrazyflieResponse.h>


namespace crazyflie_driver
{

struct AddCrazyflie
{

typedef AddCrazyflieRequest Request;
typedef AddCrazyflieResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddCrazyflie
} // namespace crazyflie_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crazyflie_driver::AddCrazyflie > {
  static const char* value()
  {
    return "34565a892f53e550eb958f2ab393aeee";
  }

  static const char* value(const ::crazyflie_driver::AddCrazyflie&) { return value(); }
};

template<>
struct DataType< ::crazyflie_driver::AddCrazyflie > {
  static const char* value()
  {
    return "crazyflie_driver/AddCrazyflie";
  }

  static const char* value(const ::crazyflie_driver::AddCrazyflie&) { return value(); }
};


// service_traits::MD5Sum< ::crazyflie_driver::AddCrazyflieRequest> should match
// service_traits::MD5Sum< ::crazyflie_driver::AddCrazyflie >
template<>
struct MD5Sum< ::crazyflie_driver::AddCrazyflieRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_driver::AddCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_driver::AddCrazyflieRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_driver::AddCrazyflieRequest> should match
// service_traits::DataType< ::crazyflie_driver::AddCrazyflie >
template<>
struct DataType< ::crazyflie_driver::AddCrazyflieRequest>
{
  static const char* value()
  {
    return DataType< ::crazyflie_driver::AddCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_driver::AddCrazyflieRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crazyflie_driver::AddCrazyflieResponse> should match
// service_traits::MD5Sum< ::crazyflie_driver::AddCrazyflie >
template<>
struct MD5Sum< ::crazyflie_driver::AddCrazyflieResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_driver::AddCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_driver::AddCrazyflieResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_driver::AddCrazyflieResponse> should match
// service_traits::DataType< ::crazyflie_driver::AddCrazyflie >
template<>
struct DataType< ::crazyflie_driver::AddCrazyflieResponse>
{
  static const char* value()
  {
    return DataType< ::crazyflie_driver::AddCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_driver::AddCrazyflieResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_ADDCRAZYFLIE_H
