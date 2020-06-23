// Generated by gencpp from file crazyflie_gazebo/StopRequest.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_GAZEBO_MESSAGE_STOPREQUEST_H
#define CRAZYFLIE_GAZEBO_MESSAGE_STOPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace crazyflie_gazebo
{
template <class ContainerAllocator>
struct StopRequest_
{
  typedef StopRequest_<ContainerAllocator> Type;

  StopRequest_()
    : groupMask(0)  {
    }
  StopRequest_(const ContainerAllocator& _alloc)
    : groupMask(0)  {
  (void)_alloc;
    }



   typedef uint8_t _groupMask_type;
  _groupMask_type groupMask;





  typedef boost::shared_ptr< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StopRequest_

typedef ::crazyflie_gazebo::StopRequest_<std::allocator<void> > StopRequest;

typedef boost::shared_ptr< ::crazyflie_gazebo::StopRequest > StopRequestPtr;
typedef boost::shared_ptr< ::crazyflie_gazebo::StopRequest const> StopRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_gazebo::StopRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::crazyflie_gazebo::StopRequest_<ContainerAllocator1> & lhs, const ::crazyflie_gazebo::StopRequest_<ContainerAllocator2> & rhs)
{
  return lhs.groupMask == rhs.groupMask;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::crazyflie_gazebo::StopRequest_<ContainerAllocator1> & lhs, const ::crazyflie_gazebo::StopRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace crazyflie_gazebo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d44d7e9aa94d069ed5834dbd7329e1bb";
  }

  static const char* value(const ::crazyflie_gazebo::StopRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd44d7e9aa94d069eULL;
  static const uint64_t static_value2 = 0xd5834dbd7329e1bbULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_gazebo/StopRequest";
  }

  static const char* value(const ::crazyflie_gazebo::StopRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 groupMask\n"
;
  }

  static const char* value(const ::crazyflie_gazebo::StopRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.groupMask);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StopRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_gazebo::StopRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_gazebo::StopRequest_<ContainerAllocator>& v)
  {
    s << indent << "groupMask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.groupMask);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_GAZEBO_MESSAGE_STOPREQUEST_H