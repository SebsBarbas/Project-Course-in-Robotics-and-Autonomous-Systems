// Generated by gencpp from file crazyflie_gazebo/RemoveCrazyflieRequest.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_GAZEBO_MESSAGE_REMOVECRAZYFLIEREQUEST_H
#define CRAZYFLIE_GAZEBO_MESSAGE_REMOVECRAZYFLIEREQUEST_H


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
struct RemoveCrazyflieRequest_
{
  typedef RemoveCrazyflieRequest_<ContainerAllocator> Type;

  RemoveCrazyflieRequest_()
    : uri()  {
    }
  RemoveCrazyflieRequest_(const ContainerAllocator& _alloc)
    : uri(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _uri_type;
  _uri_type uri;





  typedef boost::shared_ptr< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RemoveCrazyflieRequest_

typedef ::crazyflie_gazebo::RemoveCrazyflieRequest_<std::allocator<void> > RemoveCrazyflieRequest;

typedef boost::shared_ptr< ::crazyflie_gazebo::RemoveCrazyflieRequest > RemoveCrazyflieRequestPtr;
typedef boost::shared_ptr< ::crazyflie_gazebo::RemoveCrazyflieRequest const> RemoveCrazyflieRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator1> & lhs, const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator2> & rhs)
{
  return lhs.uri == rhs.uri;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator1> & lhs, const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace crazyflie_gazebo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "636fe5e07550f026d28388e95c38a9f4";
  }

  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x636fe5e07550f026ULL;
  static const uint64_t static_value2 = 0xd28388e95c38a9f4ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_gazebo/RemoveCrazyflieRequest";
  }

  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string uri\n"
;
  }

  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.uri);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RemoveCrazyflieRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_gazebo::RemoveCrazyflieRequest_<ContainerAllocator>& v)
  {
    s << indent << "uri: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.uri);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_GAZEBO_MESSAGE_REMOVECRAZYFLIEREQUEST_H
