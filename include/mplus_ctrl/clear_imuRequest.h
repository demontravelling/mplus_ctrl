// Generated by gencpp from file mplus_ctrl/clear_imuRequest.msg
// DO NOT EDIT!


#ifndef MPLUS_CTRL_MESSAGE_CLEAR_IMUREQUEST_H
#define MPLUS_CTRL_MESSAGE_CLEAR_IMUREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mplus_ctrl
{
template <class ContainerAllocator>
struct clear_imuRequest_
{
  typedef clear_imuRequest_<ContainerAllocator> Type;

  clear_imuRequest_()
    : imu_clear(0)  {
    }
  clear_imuRequest_(const ContainerAllocator& _alloc)
    : imu_clear(0)  {
  (void)_alloc;
    }



   typedef int8_t _imu_clear_type;
  _imu_clear_type imu_clear;





  typedef boost::shared_ptr< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> const> ConstPtr;

}; // struct clear_imuRequest_

typedef ::mplus_ctrl::clear_imuRequest_<std::allocator<void> > clear_imuRequest;

typedef boost::shared_ptr< ::mplus_ctrl::clear_imuRequest > clear_imuRequestPtr;
typedef boost::shared_ptr< ::mplus_ctrl::clear_imuRequest const> clear_imuRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator1> & lhs, const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator2> & rhs)
{
  return lhs.imu_clear == rhs.imu_clear;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator1> & lhs, const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mplus_ctrl

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e7e981546fcf649f8695ad67c7f14a98";
  }

  static const char* value(const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe7e981546fcf649fULL;
  static const uint64_t static_value2 = 0x8695ad67c7f14a98ULL;
};

template<class ContainerAllocator>
struct DataType< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mplus_ctrl/clear_imuRequest";
  }

  static const char* value(const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 imu_clear\n"
;
  }

  static const char* value(const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.imu_clear);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct clear_imuRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mplus_ctrl::clear_imuRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mplus_ctrl::clear_imuRequest_<ContainerAllocator>& v)
  {
    s << indent << "imu_clear: ";
    Printer<int8_t>::stream(s, indent + "  ", v.imu_clear);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MPLUS_CTRL_MESSAGE_CLEAR_IMUREQUEST_H
