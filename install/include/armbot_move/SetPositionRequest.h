// Generated by gencpp from file armbot_move/SetPositionRequest.msg
// DO NOT EDIT!


#ifndef ARMBOT_MOVE_MESSAGE_SETPOSITIONREQUEST_H
#define ARMBOT_MOVE_MESSAGE_SETPOSITIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace armbot_move
{
template <class ContainerAllocator>
struct SetPositionRequest_
{
  typedef SetPositionRequest_<ContainerAllocator> Type;

  SetPositionRequest_()
    : position()
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  SetPositionRequest_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _position_type;
  _position_type position;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::armbot_move::SetPositionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::armbot_move::SetPositionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetPositionRequest_

typedef ::armbot_move::SetPositionRequest_<std::allocator<void> > SetPositionRequest;

typedef boost::shared_ptr< ::armbot_move::SetPositionRequest > SetPositionRequestPtr;
typedef boost::shared_ptr< ::armbot_move::SetPositionRequest const> SetPositionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::armbot_move::SetPositionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::armbot_move::SetPositionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::armbot_move::SetPositionRequest_<ContainerAllocator1> & lhs, const ::armbot_move::SetPositionRequest_<ContainerAllocator2> & rhs)
{
  return lhs.position == rhs.position &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::armbot_move::SetPositionRequest_<ContainerAllocator1> & lhs, const ::armbot_move::SetPositionRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace armbot_move

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::armbot_move::SetPositionRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::armbot_move::SetPositionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::armbot_move::SetPositionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "02998e85e61b6ea8fe7e5b954b8abdfe";
  }

  static const char* value(const ::armbot_move::SetPositionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x02998e85e61b6ea8ULL;
  static const uint64_t static_value2 = 0xfe7e5b954b8abdfeULL;
};

template<class ContainerAllocator>
struct DataType< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "armbot_move/SetPositionRequest";
  }

  static const char* value(const ::armbot_move::SetPositionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string position\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::armbot_move::SetPositionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPositionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::armbot_move::SetPositionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::armbot_move::SetPositionRequest_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.position);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARMBOT_MOVE_MESSAGE_SETPOSITIONREQUEST_H