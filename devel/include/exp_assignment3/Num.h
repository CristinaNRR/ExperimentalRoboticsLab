// Generated by gencpp from file exp_assignment3/Num.msg
// DO NOT EDIT!


#ifndef EXP_ASSIGNMENT3_MESSAGE_NUM_H
#define EXP_ASSIGNMENT3_MESSAGE_NUM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace exp_assignment3
{
template <class ContainerAllocator>
struct Num_
{
  typedef Num_<ContainerAllocator> Type;

  Num_()
    : num()  {
    }
  Num_(const ContainerAllocator& _alloc)
    : num(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int64_t, typename ContainerAllocator::template rebind<int64_t>::other >  _num_type;
  _num_type num;





  typedef boost::shared_ptr< ::exp_assignment3::Num_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::exp_assignment3::Num_<ContainerAllocator> const> ConstPtr;

}; // struct Num_

typedef ::exp_assignment3::Num_<std::allocator<void> > Num;

typedef boost::shared_ptr< ::exp_assignment3::Num > NumPtr;
typedef boost::shared_ptr< ::exp_assignment3::Num const> NumConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::exp_assignment3::Num_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::exp_assignment3::Num_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace exp_assignment3

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'exp_assignment3': ['/home/cristina/new_ws/src/exp_assignment3-main/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::exp_assignment3::Num_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exp_assignment3::Num_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exp_assignment3::Num_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exp_assignment3::Num_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exp_assignment3::Num_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exp_assignment3::Num_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::exp_assignment3::Num_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fc220881caae13608159b5e38bd72534";
  }

  static const char* value(const ::exp_assignment3::Num_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfc220881caae1360ULL;
  static const uint64_t static_value2 = 0x8159b5e38bd72534ULL;
};

template<class ContainerAllocator>
struct DataType< ::exp_assignment3::Num_<ContainerAllocator> >
{
  static const char* value()
  {
    return "exp_assignment3/Num";
  }

  static const char* value(const ::exp_assignment3::Num_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::exp_assignment3::Num_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64[] num\n\
";
  }

  static const char* value(const ::exp_assignment3::Num_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::exp_assignment3::Num_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Num_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::exp_assignment3::Num_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::exp_assignment3::Num_<ContainerAllocator>& v)
  {
    s << indent << "num[]" << std::endl;
    for (size_t i = 0; i < v.num.size(); ++i)
    {
      s << indent << "  num[" << i << "]: ";
      Printer<int64_t>::stream(s, indent + "  ", v.num[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXP_ASSIGNMENT3_MESSAGE_NUM_H