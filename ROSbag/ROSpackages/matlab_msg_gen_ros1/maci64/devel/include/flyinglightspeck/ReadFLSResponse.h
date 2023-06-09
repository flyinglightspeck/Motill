// Generated by gencpp from file flyinglightspeck/ReadFLSResponse.msg
// DO NOT EDIT!


#ifndef FLYINGLIGHTSPECK_MESSAGE_READFLSRESPONSE_H
#define FLYINGLIGHTSPECK_MESSAGE_READFLSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace flyinglightspeck
{
template <class ContainerAllocator>
struct ReadFLSResponse_
{
  typedef ReadFLSResponse_<ContainerAllocator> Type;

  ReadFLSResponse_()
    : str()  {
    }
  ReadFLSResponse_(const ContainerAllocator& _alloc)
    : str(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _str_type;
  _str_type str;





  typedef boost::shared_ptr< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ReadFLSResponse_

typedef ::flyinglightspeck::ReadFLSResponse_<std::allocator<void> > ReadFLSResponse;

typedef boost::shared_ptr< ::flyinglightspeck::ReadFLSResponse > ReadFLSResponsePtr;
typedef boost::shared_ptr< ::flyinglightspeck::ReadFLSResponse const> ReadFLSResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator1> & lhs, const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator2> & rhs)
{
  return lhs.str == rhs.str;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator1> & lhs, const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace flyinglightspeck

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "994972b6e03928b2476860ce6c4c8e17";
  }

  static const char* value(const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x994972b6e03928b2ULL;
  static const uint64_t static_value2 = 0x476860ce6c4c8e17ULL;
};

template<class ContainerAllocator>
struct DataType< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "flyinglightspeck/ReadFLSResponse";
  }

  static const char* value(const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string str\n"
"\n"
;
  }

  static const char* value(const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.str);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReadFLSResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::flyinglightspeck::ReadFLSResponse_<ContainerAllocator>& v)
  {
    s << indent << "str: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.str);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FLYINGLIGHTSPECK_MESSAGE_READFLSRESPONSE_H
