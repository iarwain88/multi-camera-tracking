/* Auto-generated by genmsg_cpp for file /home/matteom/fuerte_workspace/tutorialROSOpenCV/msg/Stringts.msg */
#ifndef TUTORIALROSOPENCV_MESSAGE_STRINGTS_H
#define TUTORIALROSOPENCV_MESSAGE_STRINGTS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace tutorialROSOpenCV
{
template <class ContainerAllocator>
struct Stringts_ {
  typedef Stringts_<ContainerAllocator> Type;

  Stringts_()
  : data()
  , stamp()
  {
  }

  Stringts_(const ContainerAllocator& _alloc)
  : data(_alloc)
  , stamp()
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _data_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  data;

  typedef ros::Time _stamp_type;
  ros::Time stamp;


  typedef boost::shared_ptr< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tutorialROSOpenCV::Stringts_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Stringts
typedef  ::tutorialROSOpenCV::Stringts_<std::allocator<void> > Stringts;

typedef boost::shared_ptr< ::tutorialROSOpenCV::Stringts> StringtsPtr;
typedef boost::shared_ptr< ::tutorialROSOpenCV::Stringts const> StringtsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::tutorialROSOpenCV::Stringts_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace tutorialROSOpenCV

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tutorialROSOpenCV::Stringts_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d1fd498755e0380859e3dd5ec7217be4";
  }

  static const char* value(const  ::tutorialROSOpenCV::Stringts_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd1fd498755e03808ULL;
  static const uint64_t static_value2 = 0x59e3dd5ec7217be4ULL;
};

template<class ContainerAllocator>
struct DataType< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tutorialROSOpenCV/Stringts";
  }

  static const char* value(const  ::tutorialROSOpenCV::Stringts_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string data\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
time stamp\n\
\n\
";
  }

  static const char* value(const  ::tutorialROSOpenCV::Stringts_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
    stream.next(m.stamp);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Stringts_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tutorialROSOpenCV::Stringts_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::tutorialROSOpenCV::Stringts_<ContainerAllocator> & v) 
  {
    s << indent << "data: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.data);
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
  }
};


} // namespace message_operations
} // namespace ros

#endif // TUTORIALROSOPENCV_MESSAGE_STRINGTS_H
