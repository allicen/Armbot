// Generated by gencpp from file armbot_move/SetPosition.msg
// DO NOT EDIT!


#ifndef ARMBOT_MOVE_MESSAGE_SETPOSITION_H
#define ARMBOT_MOVE_MESSAGE_SETPOSITION_H

#include <ros/service_traits.h>


#include <armbot_move/SetPositionRequest.h>
#include <armbot_move/SetPositionResponse.h>


namespace armbot_move
{

struct SetPosition
{

typedef SetPositionRequest Request;
typedef SetPositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetPosition
} // namespace armbot_move


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::armbot_move::SetPosition > {
  static const char* value()
  {
    return "44ff66b996c0035936f8e48c32854dc3";
  }

  static const char* value(const ::armbot_move::SetPosition&) { return value(); }
};

template<>
struct DataType< ::armbot_move::SetPosition > {
  static const char* value()
  {
    return "armbot_move/SetPosition";
  }

  static const char* value(const ::armbot_move::SetPosition&) { return value(); }
};


// service_traits::MD5Sum< ::armbot_move::SetPositionRequest> should match
// service_traits::MD5Sum< ::armbot_move::SetPosition >
template<>
struct MD5Sum< ::armbot_move::SetPositionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::armbot_move::SetPosition >::value();
  }
  static const char* value(const ::armbot_move::SetPositionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::armbot_move::SetPositionRequest> should match
// service_traits::DataType< ::armbot_move::SetPosition >
template<>
struct DataType< ::armbot_move::SetPositionRequest>
{
  static const char* value()
  {
    return DataType< ::armbot_move::SetPosition >::value();
  }
  static const char* value(const ::armbot_move::SetPositionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::armbot_move::SetPositionResponse> should match
// service_traits::MD5Sum< ::armbot_move::SetPosition >
template<>
struct MD5Sum< ::armbot_move::SetPositionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::armbot_move::SetPosition >::value();
  }
  static const char* value(const ::armbot_move::SetPositionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::armbot_move::SetPositionResponse> should match
// service_traits::DataType< ::armbot_move::SetPosition >
template<>
struct DataType< ::armbot_move::SetPositionResponse>
{
  static const char* value()
  {
    return DataType< ::armbot_move::SetPosition >::value();
  }
  static const char* value(const ::armbot_move::SetPositionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ARMBOT_MOVE_MESSAGE_SETPOSITION_H
