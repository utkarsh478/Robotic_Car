// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_Detection_observation_count
{
public:
  explicit Init_Detection_observation_count(::my_robot_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::Detection observation_count(::my_robot_interfaces::msg::Detection::_observation_count_type arg)
  {
    msg_.observation_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::Detection msg_;
};

class Init_Detection_position_stddev
{
public:
  explicit Init_Detection_position_stddev(::my_robot_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_observation_count position_stddev(::my_robot_interfaces::msg::Detection::_position_stddev_type arg)
  {
    msg_.position_stddev = std::move(arg);
    return Init_Detection_observation_count(msg_);
  }

private:
  ::my_robot_interfaces::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::my_robot_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_position_stddev confidence(::my_robot_interfaces::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_position_stddev(msg_);
  }

private:
  ::my_robot_interfaces::msg::Detection msg_;
};

class Init_Detection_pose
{
public:
  explicit Init_Detection_pose(::my_robot_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_confidence pose(::my_robot_interfaces::msg::Detection::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::my_robot_interfaces::msg::Detection msg_;
};

class Init_Detection_label
{
public:
  explicit Init_Detection_label(::my_robot_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_pose label(::my_robot_interfaces::msg::Detection::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Detection_pose(msg_);
  }

private:
  ::my_robot_interfaces::msg::Detection msg_;
};

class Init_Detection_header
{
public:
  Init_Detection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_label header(::my_robot_interfaces::msg::Detection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Detection_label(msg_);
  }

private:
  ::my_robot_interfaces::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::Detection>()
{
  return my_robot_interfaces::msg::builder::Init_Detection_header();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
