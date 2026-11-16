// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:action/PlaceObject.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__ACTION__DETAIL__PLACE_OBJECT__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__ACTION__DETAIL__PLACE_OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/action/detail/place_object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_Goal_place_pose
{
public:
  Init_PlaceObject_Goal_place_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_robot_interfaces::action::PlaceObject_Goal place_pose(::my_robot_interfaces::action::PlaceObject_Goal::_place_pose_type arg)
  {
    msg_.place_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_Goal>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_Goal_place_pose();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_Result_message
{
public:
  explicit Init_PlaceObject_Result_message(::my_robot_interfaces::action::PlaceObject_Result & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::action::PlaceObject_Result message(::my_robot_interfaces::action::PlaceObject_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_Result msg_;
};

class Init_PlaceObject_Result_error_code
{
public:
  explicit Init_PlaceObject_Result_error_code(::my_robot_interfaces::action::PlaceObject_Result & msg)
  : msg_(msg)
  {}
  Init_PlaceObject_Result_message error_code(::my_robot_interfaces::action::PlaceObject_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_PlaceObject_Result_message(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_Result msg_;
};

class Init_PlaceObject_Result_success
{
public:
  Init_PlaceObject_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceObject_Result_error_code success(::my_robot_interfaces::action::PlaceObject_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlaceObject_Result_error_code(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_Result>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_Result_success();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_Feedback_stage
{
public:
  Init_PlaceObject_Feedback_stage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_robot_interfaces::action::PlaceObject_Feedback stage(::my_robot_interfaces::action::PlaceObject_Feedback::_stage_type arg)
  {
    msg_.stage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_Feedback>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_Feedback_stage();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_SendGoal_Request_goal
{
public:
  explicit Init_PlaceObject_SendGoal_Request_goal(::my_robot_interfaces::action::PlaceObject_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::action::PlaceObject_SendGoal_Request goal(::my_robot_interfaces::action::PlaceObject_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_SendGoal_Request msg_;
};

class Init_PlaceObject_SendGoal_Request_goal_id
{
public:
  Init_PlaceObject_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceObject_SendGoal_Request_goal goal_id(::my_robot_interfaces::action::PlaceObject_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlaceObject_SendGoal_Request_goal(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_SendGoal_Request>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_SendGoal_Request_goal_id();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_SendGoal_Response_stamp
{
public:
  explicit Init_PlaceObject_SendGoal_Response_stamp(::my_robot_interfaces::action::PlaceObject_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::action::PlaceObject_SendGoal_Response stamp(::my_robot_interfaces::action::PlaceObject_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_SendGoal_Response msg_;
};

class Init_PlaceObject_SendGoal_Response_accepted
{
public:
  Init_PlaceObject_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceObject_SendGoal_Response_stamp accepted(::my_robot_interfaces::action::PlaceObject_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PlaceObject_SendGoal_Response_stamp(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_SendGoal_Response>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_SendGoal_Response_accepted();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_GetResult_Request_goal_id
{
public:
  Init_PlaceObject_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_robot_interfaces::action::PlaceObject_GetResult_Request goal_id(::my_robot_interfaces::action::PlaceObject_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_GetResult_Request>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_GetResult_Request_goal_id();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_GetResult_Response_result
{
public:
  explicit Init_PlaceObject_GetResult_Response_result(::my_robot_interfaces::action::PlaceObject_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::action::PlaceObject_GetResult_Response result(::my_robot_interfaces::action::PlaceObject_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_GetResult_Response msg_;
};

class Init_PlaceObject_GetResult_Response_status
{
public:
  Init_PlaceObject_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceObject_GetResult_Response_result status(::my_robot_interfaces::action::PlaceObject_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PlaceObject_GetResult_Response_result(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_GetResult_Response>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_GetResult_Response_status();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace action
{

namespace builder
{

class Init_PlaceObject_FeedbackMessage_feedback
{
public:
  explicit Init_PlaceObject_FeedbackMessage_feedback(::my_robot_interfaces::action::PlaceObject_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::action::PlaceObject_FeedbackMessage feedback(::my_robot_interfaces::action::PlaceObject_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_FeedbackMessage msg_;
};

class Init_PlaceObject_FeedbackMessage_goal_id
{
public:
  Init_PlaceObject_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaceObject_FeedbackMessage_feedback goal_id(::my_robot_interfaces::action::PlaceObject_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlaceObject_FeedbackMessage_feedback(msg_);
  }

private:
  ::my_robot_interfaces::action::PlaceObject_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::action::PlaceObject_FeedbackMessage>()
{
  return my_robot_interfaces::action::builder::Init_PlaceObject_FeedbackMessage_goal_id();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__ACTION__DETAIL__PLACE_OBJECT__BUILDER_HPP_
