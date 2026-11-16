
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_Goal() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_Goal__init(msg: *mut PickObject_Goal) -> bool;
    fn my_robot_interfaces__action__PickObject_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_Goal>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_Goal>);
    fn my_robot_interfaces__action__PickObject_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_Goal>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_Goal {
    /// cylinder axis point at mid-height; frame: odom
    pub object_pose: geometry_msgs::msg::rmw::PoseStamped,

    /// 0.05
    pub object_diameter: f32,

    /// 0.15
    pub object_height: f32,

}



impl Default for PickObject_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_Goal__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_Goal() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_Result() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_Result__init(msg: *mut PickObject_Result) -> bool;
    fn my_robot_interfaces__action__PickObject_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_Result>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_Result>);
    fn my_robot_interfaces__action__PickObject_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_Result>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_code: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}

impl PickObject_Result {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NONE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNREACHABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PLANNING_FAILED: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXECUTION_FAILED: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GRASP_NOT_DETECTED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const OBJECT_LOST: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIMEOUT: u8 = 6;

}


impl Default for PickObject_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_Result__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_Result where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_Result() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_Feedback__init(msg: *mut PickObject_Feedback) -> bool;
    fn my_robot_interfaces__action__PickObject_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_Feedback>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_Feedback>);
    fn my_robot_interfaces__action__PickObject_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_Feedback>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_Feedback {
    /// "open", "pregrasp", "descend", "close", "verify", "lift", "transport"
    pub stage: rosidl_runtime_rs::String,

}



impl Default for PickObject_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_Feedback__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_Feedback() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_FeedbackMessage__init(msg: *mut PickObject_FeedbackMessage) -> bool;
    fn my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_FeedbackMessage>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_FeedbackMessage>);
    fn my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_FeedbackMessage>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::PickObject_Feedback,

}



impl Default for PickObject_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_FeedbackMessage() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_Goal() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_Goal__init(msg: *mut PlaceObject_Goal) -> bool;
    fn my_robot_interfaces__action__PlaceObject_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Goal>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Goal>);
    fn my_robot_interfaces__action__PlaceObject_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Goal>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_Goal {
    /// desired object centre (axis mid-height) when released; frame: odom
    pub place_pose: geometry_msgs::msg::rmw::PoseStamped,

}



impl Default for PlaceObject_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_Goal__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_Goal() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_Result() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_Result__init(msg: *mut PlaceObject_Result) -> bool;
    fn my_robot_interfaces__action__PlaceObject_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Result>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Result>);
    fn my_robot_interfaces__action__PlaceObject_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Result>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_code: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}

impl PlaceObject_Result {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NONE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNREACHABLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PLANNING_FAILED: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const EXECUTION_FAILED: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RELEASE_NOT_CONFIRMED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIMEOUT: u8 = 6;

}


impl Default for PlaceObject_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_Result__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_Result where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_Result() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_Feedback__init(msg: *mut PlaceObject_Feedback) -> bool;
    fn my_robot_interfaces__action__PlaceObject_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Feedback>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Feedback>);
    fn my_robot_interfaces__action__PlaceObject_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_Feedback>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_Feedback {
    /// "preplace", "descend", "release", "verify", "retract"
    pub stage: rosidl_runtime_rs::String,

}



impl Default for PlaceObject_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_Feedback__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_Feedback() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_FeedbackMessage__init(msg: *mut PlaceObject_FeedbackMessage) -> bool;
    fn my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_FeedbackMessage>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_FeedbackMessage>);
    fn my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_FeedbackMessage>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::PlaceObject_Feedback,

}



impl Default for PlaceObject_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_FeedbackMessage() }
  }
}




#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_SendGoal_Request__init(msg: *mut PickObject_SendGoal_Request) -> bool;
    fn my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_SendGoal_Request>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_SendGoal_Request>);
    fn my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_SendGoal_Request>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::PickObject_Goal,

}



impl Default for PickObject_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_SendGoal_Request() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_SendGoal_Response__init(msg: *mut PickObject_SendGoal_Response) -> bool;
    fn my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_SendGoal_Response>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_SendGoal_Response>);
    fn my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_SendGoal_Response>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for PickObject_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_SendGoal_Response() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_GetResult_Request__init(msg: *mut PickObject_GetResult_Request) -> bool;
    fn my_robot_interfaces__action__PickObject_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_GetResult_Request>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_GetResult_Request>);
    fn my_robot_interfaces__action__PickObject_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_GetResult_Request>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for PickObject_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_GetResult_Request() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PickObject_GetResult_Response__init(msg: *mut PickObject_GetResult_Response) -> bool;
    fn my_robot_interfaces__action__PickObject_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickObject_GetResult_Response>, size: usize) -> bool;
    fn my_robot_interfaces__action__PickObject_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickObject_GetResult_Response>);
    fn my_robot_interfaces__action__PickObject_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickObject_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PickObject_GetResult_Response>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PickObject_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickObject_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::PickObject_Result,

}



impl Default for PickObject_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PickObject_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PickObject_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickObject_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PickObject_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickObject_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickObject_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PickObject_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PickObject_GetResult_Response() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Request__init(msg: *mut PlaceObject_SendGoal_Request) -> bool;
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Request>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Request>);
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Request>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::PlaceObject_Goal,

}



impl Default for PlaceObject_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_SendGoal_Request() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Response__init(msg: *mut PlaceObject_SendGoal_Response) -> bool;
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Response>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Response>);
    fn my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_SendGoal_Response>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for PlaceObject_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_SendGoal_Response() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_GetResult_Request__init(msg: *mut PlaceObject_GetResult_Request) -> bool;
    fn my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Request>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Request>);
    fn my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Request>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for PlaceObject_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_GetResult_Request() }
  }
}


#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "my_robot_interfaces__rosidl_generator_c")]
extern "C" {
    fn my_robot_interfaces__action__PlaceObject_GetResult_Response__init(msg: *mut PlaceObject_GetResult_Response) -> bool;
    fn my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Response>, size: usize) -> bool;
    fn my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Response>);
    fn my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PlaceObject_GetResult_Response>) -> bool;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlaceObject_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::PlaceObject_Result,

}



impl Default for PlaceObject_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_robot_interfaces__action__PlaceObject_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to my_robot_interfaces__action__PlaceObject_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlaceObject_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlaceObject_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlaceObject_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "my_robot_interfaces/action/PlaceObject_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_robot_interfaces__action__PlaceObject_GetResult_Response() }
  }
}






#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PickObject_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to my_robot_interfaces__action__PickObject_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct PickObject_SendGoal;

impl rosidl_runtime_rs::Service for PickObject_SendGoal {
    type Request = PickObject_SendGoal_Request;
    type Response = PickObject_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PickObject_SendGoal() }
    }
}




#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PickObject_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to my_robot_interfaces__action__PickObject_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct PickObject_GetResult;

impl rosidl_runtime_rs::Service for PickObject_GetResult {
    type Request = PickObject_GetResult_Request;
    type Response = PickObject_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PickObject_GetResult() }
    }
}




#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PlaceObject_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct PlaceObject_SendGoal;

impl rosidl_runtime_rs::Service for PlaceObject_SendGoal {
    type Request = PlaceObject_SendGoal_Request;
    type Response = PlaceObject_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PlaceObject_SendGoal() }
    }
}




#[link(name = "my_robot_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PlaceObject_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to my_robot_interfaces__action__PlaceObject_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct PlaceObject_GetResult;

impl rosidl_runtime_rs::Service for PlaceObject_GetResult {
    type Request = PlaceObject_GetResult_Request;
    type Response = PlaceObject_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__my_robot_interfaces__action__PlaceObject_GetResult() }
    }
}


