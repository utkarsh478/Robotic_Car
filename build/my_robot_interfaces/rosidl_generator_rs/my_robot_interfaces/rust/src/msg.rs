#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to my_robot_interfaces__msg__Detection
/// A perceived object, fused over several observations. SPEC §7.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Detection {
    /// frame_id: map (pose expressed in map == Gazebo world, DC-9)
    pub header: std_msgs::msg::Header,

    /// "target" | "box"
    pub label: std::string::String,

    /// target: cylinder axis point at mid-height; box: centre of the box floor (inside)
    pub pose: geometry_msgs::msg::Pose,

    /// 0..1
    pub confidence: f32,

    /// m, std-dev of the fused observations
    pub position_stddev: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub observation_count: u32,

}



impl Default for Detection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Detection::default())
  }
}

impl rosidl_runtime_rs::Message for Detection {
  type RmwMsg = super::msg::rmw::Detection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        label: msg.label.as_str().into(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        confidence: msg.confidence,
        position_stddev: msg.position_stddev,
        observation_count: msg.observation_count,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        label: msg.label.as_str().into(),
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      confidence: msg.confidence,
      position_stddev: msg.position_stddev,
      observation_count: msg.observation_count,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      label: msg.label.to_string(),
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
      confidence: msg.confidence,
      position_stddev: msg.position_stddev,
      observation_count: msg.observation_count,
    }
  }
}


