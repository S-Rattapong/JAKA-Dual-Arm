#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to jaka_msgs__msg__RobotMsg

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotMsg {

    // This member is not documented.
    #[allow(missing_docs)]
    pub motion_state: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub power_state: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub servo_state: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub collision_state: i16,

}



impl Default for RobotMsg {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotMsg::default())
  }
}

impl rosidl_runtime_rs::Message for RobotMsg {
  type RmwMsg = super::msg::rmw::RobotMsg;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        motion_state: msg.motion_state,
        power_state: msg.power_state,
        servo_state: msg.servo_state,
        collision_state: msg.collision_state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      motion_state: msg.motion_state,
      power_state: msg.power_state,
      servo_state: msg.servo_state,
      collision_state: msg.collision_state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      motion_state: msg.motion_state,
      power_state: msg.power_state,
      servo_state: msg.servo_state,
      collision_state: msg.collision_state,
    }
  }
}


