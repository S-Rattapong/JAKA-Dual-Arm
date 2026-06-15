#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__msg__RobotMsg() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__msg__RobotMsg__init(msg: *mut RobotMsg) -> bool;
    fn jaka_msgs__msg__RobotMsg__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotMsg>, size: usize) -> bool;
    fn jaka_msgs__msg__RobotMsg__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotMsg>);
    fn jaka_msgs__msg__RobotMsg__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotMsg>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotMsg>) -> bool;
}

// Corresponds to jaka_msgs__msg__RobotMsg
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__msg__RobotMsg__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__msg__RobotMsg__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotMsg {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__msg__RobotMsg__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__msg__RobotMsg__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__msg__RobotMsg__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotMsg {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotMsg where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/msg/RobotMsg";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__msg__RobotMsg() }
  }
}


