#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__Move_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__Move_Request__init(msg: *mut Move_Request) -> bool;
    fn jaka_msgs__srv__Move_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Move_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__Move_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Move_Request>);
    fn jaka_msgs__srv__Move_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Move_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Move_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__Move_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Move_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub has_ref: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ref_joint: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mvvelo: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mvacc: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mvtime: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mvradii: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coord_mode: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub index: i16,

}



impl Default for Move_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__Move_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__Move_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Move_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__Move_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__Move_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__Move_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Move_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Move_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/Move_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__Move_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__Move_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__Move_Response__init(msg: *mut Move_Response) -> bool;
    fn jaka_msgs__srv__Move_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Move_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__Move_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Move_Response>);
    fn jaka_msgs__srv__Move_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Move_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Move_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__Move_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Move_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for Move_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__Move_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__Move_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Move_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__Move_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__Move_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__Move_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Move_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Move_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/Move_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__Move_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMoveEnable_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__ServoMoveEnable_Request__init(msg: *mut ServoMoveEnable_Request) -> bool;
    fn jaka_msgs__srv__ServoMoveEnable_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ServoMoveEnable_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__ServoMoveEnable_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ServoMoveEnable_Request>);
    fn jaka_msgs__srv__ServoMoveEnable_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ServoMoveEnable_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ServoMoveEnable_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__ServoMoveEnable_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMoveEnable_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub enable: bool,

}



impl Default for ServoMoveEnable_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__ServoMoveEnable_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__ServoMoveEnable_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ServoMoveEnable_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMoveEnable_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMoveEnable_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMoveEnable_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ServoMoveEnable_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ServoMoveEnable_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/ServoMoveEnable_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMoveEnable_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMoveEnable_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__ServoMoveEnable_Response__init(msg: *mut ServoMoveEnable_Response) -> bool;
    fn jaka_msgs__srv__ServoMoveEnable_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ServoMoveEnable_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__ServoMoveEnable_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ServoMoveEnable_Response>);
    fn jaka_msgs__srv__ServoMoveEnable_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ServoMoveEnable_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ServoMoveEnable_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__ServoMoveEnable_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMoveEnable_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for ServoMoveEnable_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__ServoMoveEnable_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__ServoMoveEnable_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ServoMoveEnable_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMoveEnable_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMoveEnable_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMoveEnable_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ServoMoveEnable_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ServoMoveEnable_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/ServoMoveEnable_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMoveEnable_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMove_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__ServoMove_Request__init(msg: *mut ServoMove_Request) -> bool;
    fn jaka_msgs__srv__ServoMove_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ServoMove_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__ServoMove_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ServoMove_Request>);
    fn jaka_msgs__srv__ServoMove_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ServoMove_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ServoMove_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__ServoMove_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMove_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for ServoMove_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__ServoMove_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__ServoMove_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ServoMove_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMove_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMove_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMove_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ServoMove_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ServoMove_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/ServoMove_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMove_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMove_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__ServoMove_Response__init(msg: *mut ServoMove_Response) -> bool;
    fn jaka_msgs__srv__ServoMove_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ServoMove_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__ServoMove_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ServoMove_Response>);
    fn jaka_msgs__srv__ServoMove_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ServoMove_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ServoMove_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__ServoMove_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMove_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for ServoMove_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__ServoMove_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__ServoMove_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ServoMove_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMove_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMove_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ServoMove_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ServoMove_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ServoMove_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/ServoMove_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ServoMove_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetTcpFrame_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetTcpFrame_Request__init(msg: *mut SetTcpFrame_Request) -> bool;
    fn jaka_msgs__srv__SetTcpFrame_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetTcpFrame_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__SetTcpFrame_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetTcpFrame_Request>);
    fn jaka_msgs__srv__SetTcpFrame_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetTcpFrame_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetTcpFrame_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetTcpFrame_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTcpFrame_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tool_num: i16,

}



impl Default for SetTcpFrame_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetTcpFrame_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetTcpFrame_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetTcpFrame_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetTcpFrame_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetTcpFrame_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetTcpFrame_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetTcpFrame_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetTcpFrame_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetTcpFrame_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetTcpFrame_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetTcpFrame_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetTcpFrame_Response__init(msg: *mut SetTcpFrame_Response) -> bool;
    fn jaka_msgs__srv__SetTcpFrame_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetTcpFrame_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__SetTcpFrame_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetTcpFrame_Response>);
    fn jaka_msgs__srv__SetTcpFrame_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetTcpFrame_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetTcpFrame_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetTcpFrame_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTcpFrame_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetTcpFrame_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetTcpFrame_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetTcpFrame_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetTcpFrame_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetTcpFrame_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetTcpFrame_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetTcpFrame_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetTcpFrame_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetTcpFrame_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetTcpFrame_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetTcpFrame_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetUserFrame_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetUserFrame_Request__init(msg: *mut SetUserFrame_Request) -> bool;
    fn jaka_msgs__srv__SetUserFrame_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetUserFrame_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__SetUserFrame_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetUserFrame_Request>);
    fn jaka_msgs__srv__SetUserFrame_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetUserFrame_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetUserFrame_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetUserFrame_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetUserFrame_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub user_num: i16,

}



impl Default for SetUserFrame_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetUserFrame_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetUserFrame_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetUserFrame_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetUserFrame_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetUserFrame_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetUserFrame_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetUserFrame_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetUserFrame_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetUserFrame_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetUserFrame_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetUserFrame_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetUserFrame_Response__init(msg: *mut SetUserFrame_Response) -> bool;
    fn jaka_msgs__srv__SetUserFrame_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetUserFrame_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__SetUserFrame_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetUserFrame_Response>);
    fn jaka_msgs__srv__SetUserFrame_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetUserFrame_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetUserFrame_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetUserFrame_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetUserFrame_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetUserFrame_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetUserFrame_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetUserFrame_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetUserFrame_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetUserFrame_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetUserFrame_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetUserFrame_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetUserFrame_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetUserFrame_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetUserFrame_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetUserFrame_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetPayload_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetPayload_Request__init(msg: *mut SetPayload_Request) -> bool;
    fn jaka_msgs__srv__SetPayload_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPayload_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__SetPayload_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPayload_Request>);
    fn jaka_msgs__srv__SetPayload_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPayload_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPayload_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetPayload_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPayload_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub tool_num: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mass: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub xc: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yc: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub zc: f32,

}



impl Default for SetPayload_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetPayload_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetPayload_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPayload_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetPayload_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetPayload_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetPayload_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPayload_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPayload_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetPayload_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetPayload_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetPayload_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetPayload_Response__init(msg: *mut SetPayload_Response) -> bool;
    fn jaka_msgs__srv__SetPayload_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPayload_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__SetPayload_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPayload_Response>);
    fn jaka_msgs__srv__SetPayload_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPayload_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPayload_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetPayload_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPayload_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetPayload_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetPayload_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetPayload_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPayload_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetPayload_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetPayload_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetPayload_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPayload_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPayload_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetPayload_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetPayload_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetCollision_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetCollision_Request__init(msg: *mut SetCollision_Request) -> bool;
    fn jaka_msgs__srv__SetCollision_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCollision_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__SetCollision_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCollision_Request>);
    fn jaka_msgs__srv__SetCollision_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCollision_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCollision_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetCollision_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCollision_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub is_enable: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub value: i16,

}



impl Default for SetCollision_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetCollision_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetCollision_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCollision_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetCollision_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetCollision_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetCollision_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCollision_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCollision_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetCollision_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetCollision_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetCollision_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetCollision_Response__init(msg: *mut SetCollision_Response) -> bool;
    fn jaka_msgs__srv__SetCollision_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCollision_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__SetCollision_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCollision_Response>);
    fn jaka_msgs__srv__SetCollision_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCollision_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCollision_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetCollision_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCollision_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetCollision_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetCollision_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetCollision_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCollision_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetCollision_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetCollision_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetCollision_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCollision_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCollision_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetCollision_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetCollision_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetIO_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetIO_Request__init(msg: *mut SetIO_Request) -> bool;
    fn jaka_msgs__srv__SetIO_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetIO_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__SetIO_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetIO_Request>);
    fn jaka_msgs__srv__SetIO_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetIO_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetIO_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetIO_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetIO_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub signal: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub index: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub value: f32,

}



impl Default for SetIO_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetIO_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetIO_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetIO_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetIO_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetIO_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetIO_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetIO_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetIO_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetIO_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetIO_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetIO_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__SetIO_Response__init(msg: *mut SetIO_Response) -> bool;
    fn jaka_msgs__srv__SetIO_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetIO_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__SetIO_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetIO_Response>);
    fn jaka_msgs__srv__SetIO_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetIO_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetIO_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__SetIO_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetIO_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetIO_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__SetIO_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__SetIO_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetIO_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetIO_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetIO_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__SetIO_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetIO_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetIO_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/SetIO_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__SetIO_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIO_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__GetIO_Request__init(msg: *mut GetIO_Request) -> bool;
    fn jaka_msgs__srv__GetIO_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetIO_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__GetIO_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetIO_Request>);
    fn jaka_msgs__srv__GetIO_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetIO_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetIO_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__GetIO_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIO_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub signal: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub index: i16,

}



impl Default for GetIO_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__GetIO_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__GetIO_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetIO_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIO_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIO_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIO_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetIO_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetIO_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/GetIO_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIO_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIO_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__GetIO_Response__init(msg: *mut GetIO_Response) -> bool;
    fn jaka_msgs__srv__GetIO_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetIO_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__GetIO_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetIO_Response>);
    fn jaka_msgs__srv__GetIO_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetIO_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetIO_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__GetIO_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIO_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub value: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetIO_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__GetIO_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__GetIO_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetIO_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIO_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIO_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIO_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetIO_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetIO_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/GetIO_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIO_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ClearError_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__ClearError_Request__init(msg: *mut ClearError_Request) -> bool;
    fn jaka_msgs__srv__ClearError_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearError_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__ClearError_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearError_Request>);
    fn jaka_msgs__srv__ClearError_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearError_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearError_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__ClearError_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearError_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub enable: bool,

}



impl Default for ClearError_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__ClearError_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__ClearError_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearError_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ClearError_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ClearError_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ClearError_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearError_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearError_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/ClearError_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ClearError_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ClearError_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__ClearError_Response__init(msg: *mut ClearError_Response) -> bool;
    fn jaka_msgs__srv__ClearError_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClearError_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__ClearError_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClearError_Response>);
    fn jaka_msgs__srv__ClearError_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClearError_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ClearError_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__ClearError_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearError_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for ClearError_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__ClearError_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__ClearError_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClearError_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ClearError_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ClearError_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__ClearError_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClearError_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClearError_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/ClearError_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__ClearError_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetFK_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__GetFK_Request__init(msg: *mut GetFK_Request) -> bool;
    fn jaka_msgs__srv__GetFK_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetFK_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__GetFK_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetFK_Request>);
    fn jaka_msgs__srv__GetFK_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetFK_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetFK_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__GetFK_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetFK_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for GetFK_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__GetFK_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__GetFK_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetFK_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetFK_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetFK_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetFK_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetFK_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetFK_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/GetFK_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetFK_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetFK_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__GetFK_Response__init(msg: *mut GetFK_Response) -> bool;
    fn jaka_msgs__srv__GetFK_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetFK_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__GetFK_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetFK_Response>);
    fn jaka_msgs__srv__GetFK_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetFK_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetFK_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__GetFK_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetFK_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub cartesian_pose: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetFK_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__GetFK_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__GetFK_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetFK_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetFK_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetFK_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetFK_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetFK_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetFK_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/GetFK_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetFK_Response() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIK_Request() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__GetIK_Request__init(msg: *mut GetIK_Request) -> bool;
    fn jaka_msgs__srv__GetIK_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetIK_Request>, size: usize) -> bool;
    fn jaka_msgs__srv__GetIK_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetIK_Request>);
    fn jaka_msgs__srv__GetIK_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetIK_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetIK_Request>) -> bool;
}

// Corresponds to jaka_msgs__srv__GetIK_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIK_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ref_joint: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cartesian_pose: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for GetIK_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__GetIK_Request__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__GetIK_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetIK_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIK_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIK_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIK_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetIK_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetIK_Request where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/GetIK_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIK_Request() }
  }
}


#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIK_Response() -> *const std::ffi::c_void;
}

#[link(name = "jaka_msgs__rosidl_generator_c")]
extern "C" {
    fn jaka_msgs__srv__GetIK_Response__init(msg: *mut GetIK_Response) -> bool;
    fn jaka_msgs__srv__GetIK_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetIK_Response>, size: usize) -> bool;
    fn jaka_msgs__srv__GetIK_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetIK_Response>);
    fn jaka_msgs__srv__GetIK_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetIK_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetIK_Response>) -> bool;
}

// Corresponds to jaka_msgs__srv__GetIK_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIK_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetIK_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jaka_msgs__srv__GetIK_Response__init(&mut msg as *mut _) {
        panic!("Call to jaka_msgs__srv__GetIK_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetIK_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIK_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIK_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jaka_msgs__srv__GetIK_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetIK_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetIK_Response where Self: Sized {
  const TYPE_NAME: &'static str = "jaka_msgs/srv/GetIK_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jaka_msgs__srv__GetIK_Response() }
  }
}






#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__Move() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__Move
#[allow(missing_docs, non_camel_case_types)]
pub struct Move;

impl rosidl_runtime_rs::Service for Move {
    type Request = Move_Request;
    type Response = Move_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__Move() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__ServoMoveEnable() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__ServoMoveEnable
#[allow(missing_docs, non_camel_case_types)]
pub struct ServoMoveEnable;

impl rosidl_runtime_rs::Service for ServoMoveEnable {
    type Request = ServoMoveEnable_Request;
    type Response = ServoMoveEnable_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__ServoMoveEnable() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__ServoMove() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__ServoMove
#[allow(missing_docs, non_camel_case_types)]
pub struct ServoMove;

impl rosidl_runtime_rs::Service for ServoMove {
    type Request = ServoMove_Request;
    type Response = ServoMove_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__ServoMove() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetTcpFrame() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__SetTcpFrame
#[allow(missing_docs, non_camel_case_types)]
pub struct SetTcpFrame;

impl rosidl_runtime_rs::Service for SetTcpFrame {
    type Request = SetTcpFrame_Request;
    type Response = SetTcpFrame_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetTcpFrame() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetUserFrame() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__SetUserFrame
#[allow(missing_docs, non_camel_case_types)]
pub struct SetUserFrame;

impl rosidl_runtime_rs::Service for SetUserFrame {
    type Request = SetUserFrame_Request;
    type Response = SetUserFrame_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetUserFrame() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetPayload() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__SetPayload
#[allow(missing_docs, non_camel_case_types)]
pub struct SetPayload;

impl rosidl_runtime_rs::Service for SetPayload {
    type Request = SetPayload_Request;
    type Response = SetPayload_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetPayload() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetCollision() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__SetCollision
#[allow(missing_docs, non_camel_case_types)]
pub struct SetCollision;

impl rosidl_runtime_rs::Service for SetCollision {
    type Request = SetCollision_Request;
    type Response = SetCollision_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetCollision() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetIO() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__SetIO
#[allow(missing_docs, non_camel_case_types)]
pub struct SetIO;

impl rosidl_runtime_rs::Service for SetIO {
    type Request = SetIO_Request;
    type Response = SetIO_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__SetIO() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__GetIO() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__GetIO
#[allow(missing_docs, non_camel_case_types)]
pub struct GetIO;

impl rosidl_runtime_rs::Service for GetIO {
    type Request = GetIO_Request;
    type Response = GetIO_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__GetIO() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__ClearError() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__ClearError
#[allow(missing_docs, non_camel_case_types)]
pub struct ClearError;

impl rosidl_runtime_rs::Service for ClearError {
    type Request = ClearError_Request;
    type Response = ClearError_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__ClearError() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__GetFK() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__GetFK
#[allow(missing_docs, non_camel_case_types)]
pub struct GetFK;

impl rosidl_runtime_rs::Service for GetFK {
    type Request = GetFK_Request;
    type Response = GetFK_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__GetFK() }
    }
}




#[link(name = "jaka_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__GetIK() -> *const std::ffi::c_void;
}

// Corresponds to jaka_msgs__srv__GetIK
#[allow(missing_docs, non_camel_case_types)]
pub struct GetIK;

impl rosidl_runtime_rs::Service for GetIK {
    type Request = GetIK_Request;
    type Response = GetIK_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__jaka_msgs__srv__GetIK() }
    }
}


