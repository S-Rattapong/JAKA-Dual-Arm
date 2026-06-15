#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to jaka_msgs__srv__Move_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Move_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub has_ref: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ref_joint: Vec<f32>,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Move_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Move_Request {
  type RmwMsg = super::srv::rmw::Move_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.into(),
        has_ref: msg.has_ref,
        ref_joint: msg.ref_joint.into(),
        mvvelo: msg.mvvelo,
        mvacc: msg.mvacc,
        mvtime: msg.mvtime,
        mvradii: msg.mvradii,
        coord_mode: msg.coord_mode,
        index: msg.index,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.as_slice().into(),
      has_ref: msg.has_ref,
        ref_joint: msg.ref_joint.as_slice().into(),
      mvvelo: msg.mvvelo,
      mvacc: msg.mvacc,
      mvtime: msg.mvtime,
      mvradii: msg.mvradii,
      coord_mode: msg.coord_mode,
      index: msg.index,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: msg.pose
          .into_iter()
          .collect(),
      has_ref: msg.has_ref,
      ref_joint: msg.ref_joint
          .into_iter()
          .collect(),
      mvvelo: msg.mvvelo,
      mvacc: msg.mvacc,
      mvtime: msg.mvtime,
      mvradii: msg.mvradii,
      coord_mode: msg.coord_mode,
      index: msg.index,
    }
  }
}


// Corresponds to jaka_msgs__srv__Move_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Move_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for Move_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Move_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Move_Response {
  type RmwMsg = super::srv::rmw::Move_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__ServoMoveEnable_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMoveEnable_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub enable: bool,

}



impl Default for ServoMoveEnable_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ServoMoveEnable_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ServoMoveEnable_Request {
  type RmwMsg = super::srv::rmw::ServoMoveEnable_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        enable: msg.enable,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      enable: msg.enable,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      enable: msg.enable,
    }
  }
}


// Corresponds to jaka_msgs__srv__ServoMoveEnable_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMoveEnable_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for ServoMoveEnable_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ServoMoveEnable_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ServoMoveEnable_Response {
  type RmwMsg = super::srv::rmw::ServoMoveEnable_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__ServoMove_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMove_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed: Vec<f32>,

}



impl Default for ServoMove_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ServoMove_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ServoMove_Request {
  type RmwMsg = super::srv::rmw::ServoMove_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.into(),
        speed: msg.speed.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.as_slice().into(),
        speed: msg.speed.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: msg.pose
          .into_iter()
          .collect(),
      speed: msg.speed
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to jaka_msgs__srv__ServoMove_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ServoMove_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for ServoMove_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ServoMove_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ServoMove_Response {
  type RmwMsg = super::srv::rmw::ServoMove_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__SetTcpFrame_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTcpFrame_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tool_num: i16,

}



impl Default for SetTcpFrame_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetTcpFrame_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetTcpFrame_Request {
  type RmwMsg = super::srv::rmw::SetTcpFrame_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.into(),
        tool_num: msg.tool_num,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.as_slice().into(),
      tool_num: msg.tool_num,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: msg.pose
          .into_iter()
          .collect(),
      tool_num: msg.tool_num,
    }
  }
}


// Corresponds to jaka_msgs__srv__SetTcpFrame_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTcpFrame_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetTcpFrame_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetTcpFrame_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetTcpFrame_Response {
  type RmwMsg = super::srv::rmw::SetTcpFrame_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__SetUserFrame_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetUserFrame_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub user_num: i16,

}



impl Default for SetUserFrame_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetUserFrame_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetUserFrame_Request {
  type RmwMsg = super::srv::rmw::SetUserFrame_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.into(),
        user_num: msg.user_num,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: msg.pose.as_slice().into(),
      user_num: msg.user_num,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: msg.pose
          .into_iter()
          .collect(),
      user_num: msg.user_num,
    }
  }
}


// Corresponds to jaka_msgs__srv__SetUserFrame_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetUserFrame_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetUserFrame_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetUserFrame_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetUserFrame_Response {
  type RmwMsg = super::srv::rmw::SetUserFrame_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__SetPayload_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPayload_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetPayload_Request {
  type RmwMsg = super::srv::rmw::SetPayload_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        tool_num: msg.tool_num,
        mass: msg.mass,
        xc: msg.xc,
        yc: msg.yc,
        zc: msg.zc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      tool_num: msg.tool_num,
      mass: msg.mass,
      xc: msg.xc,
      yc: msg.yc,
      zc: msg.zc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      tool_num: msg.tool_num,
      mass: msg.mass,
      xc: msg.xc,
      yc: msg.yc,
      zc: msg.zc,
    }
  }
}


// Corresponds to jaka_msgs__srv__SetPayload_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPayload_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetPayload_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPayload_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetPayload_Response {
  type RmwMsg = super::srv::rmw::SetPayload_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__SetCollision_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCollision_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetCollision_Request {
  type RmwMsg = super::srv::rmw::SetCollision_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        is_enable: msg.is_enable,
        value: msg.value,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      is_enable: msg.is_enable,
      value: msg.value,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      is_enable: msg.is_enable,
      value: msg.value,
    }
  }
}


// Corresponds to jaka_msgs__srv__SetCollision_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCollision_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetCollision_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetCollision_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetCollision_Response {
  type RmwMsg = super::srv::rmw::SetCollision_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__SetIO_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetIO_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub signal: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetIO_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetIO_Request {
  type RmwMsg = super::srv::rmw::SetIO_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        signal: msg.signal.as_str().into(),
        type_: msg.type_,
        index: msg.index,
        value: msg.value,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        signal: msg.signal.as_str().into(),
      type_: msg.type_,
      index: msg.index,
      value: msg.value,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      signal: msg.signal.to_string(),
      type_: msg.type_,
      index: msg.index,
      value: msg.value,
    }
  }
}


// Corresponds to jaka_msgs__srv__SetIO_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetIO_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for SetIO_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetIO_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetIO_Response {
  type RmwMsg = super::srv::rmw::SetIO_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__GetIO_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIO_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub signal: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetIO_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetIO_Request {
  type RmwMsg = super::srv::rmw::GetIO_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        signal: msg.signal.as_str().into(),
        path: msg.path,
        type_: msg.type_,
        index: msg.index,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        signal: msg.signal.as_str().into(),
      path: msg.path,
      type_: msg.type_,
      index: msg.index,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      signal: msg.signal.to_string(),
      path: msg.path,
      type_: msg.type_,
      index: msg.index,
    }
  }
}


// Corresponds to jaka_msgs__srv__GetIO_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIO_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub value: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetIO_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetIO_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetIO_Response {
  type RmwMsg = super::srv::rmw::GetIO_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        value: msg.value,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      value: msg.value,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      value: msg.value,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__ClearError_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearError_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub enable: bool,

}



impl Default for ClearError_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearError_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ClearError_Request {
  type RmwMsg = super::srv::rmw::ClearError_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        enable: msg.enable,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      enable: msg.enable,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      enable: msg.enable,
    }
  }
}


// Corresponds to jaka_msgs__srv__ClearError_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClearError_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ret: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for ClearError_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ClearError_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ClearError_Response {
  type RmwMsg = super::srv::rmw::ClearError_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ret: msg.ret,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ret: msg.ret,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ret: msg.ret,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__GetFK_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetFK_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: Vec<f32>,

}



impl Default for GetFK_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetFK_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetFK_Request {
  type RmwMsg = super::srv::rmw::GetFK_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint: msg.joint.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint: msg.joint.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      joint: msg.joint
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to jaka_msgs__srv__GetFK_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetFK_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub cartesian_pose: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetFK_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetFK_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetFK_Response {
  type RmwMsg = super::srv::rmw::GetFK_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cartesian_pose: msg.cartesian_pose.into(),
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cartesian_pose: msg.cartesian_pose.as_slice().into(),
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cartesian_pose: msg.cartesian_pose
          .into_iter()
          .collect(),
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to jaka_msgs__srv__GetIK_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIK_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ref_joint: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cartesian_pose: Vec<f32>,

}



impl Default for GetIK_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetIK_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetIK_Request {
  type RmwMsg = super::srv::rmw::GetIK_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ref_joint: msg.ref_joint.into(),
        cartesian_pose: msg.cartesian_pose.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ref_joint: msg.ref_joint.as_slice().into(),
        cartesian_pose: msg.cartesian_pose.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ref_joint: msg.ref_joint
          .into_iter()
          .collect(),
      cartesian_pose: msg.cartesian_pose
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to jaka_msgs__srv__GetIK_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetIK_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for GetIK_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetIK_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetIK_Response {
  type RmwMsg = super::srv::rmw::GetIK_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint: msg.joint.into(),
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint: msg.joint.as_slice().into(),
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      joint: msg.joint
          .into_iter()
          .collect(),
      message: msg.message.to_string(),
    }
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


