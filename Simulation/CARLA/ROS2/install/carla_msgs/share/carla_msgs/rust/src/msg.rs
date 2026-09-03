#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to carla_msgs__msg__CarlaEgoVehicleControl

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarlaEgoVehicleControl {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub throttle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steer: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub brake: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub hand_brake: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reverse: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gear: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub manual_gear_shift: bool,

}



impl Default for CarlaEgoVehicleControl {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CarlaEgoVehicleControl::default())
  }
}

impl rosidl_runtime_rs::Message for CarlaEgoVehicleControl {
  type RmwMsg = super::msg::rmw::CarlaEgoVehicleControl;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        throttle: msg.throttle,
        steer: msg.steer,
        brake: msg.brake,
        hand_brake: msg.hand_brake,
        reverse: msg.reverse,
        gear: msg.gear,
        manual_gear_shift: msg.manual_gear_shift,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      throttle: msg.throttle,
      steer: msg.steer,
      brake: msg.brake,
      hand_brake: msg.hand_brake,
      reverse: msg.reverse,
      gear: msg.gear,
      manual_gear_shift: msg.manual_gear_shift,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      throttle: msg.throttle,
      steer: msg.steer,
      brake: msg.brake,
      hand_brake: msg.hand_brake,
      reverse: msg.reverse,
      gear: msg.gear,
      manual_gear_shift: msg.manual_gear_shift,
    }
  }
}


