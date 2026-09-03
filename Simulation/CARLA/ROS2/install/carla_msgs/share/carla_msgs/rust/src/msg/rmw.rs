#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "carla_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__carla_msgs__msg__CarlaEgoVehicleControl() -> *const std::ffi::c_void;
}

#[link(name = "carla_msgs__rosidl_generator_c")]
extern "C" {
    fn carla_msgs__msg__CarlaEgoVehicleControl__init(msg: *mut CarlaEgoVehicleControl) -> bool;
    fn carla_msgs__msg__CarlaEgoVehicleControl__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CarlaEgoVehicleControl>, size: usize) -> bool;
    fn carla_msgs__msg__CarlaEgoVehicleControl__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CarlaEgoVehicleControl>);
    fn carla_msgs__msg__CarlaEgoVehicleControl__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CarlaEgoVehicleControl>, out_seq: *mut rosidl_runtime_rs::Sequence<CarlaEgoVehicleControl>) -> bool;
}

// Corresponds to carla_msgs__msg__CarlaEgoVehicleControl
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CarlaEgoVehicleControl {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !carla_msgs__msg__CarlaEgoVehicleControl__init(&mut msg as *mut _) {
        panic!("Call to carla_msgs__msg__CarlaEgoVehicleControl__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CarlaEgoVehicleControl {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { carla_msgs__msg__CarlaEgoVehicleControl__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { carla_msgs__msg__CarlaEgoVehicleControl__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { carla_msgs__msg__CarlaEgoVehicleControl__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CarlaEgoVehicleControl {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CarlaEgoVehicleControl where Self: Sized {
  const TYPE_NAME: &'static str = "carla_msgs/msg/CarlaEgoVehicleControl";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__carla_msgs__msg__CarlaEgoVehicleControl() }
  }
}


