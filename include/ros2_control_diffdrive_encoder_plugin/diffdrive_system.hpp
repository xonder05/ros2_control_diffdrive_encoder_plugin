#ifndef ROS2_CONTROL_DIFFDRIVE_ENCODER_PLUGIN__DIFFDRIVE_SYSTEM_HPP_
#define ROS2_CONTROL_DIFFDRIVE_ENCODER_PLUGIN__DIFFDRIVE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <unistd.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <pigpiod_if2.h>

namespace ros2_control_diffdrive_encoder_plugin
{
class DiffDriveHardware : public hardware_interface::SystemInterface
{

struct params 
{
  uint8_t left_enable_pin = -1;
  uint8_t left_forward_pin = -1;
  uint8_t left_backward_pin = -1;
  uint8_t right_enable_pin = -1;
  uint8_t right_forward_pin = -1;
  uint8_t right_backward_pin = -1;
  
  double max_speed = -1;

  uint8_t left_encoder_green_pin = -1;
  uint8_t left_encoder_yellow_pin = -1;
  uint8_t right_encoder_green_pin = -1;
  uint8_t right_encoder_yellow_pin = -1;
};

struct encoder_state 
{
  uint8_t left_encoder_green_state = 0;
  uint8_t left_encoder_yellow_state = 0;
  uint8_t right_encoder_green_state = 0;
  uint8_t right_encoder_yellow_state = 0;
  
  int left_wheel_tics = 0;
  int right_wheel_tics = 0;

};

// encoder values sequence 00 -> 01 -> 11 -> 10 -> 00 (or in reverse)
// row == previous state, column == current state
// value == direction (0 == invalid)
int8_t encoder_state_to_direction_table[4][4] = {
    { 0,  1, -1,  0},
    {-1,  0,  0,  1},
    { 1,  0,  0, -1},
    { 0, -1,  1,  0}
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  params params;
  encoder_state state;
  int pigpio_deamon;

  void encoder_callback(uint32_t gpio, uint32_t level, uint32_t tick);
  
  static void encoder_callback(int pi, uint32_t gpio, uint32_t level, uint32_t tick, void *userdata) 
  {
      auto *self = static_cast<DiffDriveHardware*>(userdata);
      self->encoder_callback(gpio, level, tick);
  }

};

}  // namespace ros2_control_diffdrive_encoder_plugin


#endif  // ROS2_CONTROL_DIFFDRIVE_ENCODER_PLUGIN__DIFFDRIVE_SYSTEM_HPP_
