/**
 * @file diffdrive_system.hpp
 * @brief ros2_control hardware interface for controlling two dc motors with encoders
 * @author Daniel Onderka (xonder05)
 * @date 08/2025
 */

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

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // will be filled with data from xacro
  struct config
  {
    uint8_t motor_left_enable_pin = -1;
    uint8_t motor_left_forward_pin = -1;
    uint8_t motor_left_backward_pin = -1;
    uint8_t motor_right_enable_pin = -1;
    uint8_t motor_right_forward_pin = -1;
    uint8_t motor_right_backward_pin = -1;
    uint16_t motor_rpm = -1;
    double motor_rad_s = -1;

    uint8_t encoder_left_green_pin = -1;
    uint8_t encoder_left_yellow_pin = -1;
    uint8_t encoder_right_green_pin = -1;
    uint8_t encoder_right_yellow_pin = -1;
    uint16_t encoder_tics_per_rotation = -1;
  };

  struct encoder_state 
  {
    uint8_t left_green_callback_id = -1;
    uint8_t left_yellow_callback_id = -1;
    uint8_t right_green_callback_id = -1;
    uint8_t right_yellow_callback_id = -1;

    uint8_t left_green_state = 0;
    uint8_t left_yellow_state = 0;
    uint8_t right_green_state = 0;
    uint8_t right_yellow_state = 0;
    
    int left_tics = 0;
    int right_tics = 0;
    
    int left_tics_old = 0;
    int right_tics_old = 0;
  };

  const double pi = 3.14159;

  // encoder values sequence 00 -> 01 -> 11 -> 10 -> 00 (or in reverse)
  // row == previous state, column == current state
  // value == direction (0 == invalid)
  const int8_t encoder_state_to_direction_table[4][4] = {
    { 0,  1, -1,  0},
    {-1,  0,  0,  1},
    { 1,  0,  0, -1},
    { 0, -1,  1,  0}
  };

  config config;
  encoder_state encoder_state;
  int pigpio_deamon;

  void encoder_callback(uint32_t gpio, uint32_t level);
  
  static void encoder_callback(int /*pi*/, uint32_t gpio, uint32_t level, uint32_t /*tick*/, void *userdata) 
  {
      auto *self = static_cast<DiffDriveHardware*>(userdata);
      self->encoder_callback(gpio, level);
  }

};

}  // namespace ros2_control_diffdrive_encoder_plugin

#endif  // ROS2_CONTROL_DIFFDRIVE_ENCODER_PLUGIN__DIFFDRIVE_SYSTEM_HPP_
