/**
 * @file diffdrive_system.cpp
 * @brief ros2_control hardware interface for controlling two dc motors with encoders
 * @author Daniel Onderka (xonder05)
 * @date 08/2025
 */

#include "ros2_control_diffdrive_encoder_plugin/diffdrive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_control_diffdrive_encoder_plugin
{
hardware_interface::CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // loading parameters from xacro
  config.motor_left_enable_pin = std::stoi(info_.hardware_parameters["motor_left_enable_pin"]);
  config.motor_left_forward_pin = std::stoi(info_.hardware_parameters["motor_left_forward_pin"]);
  config.motor_left_backward_pin = std::stoi(info_.hardware_parameters["motor_left_backward_pin"]);
  config.motor_right_enable_pin = std::stoi(info_.hardware_parameters["motor_right_enable_pin"]);
  config.motor_right_forward_pin = std::stoi(info_.hardware_parameters["motor_right_forward_pin"]);
  config.motor_right_backward_pin = std::stoi(info_.hardware_parameters["motor_right_backward_pin"]);
  config.motor_rpm = std::stoi(info_.hardware_parameters["motor_rpm"]);
  config.motor_rad_s = ((double)config.motor_rpm * 2 * pi) / 60;

  config.encoder_left_green_pin = std::stoi(info_.hardware_parameters["encoder_left_green_pin"]);
  config.encoder_left_yellow_pin = std::stoi(info_.hardware_parameters["encoder_left_yellow_pin"]);
  config.encoder_right_green_pin = std::stoi(info_.hardware_parameters["encoder_right_green_pin"]);
  config.encoder_right_yellow_pin = std::stoi(info_.hardware_parameters["encoder_right_yellow_pin"]);
  config.encoder_tics_per_rotation = std::stoi(info_.hardware_parameters["encoder_tics_per_rotation"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffDriveSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu encoder_state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first encoder_state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second encoder_state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DiffDriveHardware::encoder_callback(uint32_t gpio, uint32_t level)
{
  int previous, current, direction;

  if (gpio == config.encoder_left_green_pin)
  {
    previous = (encoder_state.left_green_state << 1) | encoder_state.left_yellow_state;
    current = (level << 1) | encoder_state.left_yellow_state;
    direction = encoder_state_to_direction_table[previous][current];
    encoder_state.left_tics += direction;
    encoder_state.left_green_state = level;
  }
  else if (gpio == config.encoder_left_yellow_pin)
  {
    previous = (encoder_state.left_green_state << 1) | encoder_state.left_yellow_state;
    current = (encoder_state.left_green_state << 1) | level;
    direction = encoder_state_to_direction_table[previous][current];
    encoder_state.left_tics += direction;
    encoder_state.left_yellow_state = level;
  }
  else if (gpio == config.encoder_right_green_pin)
  {
    previous = (encoder_state.right_green_state << 1) | encoder_state.right_yellow_state;
    current = (level << 1) | encoder_state.right_yellow_state;
    direction = encoder_state_to_direction_table[previous][current];
    encoder_state.right_tics += direction;
    encoder_state.right_green_state = level;
  }
  else if (gpio == config.encoder_right_yellow_pin)
  {
    previous = (encoder_state.right_green_state << 1) | encoder_state.right_yellow_state;
    current = (encoder_state.right_green_state << 1) | level;
    direction = encoder_state_to_direction_table[previous][current];
    encoder_state.right_tics += direction;
    encoder_state.right_yellow_state = level;
  }
  else
  {
      // unknown pin number error
  }  
}

hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  // connect to gpio deamon on localhost
  pigpio_deamon = pigpio_start(NULL, NULL);
  if (pigpio_deamon < 0) {
    RCLCPP_INFO(get_logger(), "Cannot connect to gpio deamon");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // gpio direciton
  set_mode(pigpio_deamon, config.motor_left_enable_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, config.motor_left_forward_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, config.motor_left_backward_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, config.motor_right_enable_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, config.motor_right_forward_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, config.motor_right_backward_pin, PI_OUTPUT);

  set_mode(pigpio_deamon, config.encoder_left_green_pin, PI_INPUT);
  set_mode(pigpio_deamon, config.encoder_left_yellow_pin, PI_INPUT);
  set_mode(pigpio_deamon, config.encoder_right_green_pin, PI_INPUT);
  set_mode(pigpio_deamon, config.encoder_right_yellow_pin, PI_INPUT);

  // encoder tics callbacks
  encoder_state.left_green_callback_id = callback_ex(pigpio_deamon, config.encoder_left_green_pin, 2, encoder_callback, this);
  encoder_state.left_yellow_callback_id = callback_ex(pigpio_deamon, config.encoder_left_yellow_pin, 2, encoder_callback, this);
  encoder_state.right_green_callback_id = callback_ex(pigpio_deamon, config.encoder_right_green_pin, 2, encoder_callback, this);
  encoder_state.right_yellow_callback_id = callback_ex(pigpio_deamon, config.encoder_right_yellow_pin, 2, encoder_callback, this);

  // pwm speed control
  set_PWM_frequency(pigpio_deamon, config.motor_left_enable_pin, 4000);
  set_PWM_frequency(pigpio_deamon, config.motor_right_enable_pin, 4000);
  
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  // stop motors
  set_PWM_dutycycle(pigpio_deamon, config.motor_left_enable_pin, 0);
  set_PWM_dutycycle(pigpio_deamon, config.motor_right_enable_pin, 0);

  //set all pins to zero
  gpio_write(pigpio_deamon, config.motor_left_enable_pin, 0);
  gpio_write(pigpio_deamon, config.motor_left_forward_pin, 0);
  gpio_write(pigpio_deamon, config.motor_left_backward_pin, 0);
  gpio_write(pigpio_deamon, config.motor_right_enable_pin, 0);
  gpio_write(pigpio_deamon, config.motor_right_forward_pin, 0);
  gpio_write(pigpio_deamon, config.motor_right_backward_pin, 0);

  // cancel callbacks
  callback_cancel(encoder_state.left_green_callback_id);
  callback_cancel(encoder_state.left_yellow_callback_id);
  callback_cancel(encoder_state.right_green_callback_id);
  callback_cancel(encoder_state.right_yellow_callback_id);

  pigpio_stop(pigpio_deamon);

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // wheel position in rad
  double left_motor_position = ((double)encoder_state.left_tics / config.encoder_tics_per_rotation) * 2 * pi;
  double right_motor_position = ((double)encoder_state.right_tics / config.encoder_tics_per_rotation) * 2 * pi;

  set_state("left_wheel_joint/position", left_motor_position);
  set_state("right_wheel_joint/position", right_motor_position);

  // wheel speed in rad/s
  double left_motor_velocity = (((double)encoder_state.left_tics - encoder_state.left_tics_old) / config.encoder_tics_per_rotation) * ((2 * pi) / period.seconds());
  double right_motor_velocity = (((double)encoder_state.right_tics - encoder_state.right_tics_old) / config.encoder_tics_per_rotation) * ((2 * pi) / period.seconds());

  set_state("left_wheel_joint/velocity", left_motor_velocity);
  set_state("right_wheel_joint/velocity", right_motor_velocity);

  encoder_state.left_tics_old = encoder_state.left_tics;
  encoder_state.right_tics_old = encoder_state.right_tics;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_diffdrive_encoder_plugin ::DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t duty_cycle_range = 255;
  double left_motor_velocity  = get_command("left_wheel_joint/velocity");
  double right_motor_velocity = get_command("right_wheel_joint/velocity");

  // cut command velocity to limits of the motor and scale it to 8bit pwm range
  double safe_left_motor_velocity = std::max(std::min(left_motor_velocity, config.motor_rad_s), -config.motor_rad_s);
  double left_motor_duty_cycle = (safe_left_motor_velocity / (config.motor_rad_s + 1)) * duty_cycle_range;
  
  double right_motor_speed = std::max(std::min(right_motor_velocity, config.motor_rad_s), -config.motor_rad_s);
  double right_motor_duty_cycle = (right_motor_speed / config.motor_rad_s) * duty_cycle_range;
  
  // direction
  if (left_motor_duty_cycle > 0) {
    gpio_write(pigpio_deamon, config.motor_left_forward_pin, 1);
    gpio_write(pigpio_deamon, config.motor_left_backward_pin, 0);
  }
  else {
    gpio_write(pigpio_deamon, config.motor_left_forward_pin, 0);
    gpio_write(pigpio_deamon, config.motor_left_backward_pin, 1);
  }

  if (right_motor_duty_cycle > 0) {
    gpio_write(pigpio_deamon, config.motor_right_forward_pin, 0);
    gpio_write(pigpio_deamon, config.motor_right_backward_pin, 1);
  }
  else {
    gpio_write(pigpio_deamon, config.motor_right_forward_pin, 1);
    gpio_write(pigpio_deamon, config.motor_right_backward_pin, 0);
  }

  // speed
  set_PWM_dutycycle(pigpio_deamon, config.motor_left_enable_pin, std::abs(left_motor_duty_cycle));
  set_PWM_dutycycle(pigpio_deamon, config.motor_right_enable_pin, std::abs(right_motor_duty_cycle));

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_diffdrive_encoder_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_diffdrive_encoder_plugin::DiffDriveHardware, hardware_interface::SystemInterface)
