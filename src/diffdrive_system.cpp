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
  params.left_enable_pin = std::stoi(info_.hardware_parameters["motor_left_enable_pin"]);
  params.left_forward_pin = std::stoi(info_.hardware_parameters["motor_left_forward_pin"]);
  params.left_backward_pin = std::stoi(info_.hardware_parameters["motor_left_backward_pin"]);
  params.right_enable_pin = std::stoi(info_.hardware_parameters["motor_right_enable_pin"]);
  params.right_forward_pin = std::stoi(info_.hardware_parameters["motor_right_forward_pin"]);
  params.right_backward_pin = std::stoi(info_.hardware_parameters["motor_right_backward_pin"]);
  params.max_speed = std::stof(info_.hardware_parameters["max_motor_rotation_speed"]);

  params.left_encoder_yellow_pin = std::stoi(info_.hardware_parameters["encoder_left_yellow_pin"]);
  params.left_encoder_green_pin = std::stoi(info_.hardware_parameters["encoder_left_green_pin"]);
  params.right_encoder_yellow_pin = std::stoi(info_.hardware_parameters["encoder_right_yellow_pin"]);
  params.right_encoder_green_pin = std::stoi(info_.hardware_parameters["encoder_right_green_pin"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
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
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void DiffDriveHardware::encoder_callback(uint32_t gpio, uint32_t level, uint32_t tick)
{
  int previous, current;

  if (gpio == params.left_encoder_yellow_pin)
  {
    previous = (state.left_encoder_green_state << 1) | state.left_encoder_yellow_state;
    current = (state.left_encoder_green_state << 1) | level;
    int direction = encoder_state_to_direction_table[previous][current];
    state.left_wheel_tics += direction;
    state.left_encoder_yellow_state = level;
  }
  else if (gpio == params.left_encoder_green_pin)
  {
    previous = (state.left_encoder_green_state << 1) | state.left_encoder_yellow_state;
    current = (level << 1) | state.left_encoder_yellow_state;
    int direction = encoder_state_to_direction_table[previous][current];
    state.left_wheel_tics += direction;
    state.left_encoder_green_state = level;
  }
  else if (gpio == params.right_encoder_yellow_pin)
  {
    previous = (state.right_encoder_green_state << 1) | state.right_encoder_yellow_state;
    current = (state.right_encoder_green_state << 1) | level;
    int direction = encoder_state_to_direction_table[previous][current];
    state.right_wheel_tics += direction;
    state.right_encoder_yellow_state = level;
  }
  else if (gpio == params.right_encoder_green_pin)
  {
    previous = (state.right_encoder_green_state << 1) | state.right_encoder_yellow_state;
    current = (level << 1) | state.right_encoder_yellow_state;
    int direction = encoder_state_to_direction_table[previous][current];
    state.right_wheel_tics += direction;
    state.right_encoder_green_state = level;
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

  pigpio_deamon = pigpio_start(NULL, NULL);

  set_mode(pigpio_deamon, params.left_enable_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, params.left_forward_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, params.left_backward_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, params.right_enable_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, params.right_forward_pin, PI_OUTPUT);
  set_mode(pigpio_deamon, params.right_backward_pin, PI_OUTPUT);

  set_mode(pigpio_deamon, params.left_encoder_yellow_pin, PI_INPUT);
  set_mode(pigpio_deamon, params.left_encoder_green_pin, PI_INPUT);
  set_mode(pigpio_deamon, params.right_encoder_yellow_pin, PI_INPUT);
  set_mode(pigpio_deamon, params.right_encoder_green_pin, PI_INPUT);

  callback_ex(pigpio_deamon, params.left_encoder_yellow_pin, 2, encoder_callback, this);
  callback_ex(pigpio_deamon, params.left_encoder_green_pin, 2, encoder_callback, this);
  callback_ex(pigpio_deamon, params.right_encoder_yellow_pin, 2, encoder_callback, this);
  callback_ex(pigpio_deamon, params.right_encoder_green_pin, 2, encoder_callback, this);

  set_PWM_frequency(pigpio_deamon, params.left_enable_pin, 1000);
  set_PWM_frequency(pigpio_deamon, params.right_enable_pin, 1000);
  
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  // stop motors
  set_PWM_dutycycle(pigpio_deamon, params.left_enable_pin, 0);
  set_PWM_dutycycle(pigpio_deamon, params.right_enable_pin, 0);

  // todo set all pins to zero

  pigpio_stop(pigpio_deamon);


  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  std::cout << "setting stuff to zero" << std::endl;

  set_state("left_wheel_joint/velocity", 0.0);
  set_state("right_wheel_joint/velocity", 0.0);
  set_state("left_wheel_joint/position", 0.0);
  set_state("right_wheel_joint/position", 0.0);

  // int level = gpioRead(pin);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_diffdrive_encoder_plugin ::DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  std::cout << "writing to pins" << std::endl;

  double left_wheel_velocity  = get_command("left_wheel_joint/velocity");
  double right_wheel_velocity = get_command("right_wheel_joint/velocity");

  // recieved command cut to limits of the motor and scaled to pwm freq
  double left_motor_speed = std::max(std::min(left_wheel_velocity, params.max_speed), -params.max_speed);
  double left_duty_cycle = (left_motor_speed / (params.max_speed + 1)) * 100;
  
  double right_motor_speed = std::max(std::min(right_wheel_velocity, params.max_speed), -params.max_speed);
  double right_duty_cycle = (right_motor_speed / (params.max_speed + 1)) * 100;
  
  // direction
  if (left_duty_cycle > 0) {
    gpio_write(pigpio_deamon, params.left_forward_pin, 1);
    gpio_write(pigpio_deamon, params.left_backward_pin, 0);
  }
  else {
    gpio_write(pigpio_deamon, params.left_forward_pin, 0);
    gpio_write(pigpio_deamon, params.left_backward_pin, 1);
  }

  if (right_duty_cycle > 0) {
    gpio_write(pigpio_deamon, params.right_forward_pin, 0);
    gpio_write(pigpio_deamon, params.right_backward_pin, 1);
  }
  else {
    gpio_write(pigpio_deamon, params.right_forward_pin, 1);
    gpio_write(pigpio_deamon, params.right_backward_pin, 0);
  }

  // speed
  set_PWM_dutycycle(pigpio_deamon, params.left_enable_pin, std::abs(255));
  set_PWM_dutycycle(pigpio_deamon, params.right_enable_pin, std::abs(255));

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_diffdrive_encoder_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_diffdrive_encoder_plugin::DiffDriveHardware, hardware_interface::SystemInterface)
