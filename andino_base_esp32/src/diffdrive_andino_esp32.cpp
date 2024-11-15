// BSD 3-Clause License
//
// Copyright (c) 2024, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "andino_base_esp32/diffdrive_andino_esp32.h"

namespace andino_base_esp32 {

hardware_interface::CallbackReturn DiffDriveAndinoEsp32::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "On init...");

  config_.left_wheel_name = info_.hardware_parameters[kLeftWheelNameParam];
  config_.right_wheel_name = info_.hardware_parameters[kRightWheelNameParam];
  config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters[kEncTicksPerRevParam]);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=andino_esp32_ros2_control" + info_.name});

  node_ = rclcpp::Node::make_shared("_", options);

  set_speed_motor_ = node_->create_publisher<std_msgs::msg::Int64MultiArray>("/andino/motor_speed", rclcpp::QoS(1));

  left_wheel_.Setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
  right_wheel_.Setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

  RCLCPP_INFO(logger_, "Finished On init.");

  const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end()) {
      return it->second;
    }
    return default_value;
  };

  get_position_left_encoder_ = node_->create_subscription<std_msgs::msg::Int64>(
      get_hardware_parameter("topic", "/andino/left_position"), rclcpp::QoS(1),
      [this](const std_msgs::msg::Int64::SharedPtr position) { left_pos_ = *position; });

  get_position_right_encoder_ = node_->create_subscription<std_msgs::msg::Int64>(
      get_hardware_parameter("topic", "/andino/rigth_position"), rclcpp::QoS(1),
      [this](const std_msgs::msg::Int64::SharedPtr position) { right_pos_ = *position; });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndinoEsp32::on_configure(const rclcpp_lifecycle::State&) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveAndinoEsp32::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_POSITION, &left_wheel_.pos_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_POSITION, &right_wheel_.pos_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveAndinoEsp32::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveAndinoEsp32::on_activate(const rclcpp_lifecycle::State&) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndinoEsp32::on_deactivate(const rclcpp_lifecycle::State&) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveAndinoEsp32::read(const rclcpp::Time&, const rclcpp::Duration& period) {
  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }

  const double delta_secs = period.seconds();

  left_wheel_.enc_ = left_pos_.data;
  right_wheel_.enc_ = right_pos_.data;

  const double left_pos_prev = left_wheel_.pos_;
  left_wheel_.pos_ = left_wheel_.Angle();
  left_wheel_.vel_ = (left_wheel_.pos_ - left_pos_prev) / delta_secs;

  const double right_pos_prev = right_wheel_.pos_;
  right_wheel_.pos_ = right_wheel_.Angle();
  right_wheel_.vel_ = (right_wheel_.pos_ - right_pos_prev) / delta_secs;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveAndinoEsp32::write(const rclcpp::Time&, const rclcpp::Duration&) {
  const int left_value_target = static_cast<int>(left_wheel_.cmd_ / left_wheel_.rads_per_tick_);
  const int right_value_target = static_cast<int>(right_wheel_.cmd_ / right_wheel_.rads_per_tick_);

  motor_speed.data = {left_value_target, right_value_target};

  if (rclcpp::ok()) {
    set_speed_motor_->publish(motor_speed);
  }

  return hardware_interface::return_type::OK;
}
}  // namespace andino_base_esp32

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(andino_base_esp32::DiffDriveAndinoEsp32, hardware_interface::SystemInterface)
