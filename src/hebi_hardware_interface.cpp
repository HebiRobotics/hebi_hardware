// Copyright (c) 2023, HEBI Robotics Inc.
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "hebi_hardware/hebi_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define COUT_INFO "[hebi_hardware] [INFO] "
#define COUT_WARN "[hebi_hardware] [WARN] "
#define COUT_ERROR "[hebi_hardware] [ERROR] "

namespace arm = hebi::experimental::arm;

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    result.push_back(item);
  }
  return result;
}


namespace hebi_hardware {

hardware_interface::CallbackReturn HEBIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Read parameters
  for (auto i = info_.hardware_parameters.begin(); i != info_.hardware_parameters.end(); i++) {
    if (i->first == "config_pkg") config_pkg_ = i->second;
    if (i->first == "config_file") config_file_ = i->second;
  }

  if (config_pkg_.empty() || config_file_.empty()) {
    std::cout << COUT_ERROR << "Config package and/or file not provided! Aborting." << std::endl;
    return CallbackReturn::ERROR;
  }

  std::cout << COUT_INFO << "Config package: "<< config_pkg_ << std::endl;
  std::cout << COUT_INFO << "Config file: "<< config_file_ << std::endl;

  joint_pos_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_vel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_acc_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HEBIHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_acc_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HEBIHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size() * 2);
  position_command_interface_names_.reserve(info_.joints.size());
  velocity_command_interface_names_.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_pos_commands_[i]));
    position_command_interface_names_.push_back(command_interfaces.back().get_name());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_vel_commands_[i]));
    velocity_command_interface_names_.push_back(command_interfaces.back().get_name());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Resolve the config file path using the package share directory
  const std::string config_file_path = ament_index_cpp::get_package_share_directory(config_pkg_) + "/" + config_file_;

  // Load the config
  std::vector<std::string> errors;
  arm_config_ = hebi::RobotConfig::loadConfig(config_file_path, errors);
  for (const auto& error : errors) {
    std::cout << COUT_ERROR << error.c_str();
  }
  if (!arm_config_) {
    std::cout << COUT_ERROR << "Failed to load configuration from: " << config_file_path.c_str() << std::endl;
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

  // Create arm from config
  arm_ = arm::Arm::create(*arm_config_);

  // Terminate if arm not found
  if (!arm_) {
    std::cout << COUT_ERROR << "Failed to create arm!" << std::endl;
    return CallbackReturn::ERROR;
  }
  std::cout << COUT_INFO << "Arm connected." << std::endl;

  // Check if arm size is same as number of joints
  if (arm_->size() != info_.joints.size()) {
    std::cout << COUT_ERROR << "Number of joints in config file does not match number of joints in URDF!" << std::endl;
    return CallbackReturn::ERROR;
  }

  arm_->update();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // prepare the robot to stop receiving commands
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
  // prepare the robot to stop receiving commands
  if (arm_) {
    arm_.release();
    std::cout << COUT_INFO << "Arm object released during shutdown." << std::endl;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HEBIHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // read robot states

  if (!arm_) {
    std::cout << COUT_ERROR << "Arm object is not initialized!" << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  if (!arm_->update()) {
    std::cout << COUT_ERROR << "Could not update arm state! Please check connection" << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  auto pos = arm_->lastFeedback().getPosition();
  auto vel = arm_->lastFeedback().getVelocity();
  auto acc = arm_->lastFeedback().getEffort();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_pos_states_[i] = pos[i];
    joint_vel_states_[i] = vel[i];
    joint_acc_states_[i] = acc[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HEBIHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // write robot's commands

  if (!arm_) {
    std::cout << COUT_ERROR << "Arm object is not initialized!" << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  auto& command = arm_->pendingCommand();

  Eigen::Map<Eigen::VectorXd> pos(joint_pos_commands_.data(), joint_pos_commands_.size());
  Eigen::Map<Eigen::VectorXd> vel(joint_vel_commands_.data(), joint_vel_commands_.size());

  command.setPosition(pos);
  command.setVelocity(vel);

  if (!arm_->send()) {
    std::cout << COUT_ERROR << "Could not send commands! Please check connection" << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hebi_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hebi_hardware::HEBIHardwareInterface, hardware_interface::SystemInterface)
