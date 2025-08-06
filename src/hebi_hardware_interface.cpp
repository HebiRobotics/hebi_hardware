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
  std::string gripper_joint;
  for (auto i = info_.hardware_parameters.begin(); i != info_.hardware_parameters.end(); i++) {
    if (i->first == "config_pkg") config_pkg_ = i->second;
    if (i->first == "config_file") config_file_ = i->second;
    if (i->first == "gripper_joint_name") gripper_joint = i->second;
  }

  if (config_pkg_.empty() || config_file_.empty()) {
    std::cout << COUT_ERROR << "Config package and/or file not provided! Aborting." << std::endl;
    return CallbackReturn::ERROR;
  }

  std::cout << COUT_INFO << "Config package: "<< config_pkg_ << std::endl;
  std::cout << COUT_INFO << "Config file: "<< config_file_ << std::endl;

  if (!gripper_joint.empty()) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (info_.joints[i].name == gripper_joint) {
        gripper_index_ = i;
        has_gripper_ = true;
        break;
      }
    }

    if (!has_gripper_) {
      std::cout << COUT_ERROR << "Gripper joint '" << gripper_joint << "' not found in hardware info! Aborting." << std::endl;
      return CallbackReturn::ERROR;
    }
  }

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

  // Setup gripper if it exists
  auto arm_config_user_data = arm_config_->getUserData();
  if (arm_config_user_data.hasBool("has_gripper")) {
    has_gripper_ = has_gripper_ && arm_config_user_data.getBool("has_gripper");
  }
  if (has_gripper_) {
    std::string gripper_family = arm_config_->getFamilies()[0];
    if (arm_config_user_data.hasString("gripper_family")) {
      gripper_family = arm_config_user_data.getString("gripper_family");
    }
    std::string gripper_name = "gripperSpool";
    if (arm_config_user_data.hasString("gripper_name")) {
      gripper_name = arm_config_user_data.getString("gripper_name");
    }
    double gripper_close_effort = 1.0;
    if (arm_config_user_data.hasFloat("gripper_close_effort")) {
      gripper_close_effort = arm_config_user_data.getFloat("gripper_close_effort");
    }
    double gripper_open_effort = -5.0;
    if (arm_config_user_data.hasFloat("gripper_open_effort")) {
      gripper_open_effort = arm_config_user_data.getFloat("gripper_open_effort");
    }

    // Create gripper from config
    gripper_ = arm::Gripper::create(
      gripper_family,
      gripper_name,
      gripper_close_effort,
      gripper_open_effort
    );

    // Terminate if gripper not found
    if (!gripper_) {
      std::cout << COUT_ERROR << "Failed to create gripper!" << std::endl;
      std::cout << COUT_ERROR << "Please check if the gripper is available in the network with the family: " << gripper_family << " and name: " << gripper_name << std::endl;
      return CallbackReturn::ERROR;
    }
    std::cout << COUT_INFO << "Gripper connected." << std::endl;

    std::string gains_file = arm_config_->getGains("gripper");
    if (!gains_file.empty()) {
      std::cout << COUT_INFO << "Loading gripper gains from: " << gains_file << std::endl;
      if (gripper_->loadGains(gains_file)) {
        std::cout << COUT_INFO << "Gripper gains loaded successfully." << std::endl;
      } else {
        std::cout << COUT_ERROR << "Failed to load gripper gains from: " << gains_file << std::endl;
        return CallbackReturn::ERROR;
      }
    }
  }

  // Check if arm size is same as number of joints
  if (arm_->size() + (has_gripper_ ? 1 : 0) != info_.joints.size()) {
    std::cout << COUT_ERROR << "Number of joints in the arm does not match number of joints in URDF!" << std::endl;
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
  if (gripper_) {
    gripper_.release();
    std::cout << COUT_INFO << "Gripper object released during shutdown." << std::endl;
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

  for (size_t i = 0; i < arm_->size(); ++i) {
    joint_pos_states_[i + (has_gripper_ && i >= gripper_index_ ? 1 : 0)] = pos[i];
    joint_vel_states_[i + (has_gripper_ && i >= gripper_index_ ? 1 : 0)] = vel[i];
    joint_acc_states_[i + (has_gripper_ && i >= gripper_index_ ? 1 : 0)] = acc[i];
  }

  if (gripper_) {
    joint_pos_states_[gripper_index_] = gripper_->getState();
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

  Eigen::VectorXd pos(arm_->size());
  Eigen::VectorXd vel(arm_->size());

  for (size_t i = 0; i < arm_->size(); ++i) {
    pos[i] = joint_pos_commands_[i + (has_gripper_ && i >= gripper_index_ ? 1 : 0)];
    vel[i] = joint_vel_commands_[i + (has_gripper_ && i >= gripper_index_ ? 1 : 0)];
  }

  command.setPosition(pos);
  command.setVelocity(vel);

  if (!arm_->send()) {
    std::cout << COUT_ERROR << "Could not send commands! Please check connection" << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  if (gripper_) {
    double gripper_command = joint_pos_commands_[gripper_index_];
    if (!std::isfinite(gripper_command)) gripper_command = 0.0; // Default to 0.0 if not set
    gripper_command = std::clamp(gripper_command, 0.0, 1.0); // Ensure command is within [0.0, 1.0]
    gripper_->setState(gripper_command);
    if (!gripper_->send()) {
      std::cout << COUT_ERROR << "Could not send gripper command! Please check connection" << std::endl;
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hebi_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hebi_hardware::HEBIHardwareInterface, hardware_interface::SystemInterface)
