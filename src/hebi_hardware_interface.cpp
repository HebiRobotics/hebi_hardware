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

  std::cout << "############################################################" << std::endl;

  std::cout << "########## Intializing HEBI Hardware Interface ##########" << std::endl;
  std::cout << COUT_INFO << "Reading parameters ==>" << std::endl;

  this->robot_name = info_.name;

  for (auto i = info_.hardware_parameters.begin(); i != info_.hardware_parameters.end(); i++) {
    if (i->first == "families") {
      this->params_.families_ = split(i->second, ';');
    }
    if (i->first == "names") {
      this->params_.names_ = split(i->second, ';');
    }
    if (i->first == "hrdf_pkg") {
      this->hrdf_pkg = i->second;
    }
    if (i->first == "hrdf_file") {
      this->hrdf_file = i->second;
    }
    if (i->first == "gains_pkg") {
      this->gains_pkg = i->second;
    }
    if (i->first == "gains_file") {
      this->gains_file = i->second;
    }
    if (i->first == "home_position") {
      std::vector<std::string> pos = split(i->second, ';');
      this->home_position_ = Eigen::VectorXd(pos.size());
      for (int i = 0; i < pos.size(); i++) {
        this->home_position_[i] = std::stod(pos[i]);
      }
    }
  }
  
  std::cout << "Families: " << std::endl;
  for (auto i = this->params_.families_.begin(); i != this->params_.families_.end(); i++) std::cout << *i << " ";
  std::cout << std::endl;

  std::cout << "Names: " << std::endl;
  for (auto i = this->params_.names_.begin(); i != this->params_.names_.end(); i++) std::cout << *i << " ";
  std::cout << std::endl;

  std::cout << "HRDF package: "<<this->hrdf_pkg << std::endl;
  std::cout << "HRDF file: "<<this->hrdf_file << std::endl;

  std::cout << "Gains package: "<<this->gains_pkg << std::endl;
  std::cout << "Gains file: "<<this->gains_file << std::endl;

  // Print home position
  std::cout << "Home position: " << std::endl;
  for (int i = 0; i < this->home_position_.size(); i++) {
    std::cout << i << ": " << this->home_position_[i];
    if (i < this->home_position_.size() - 1) {
      std::cout << ", ";
    } else {
      std::cout << std::endl;
    }
  }

  std::cout << "############################################################" << std::endl;

  joint_pos_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_vel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_acc_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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

hardware_interface::CallbackReturn HEBIHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  
  this->params_.hrdf_file_ = ament_index_cpp::get_package_share_directory(hrdf_pkg) + "/" + this->hrdf_file;
  for (int num_tries = 0; num_tries < 3; num_tries++) {
    this->arm_ = hebi::experimental::arm::Arm::create(this->params_);
    if (this->arm_) {
      break;
    }
    std::cout << COUT_WARN << "Could not initialize arm, trying again..." << std::endl;
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (!this->arm_) {
    std::cout << COUT_ERROR << "Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope)";
    return CallbackReturn::ERROR;
  } else  {
    std::cout << COUT_INFO << "Arm initialized!" << std::endl;
  }

  std::string gains_file = ament_index_cpp::get_package_share_directory(gains_pkg) + "/" + this->gains_file;
  std::cout << "Trying to load gains file at '"<<gains_file << "'" << std::endl;

  // Load the appropriate gains file
  if (!this->arm_->loadGains(gains_file)) {
    std::cout << COUT_ERROR << "Could not load gains file and/or set arm gains. Aborting." << std::endl;
    return CallbackReturn::ERROR;
  } else {
    std::cout << COUT_INFO << "Gains file loaded!" << std::endl;
  }

  this->arm_->update();

  // make sure commands are equal to the states on activation
  // joint_pos_commands_ = joint_pos_states_;
  // joint_vel_commands_ = joint_vel_states_;

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // prepare the robot to stop receiving commands
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HEBIHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
  // prepare the robot to stop receiving commands
  arm_.release();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HEBIHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // read robot states

  if (!this->arm_->update()) {
    return hardware_interface::return_type::ERROR;
  }

  auto pos = this->arm_->lastFeedback().getPosition();
  auto vel = this->arm_->lastFeedback().getVelocity();
  auto acc = this->arm_->lastFeedback().getEffort();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_pos_states_[i] = pos[i];
    joint_vel_states_[i] = vel[i];
    joint_acc_states_[i] = acc[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HEBIHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // write robot's commands
  auto& command = this->arm_->pendingCommand();

  Eigen::VectorXd pos(info_.joints.size());
  Eigen::VectorXd vel(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(joint_pos_commands_[i])) {
      pos[i] = home_position_[i];
      continue;
    } 
    pos[i] = joint_pos_commands_[i];
  }
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(joint_vel_commands_[i])) {
      vel.setConstant(std::numeric_limits<double>::quiet_NaN());
      break;
    }
    vel[i] = joint_vel_commands_[i];
  }

  command.setPosition(pos);
  command.setVelocity(vel);

  if (!this->arm_->send()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hebi_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hebi_hardware::HEBIHardwareInterface, hardware_interface::SystemInterface)
