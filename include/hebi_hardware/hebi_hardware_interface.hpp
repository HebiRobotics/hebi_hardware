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

#ifndef HEBI_HARDWARE__HEBI_HARDWARE_INTERFACE_HPP_
#define HEBI_HARDWARE__HEBI_HARDWARE_INTERFACE_HPP_

// HEBI C++ API components
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/arm/arm.hpp"

#include <string>
#include <vector>

#include "hebi_hardware/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>


namespace hebi_hardware {

class HEBIHardwareInterface : public hardware_interface::SystemInterface {

public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> joint_pos_commands_;
  std::vector<double> joint_vel_commands_;
  std::vector<double> joint_pos_states_;
  std::vector<double> joint_vel_states_;
  std::vector<double> joint_acc_states_;

  Eigen::VectorXd home_position_;

  std::vector<std::string> position_command_interface_names_;
  std::vector<std::string> velocity_command_interface_names_;

  std::string hrdf_pkg;
  std::string hrdf_file;
  std::string gains_pkg;
  std::string gains_file;
  std::string robot_name;

  std::unique_ptr<hebi::experimental::arm::Arm> arm_;
  hebi::experimental::arm::Arm::Params params_;
};

}  // namespace hebi_hardware

#endif  // HEBI_HARDWARE__HEBI_HARDWARE_INTERFACE_HPP_
