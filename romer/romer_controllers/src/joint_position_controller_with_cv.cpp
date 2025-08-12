// Copyright (c) 2021 Franka Emika GmbH
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

#include <romer_controllers/joint_position_controller_with_cv.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace romer_controllers {

controller_interface::InterfaceConfiguration
JointPositionControllerWithCV::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionControllerWithCV::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::return_type JointPositionControllerWithCV::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  q_error_ = q_goal_ - q_;
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  dq_desired_ = (k_gains_.cwiseProduct(q_error_) * 0.01 - d_gains_.cwiseProduct(dq_filtered_).cwiseProduct(q_error_.cwiseSign())).cwiseMin(dq_max_ * 0.01).cwiseMax(-dq_max_ * 0.01);
  

  for (int i = 0; i < num_joints; i++) {
    if (q_error_(i) > 0) dq_desired_(i) = std::min(dq_desired_(i), dq_(i) + ddq_max_(i) * 0.001);
    else dq_desired_(i) = std::max(dq_desired_(i), dq_(i) - ddq_max_(i) * 0.001);
    if (i == 0) RCLCPP_INFO(get_node()->get_logger(), "Joint %d velocity: %.5f, desired velocity: %.5f, effort: %.5f, error: %.5f", i, dq_(i), dq_desired_(i), ddq_(i), q_error_(i));
    if (q_error_(i) > 1e-3 || q_error_(i) < -1e-3) command_interfaces_[i].set_value(dq_desired_(i));
    else command_interfaces_[i].set_value(0);
  }
  if (flag < 60 && flag > -1) {
    
    RCLCPP_INFO(get_node()->get_logger(), "Goal: %.5f %.5f %.5f %.5f %.5f %.5f %.5f",
                        q_goal_(0), q_goal_(1),
                        q_goal_(2), q_goal_(3),
                        q_goal_(4), q_goal_(5),
                        q_goal_(6));
    RCLCPP_INFO(get_node()->get_logger(), "Start: %.5f %.5f %.5f %.5f %.5f %.5f %.5f",
                        q_(0), q_(1),
                        q_(2), q_(3),
                        q_(4), q_(5),
                        q_(6));
  }
  flag++;
  return controller_interface::return_type::OK;
}

CallbackReturn JointPositionControllerWithCV::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionControllerWithCV::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionControllerWithCV::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  RCLCPP_INFO(get_node()->get_logger(), "Joint %d actual velocity: %.5f", 0, dq_(0));
  init_time_ = rclcpp::Duration(0, 0);
  for (int i = 0; i < num_joints; ++i) {
    q_goal_(i) = q_(i);
    q_delta_(i) = dq_max_(i) * 0.1;
  }
  target_joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_position_target", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if ((int) msg->position.size() == num_joints) {
          std::lock_guard<std::mutex> lock(target_mutex_);
          updateJointStates();
          for (int i = 0; i < num_joints; ++i) {
            q_goal_(i) = msg->position[i];
            q_delta_(i) = dq_max_(i) * 0.01;
            finished_[i] = false;
          }
          
          RCLCPP_INFO(get_node()->get_logger(), "Received %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                      q_goal_(0), q_goal_(1),
                      q_goal_(2), q_goal_(3),
                      q_goal_(4), q_goal_(5),
                      q_goal_(6));
        } else {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Received joint state with wrong number of joints. Expected %d, got %zu.",
                      num_joints, msg->position.size());
        }
      });
  RCLCPP_INFO(get_node()->get_logger(), "Goal: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                      q_goal_(0), q_goal_(1),
                      q_goal_(2), q_goal_(3),
                      q_goal_(4), q_goal_(5),
                      q_goal_(6));
  RCLCPP_INFO(get_node()->get_logger(), "Start: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                      q_(0), q_(1),
                      q_(2), q_(3),
                      q_(4), q_(5),
                      q_(6));
  return CallbackReturn::SUCCESS;
}

void JointPositionControllerWithCV::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(3 * i);
    const auto& velocity_interface = state_interfaces_.at(3 * i + 1);
    const auto& effort_interface = state_interfaces_.at(3 * i + 2);

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
    ddq_(i) = effort_interface.get_value();
  }
}
}  // namespace romer_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(romer_controllers::JointPositionControllerWithCV,
                       controller_interface::ControllerInterface)