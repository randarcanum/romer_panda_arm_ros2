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

#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motion_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace romer_controllers {

/// The move to start example controller moves the robot into default pose.
class JointPositionControllerWithCV : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d q_;
  Vector7d q_goal_;
  Vector7d q_delta_;
  Vector7d q_error_;
  Vector7d dq_;
  Vector7d dq_desired_;
  Vector7d dq_filtered_;
  Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_;
  Vector7d ddq_max_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d e_sign_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time t_last_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_sub_;
  std::mutex target_mutex_;
  std::array<bool, 7> finished_{false, false, false, false, false, false, false};
  int flag = 0;
  rclcpp::Duration init_time_ = rclcpp::Duration(0, 0);

  void updateJointStates();
};
}  // namespace romer_controllers