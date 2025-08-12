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
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "romer_interfaces/srv/cartesian_to_joint.hpp"
#include "romer_interfaces/srv/coriolis.hpp"
#include <franka_msgs/srv/set_full_collision_behavior.hpp>

#include "motion_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace romer_controllers {

/// The move to start example controller moves the robot into default pose.
class CartesianPositionController : public controller_interface::ControllerInterface {
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
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Vector7d q_;
  Vector7d q_goal_;
  Vector7d q_desired_;
  Vector7d dq_;
  Vector7d torque_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d coriolis_;
  Vector7d gravity_;
  Vector7d torque_limits_ = (Vector7d() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0).finished();
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time start_time_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_cart_to_joint_sub_;
  rclcpp::Client<romer_interfaces::srv::CartesianToJoint>::SharedPtr cartesian_to_joint_client_;
  rclcpp::Client<romer_interfaces::srv::Coriolis>::SharedPtr coriolis_client_;
  std::mutex target_mutex_;
  bool moved_;
  std::vector<std::string> joint_names_; 

  void updateJointStates();
};
}  // namespace romer_controllers