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

#include <romer_controllers/cartesian_position_controller.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace romer_controllers {

controller_interface::InterfaceConfiguration
CartesianPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::return_type CartesianPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  auto trajectory_time = clock_->now() - start_time_;
  auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  q_desired_ = motion_generator_output.first;
  bool finished = motion_generator_output.second;
  double error = (q_desired_ - q_).cwiseAbs().maxCoeff();
  // RCLCPP_INFO(get_node()->get_logger(), "Delta q over treshold at %f, %d", error, moved_);
  if (!moved_) {
    // if (torque_.cwiseAbs().maxCoeff() > 36)
      // moved_ = true;
  } else {
    if (error > 0.01) {
      q_desired_ = q_;
      RCLCPP_INFO(get_node()->get_logger(), "Free move");
    }
    else {
      moved_ = false;
      motion_generator_ = std::make_unique<MotionGenerator>(0.6, q_, q_goal_);
      start_time_ = clock_->now();
    }
  }
  // Eigen::Map<const Vector7d> coriolis(
  //   franka_robot_model_->getCoriolis().data());
  // RCLCPP_INFO(get_node()->get_logger(), "Delta q: %f", coriolis[1]);

  Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_desired_ - q_) + d_gains_.cwiseProduct(-dq_);
  for (int i = 0; i < 7; ++i) {
    // if (i == 5) RCLCPP_INFO(get_node()->get_logger(), "Delta q: %f", fabs(q_desired_(i) - q_(i)));
    // if (fabs(q_desired_(i) - q_(i)) > 0.05) RCLCPP_INFO(get_node()->get_logger(), "Delta q over treshold at %d", i);
    if (!moved_) command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CartesianPositionController::on_init() {
  q_goal_ << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel("fer/robot_model",
                                                   "fer"));
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
  moved_ = false;
  // auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
  //     "param_service_server/set_full_collision_behavior");
  // while (!collision_client->wait_for_service(std::chrono::duration<int64_t, std::milli>(1000))) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
  //     return CallbackReturn::ERROR;
  //   }
  //   RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
  // }

  // auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();

  // request->lower_torque_thresholds_nominal = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->upper_torque_thresholds_nominal = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->lower_torque_thresholds_acceleration = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->upper_torque_thresholds_acceleration = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->lower_force_thresholds_nominal = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->upper_force_thresholds_nominal = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->lower_force_thresholds_acceleration = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  
  // request->upper_force_thresholds_acceleration = {
  //     100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
  // collision_client->async_send_request(request, 
  //             [this](rclcpp::Client<franka_msgs::srv::SetFullCollisionBehavior>::SharedFuture future_result) {
  //                 std::lock_guard<std::mutex> lock(target_mutex_);

  //                 auto response = future_result.get();
  //                 if (!response->success) {
  //                   RCLCPP_FATAL(get_node()->get_logger(), "Failed to set collision behavior.");
  //                   return CallbackReturn::ERROR;
  //                 } else {
  //                   RCLCPP_INFO(get_node()->get_logger(), "Collision behavior set.");
  //                 }
  //             });

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  q_goal_ = q_;
  motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
  start_time_ = clock_->now();

  cartesian_to_joint_client_ = get_node()->create_client<romer_interfaces::srv::CartesianToJoint>("/cartesian_to_joint");
  coriolis_client_ = get_node()->create_client<romer_interfaces::srv::Coriolis>("/coriolis");

  target_joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_position_target", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if ((int) msg->position.size() == num_joints) {
          std::lock_guard<std::mutex> lock(target_mutex_);
          for (int i = 0; i < num_joints; ++i) {
            q_goal_(i) = msg->position[i];
          }

          updateJointStates();
          motion_generator_ = std::make_unique<MotionGenerator>(0.6, q_, q_goal_);
          start_time_ = clock_->now();
          moved_ = false;
          
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
  target_cart_to_joint_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_pose_target", 10, [this](const geometry_msgs::msg::PoseStamped msg) {
          std::lock_guard<std::mutex> lock(target_mutex_);
        
          if (!cartesian_to_joint_client_->service_is_ready()) {
              RCLCPP_WARN(get_node()->get_logger(), "Service /cartesian_to_joint is not ready.");
              return;
          }

          auto request = std::make_shared<romer_interfaces::srv::CartesianToJoint::Request>();
          request->pose = msg;

          cartesian_to_joint_client_->async_send_request(request, 
              [this](rclcpp::Client<romer_interfaces::srv::CartesianToJoint>::SharedFuture future_result) {
                  std::lock_guard<std::mutex> lock(target_mutex_);

                  auto response = future_result.get();
                  if (response->success) {
                      for (size_t i = 0; i < 7; ++i) {
                          q_goal_(i) = response->joint_state.position[i];
                      }
                      updateJointStates();
                      motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
                      start_time_ = clock_->now();
                      moved_ = false;
                      RCLCPP_INFO(get_node()->get_logger(), "Received target from IK service.");
                  } else {
                      RCLCPP_WARN(get_node()->get_logger(), "Failed to find IK solution.");
                  }
              });
      });
  return CallbackReturn::SUCCESS;
}

void CartesianPositionController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(3 * i);
    const auto& velocity_interface = state_interfaces_.at(3 * i + 1);
    const auto& effort_interface =   state_interfaces_.at(3 * i + 2);

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
    torque_(i) = effort_interface.get_value();
  }
}
}  // namespace romer_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(romer_controllers::CartesianPositionController,
                       controller_interface::ControllerInterface)