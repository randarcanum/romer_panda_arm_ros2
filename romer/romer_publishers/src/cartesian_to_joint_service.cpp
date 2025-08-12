#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "romer_interfaces/srv/cartesian_to_joint.hpp"

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <urdf/model.h>

using namespace std::chrono_literals;
using CartToJoint = romer_interfaces::srv::CartesianToJoint;

class CartesianToJointService : public rclcpp::Node {
public:
  CartesianToJointService() : Node("joint_position_service") {
    joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };

    service_ = this->create_service<CartToJoint>(
      "cartesian_to_joint",
      std::bind(&CartesianToJointService::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_position_target_dummy", 10);
    
      // Get URDF
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

    if (!parameters_client->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(), "robot_state_publisher service not available.");
      return;
    }

    auto result = parameters_client->get_parameters({"robot_description"});
    if (result.empty() || result[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      RCLCPP_ERROR(this->get_logger(), "robot_description not found or not a string.");
      return;
    }

    std::string robot_desc_string = result[0].as_string();
    urdf::Model urdf_model;
    if (!urdf_model.initString(robot_desc_string)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      return;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree.");
      return;
    }

    if (!kdl_tree.getChain("panda_link0", "panda_link7", kdl_chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain.");
      return;
    }

    joint_min_.resize(kdl_chain_.getNrOfJoints());
    joint_max_.resize(kdl_chain_.getNrOfJoints());

    unsigned int j = 0;
    for (const auto& segment : kdl_chain_.segments) {
      const auto& joint = segment.getJoint();
      if (joint.getType() == KDL::Joint::None)
        continue;

      auto urdf_joint = urdf_model.getJoint(joint.getName());
      if (!urdf_joint || !urdf_joint->limits) {
        joint_min_(j) = -M_PI;
        joint_max_(j) = M_PI;
      } else {
        joint_min_(j) = urdf_joint->limits->lower;
        joint_max_(j) = urdf_joint->limits->upper;
      }
      ++j;
    }

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
      kdl_chain_, joint_min_, joint_max_, *fk_solver_, *ik_vel_solver_, 1000, 1e-6);

    q_last_.resize(kdl_chain_.getNrOfJoints());
    q_last_(0) = 0;
    q_last_(1) = -M_PI_4;
    q_last_(2) = 0;
    q_last_(3) = -3 * M_PI_4;
    q_last_(4) = 0;
    q_last_(5) = M_PI_2;
    q_last_(6) = M_PI_4;

    RCLCPP_INFO(this->get_logger(), "CartesianToJointService initialized.");
  }

private:
  void handle_request(const std::shared_ptr<CartToJoint::Request> request,
                      std::shared_ptr<CartToJoint::Response> response) {
    const auto& pose = request->pose.pose;

    KDL::Frame target_frame;
    target_frame.p = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);
    target_frame.M = KDL::Rotation::Quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    KDL::JntArray q_result(kdl_chain_.getNrOfJoints());

    int ret = ik_solver_->CartToJnt(q_last_, target_frame, q_result);
    if (ret >= 0) {
      response->success = true;
      sensor_msgs::msg::JointState joint_msg;
      joint_msg.header.stamp = this->now();
      joint_msg.name = joint_names_;
      joint_msg.position.resize(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        joint_msg.position[i] = q_result(i);
        q_last_(i) = q_result(i);
      }
      response->joint_state = joint_msg;
      publisher_->publish(joint_msg);
      RCLCPP_INFO(this->get_logger(), "IK solution computed and returned.");
    } else {
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "Failed to compute IK.");
    }
  }

  rclcpp::Service<CartToJoint>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::vector<std::string> joint_names_;
  KDL::JntArray joint_min_, joint_max_, q_last_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianToJointService>());
  rclcpp::shutdown();
  return 0;
}
