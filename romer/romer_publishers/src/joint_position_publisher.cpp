#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <urdf/model.h>

using namespace std::chrono_literals;

class JointPositionPublisher : public rclcpp::Node {
public:
  JointPositionPublisher()
  : Node("joint_position_publisher") {
    joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_position_target_dummy", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_pose_target", 10,
      std::bind(&JointPositionPublisher::pose_callback, this, std::placeholders::_1));

    // Load robot model and IK solver
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      this, "robot_state_publisher");

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
    RCLCPP_INFO(this->get_logger(), "URDF received, length: %zu", robot_desc_string.size());

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
    RCLCPP_INFO(this->get_logger(), "KDL chain has %d joints.", kdl_chain_.getNrOfJoints());

    // Initialize joint limits
    joint_min_.resize(kdl_chain_.getNrOfJoints());
    joint_max_.resize(kdl_chain_.getNrOfJoints());

    unsigned int j = 0;
    for (const auto& segment : kdl_chain_.segments) {
      const auto& joint = segment.getJoint();
      if (joint.getType() == KDL::Joint::None)
        continue;

      auto urdf_joint = urdf_model.getJoint(joint.getName());
      if (!urdf_joint || !urdf_joint->limits) {
        RCLCPP_WARN(this->get_logger(), "Missing limits for joint %s", joint.getName().c_str());
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
      kdl_chain_,
      joint_min_,
      joint_max_,
      *fk_solver_,
      *ik_vel_solver_,
      1000,
      1e-6);

    RCLCPP_INFO(this->get_logger(), "JointPositionPublisher with IK ready.");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    KDL::Frame target_frame;
    target_frame.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    target_frame.M = KDL::Rotation::Quaternion(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);

    KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
    KDL::JntArray q_result(kdl_chain_.getNrOfJoints());

    // Initial guess: zeros
    if (!q_initialized) {
      q_init(0) = 0;
      q_init(1) = -M_PI_4;
      q_init(2) = 0;
      q_init(3) = -3 * M_PI_4;
      q_init(4) = 0;
      q_init(5) = M_PI_2;
      q_init(6) = M_PI_4;
      q_initialized = true;
      q_last_.resize(kdl_chain_.getNrOfJoints());
    } else {
      for (unsigned int i = 0; i < q_init.rows(); ++i)
        q_init(i) = q_last_(i);
    }

    int ret = ik_solver_->CartToJnt(q_init, target_frame, q_result);
    if (ret >= 0 ) {
      sensor_msgs::msg::JointState joint_msg;
      joint_msg.header.stamp = this->now();
      joint_msg.name = joint_names_;
      joint_msg.position.resize(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i)
        joint_msg.position[i] = q_result(i);
      publisher_->publish(joint_msg);
      RCLCPP_INFO(this->get_logger(), "IK solution reached.");
      for (unsigned int i = 0; i < q_init.rows(); ++i)
        q_last_(i) = q_result(i);
      return;
    }
    RCLCPP_WARN(this->get_logger(), "IK solution failed.");
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::vector<std::string> joint_names_;
  KDL::JntArray joint_min_;
  KDL::JntArray joint_max_;
  KDL::JntArray q_last_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  bool q_initialized = false;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPositionPublisher>());
  rclcpp::shutdown();
  return 0;
}
