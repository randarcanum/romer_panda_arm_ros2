#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "romer_interfaces/srv/coriolis.hpp"

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <urdf/model.h>

using namespace std::chrono_literals;
using Coriolis = romer_interfaces::srv::Coriolis;

class CoriolisService : public rclcpp::Node {
public:
  CoriolisService() : Node("coriolis_service") {
    joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };

    service_ = this->create_service<Coriolis>(
      "coriolis",
      std::bind(&CoriolisService::handle_request, this, std::placeholders::_1, std::placeholders::_2));

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

    gravity_vector_ = KDL::Vector(0, 0, -9.81);

    kdl_dynamics_ = std::make_shared<KDL::ChainDynParam>(kdl_chain_, gravity_vector_);

    RCLCPP_INFO(this->get_logger(), "KDL Chain has %d joints and %d segments.",
                kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());
    for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
        KDL::RigidBodyInertia inertia = kdl_chain_.getSegment(i).getInertia();
        RCLCPP_INFO(this->get_logger(), "Segment %d: Mass = %f", i, inertia.getMass());
    }

    RCLCPP_INFO(this->get_logger(), "CoriolisService initialized.");
  }

private:
  void handle_request(const std::shared_ptr<Coriolis::Request> request,
                      std::shared_ptr<Coriolis::Response> response) {
    KDL::JntArray q_request(kdl_chain_.getNrOfJoints());
    KDL::JntArray dq_request(kdl_chain_.getNrOfJoints());

    for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      q_request(i) = request->joint_state.position[i];
      dq_request(i) = request->joint_state.velocity[i];
    }

    KDL::JntArray coriolis(kdl_chain_.getNrOfJoints());
    int ret = kdl_dynamics_->JntToCoriolis(q_request, dq_request, coriolis);

    if (ret >= 0) {
      response->success = true;
      sensor_msgs::msg::JointState joint_msg;
      joint_msg.header.stamp = this->now();
      joint_msg.name = joint_names_;
      joint_msg.effort.resize(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i)
        joint_msg.effort[i] = coriolis(i);
      response->coriolis = joint_msg;
    } else {
      response->success = false;
    }

    KDL::JntArray gravity(kdl_chain_.getNrOfJoints());
    ret = kdl_dynamics_->JntToGravity(q_request, gravity);
    if (ret >= 0) {
      response->success = true;
      sensor_msgs::msg::JointState joint_msg;
      joint_msg.header.stamp = this->now();
      joint_msg.name = joint_names_;
      joint_msg.effort.resize(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i)
        joint_msg.effort[i] = gravity(i);
      response->gravity = joint_msg;
    } else {
      response->success = false;
    }
  }

  std::vector<std::string> joint_names_;
  rclcpp::Service<Coriolis>::SharedPtr service_;
  KDL::Chain kdl_chain_;
  KDL::Vector gravity_vector_;
  std::shared_ptr<KDL::ChainDynParam> kdl_dynamics_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoriolisService>());
  rclcpp::shutdown();
  return 0;
}
