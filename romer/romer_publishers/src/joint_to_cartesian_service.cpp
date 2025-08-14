#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "romer_interfaces/srv/joint_to_cartesian.hpp"

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>

using namespace std::chrono_literals;
using JointToCart = romer_interfaces::srv::JointToCartesian;

class JointToCartesianService : public rclcpp::Node {
public:
  JointToCartesianService() : Node("joint_position_service") {

    service_ = this->create_service<JointToCart>(
      "joint_to_cartesian",
      std::bind(&JointToCartesianService::handle_request, this, std::placeholders::_1, std::placeholders::_2));

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

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

    RCLCPP_INFO(this->get_logger(), "JointToCartesianService initialized.");
  }

private:
  void handle_request(const std::shared_ptr<JointToCart::Request> request,
                      std::shared_ptr<JointToCart::Response> response) {
    KDL::JntArray q_request(kdl_chain_.getNrOfJoints());

    for (int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      q_request(i) = request->joint_state.position[i];
    }

    KDL::Frame result_frame;

    int ret = fk_solver_->JntToCart(q_request, result_frame);
    if (ret >= 0) {
      response->success = true;
      response->pose.header.stamp = this->now();
      response->pose.pose.position.x = result_frame.p[0];
      response->pose.pose.position.y = result_frame.p[1];
      response->pose.pose.position.z = result_frame.p[2];
      result_frame.M.GetQuaternion(
        response->pose.pose.orientation.x,
        response->pose.pose.orientation.y,
        response->pose.pose.orientation.z,
        response->pose.pose.orientation.w
      );
    } else {
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "Failed to compute FK.");
    }
  }

  rclcpp::Service<JointToCart>::SharedPtr service_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointToCartesianService>());
  rclcpp::shutdown();
  return 0;
}
