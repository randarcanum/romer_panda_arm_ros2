#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class EETestPublisher : public rclcpp::Node {
public:
  EETestPublisher() : Node("ee_test_publisher"), t_(0.0) {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ee_pose_target", 10);
    timer_ = this->create_wall_timer(5000ms, std::bind(&EETestPublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "EE test publisher started.");
  }

private:
  void timer_callback() {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "panda_link0";  // Replace with your robot's base frame if needed

    double radius = 0.2;  // m
    double z = 0.6;       // Constant height
    double speed = 0.3;   // Radians/sec
    double r = 1; //std::sin(4 * t_);

    // msg.pose.position.x = z;
    // msg.pose.position.y = -0.2 + radius * r * std::cos(t_);
    // msg.pose.position.z = 0.5 + radius * r * std::sin(t_);

    double width = 0.2;
    double height = 0.2;

    msg.pose.position.x = 0.6;
    msg.pose.position.y = ((i_ >> 1 & 1) - 0.5) * width;
    // RCLCPP_INFO(this->get_logger(), "y: %.3f", ((i_ >> 1 & 1) - 0.5) * width);
    msg.pose.position.z = 0.5 + ((i_ & 1 ^ i_ >> 1 & 1) - 0.5) * height;
    // RCLCPP_INFO(this->get_logger(), "z: %.3f", 0.5 + ((i_ & 1) - 0.5) * height);
    i_++;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 1.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;

    publisher_->publish(msg);
    // t_ += speed;
    // t_ = fabs(fmod(t_ + M_PI * 0.5, M_PI)) - M_PI * 0.5;
    // RCLCPP_INFO(this->get_logger(), "Angle: %.3f", t_);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
  int i_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EETestPublisher>());
  rclcpp::shutdown();
  return 0;
}
