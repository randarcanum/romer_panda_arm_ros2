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
    msg.header.frame_id = "panda_link0";

    double half_diag = 0.15;
    double height = 0.55;
    int a = i_ & 1;
    int b = i_ >> 1 & 1;

    msg.pose.position.x = 0.4 + (((a | b) & ~a) << 1 | a) * half_diag;
    msg.pose.position.y = ((((a | b) & ~b) << 1 | ~a & 1) - 1) * half_diag;
    msg.pose.position.z = height;
    i_++;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 1.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;

    publisher_->publish(msg);
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
