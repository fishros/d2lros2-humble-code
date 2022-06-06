#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*产生一个node_01的节点*/
  auto node = std::make_shared<rclcpp::Node>("topic_publisher_01");
  RCLCPP_INFO(node->get_logger(), "topic_publisher_01节点已经启动.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}