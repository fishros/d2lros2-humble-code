
#include "example_ros2_interfaces/srv/linear_motion.hpp"
#include "example_ros2_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

class ExampleInterfacesControl : public rclcpp::Node {
public:
  // 构造函数,有一个参数为节点名称
  ExampleInterfacesControl(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    // 创建客户端
    client_ = this->create_client<example_ros2_interfaces::srv::LinearMotion>(
      "move_robot");
  }

  void send_request(float distance) {
    RCLCPP_INFO(this->get_logger(), "请求让机器人移动%f", distance);

    // 1.等待服务端上线
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      //等待时检测rclcpp的状态
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 2.构造请求的
    auto request =
      std::make_shared<example_ros2_interfaces::srv::LinearMotion::Request>();
    request->distance = distance;

    // 3.发送异步请求，然后等待返回，返回时调用回调函数
    client_->async_send_request(
      request, std::bind(&ExampleInterfacesControl::result_callback_, this,
                         std::placeholders::_1));
  };

private:
  // 声明客户端
  rclcpp::Client<example_ros2_interfaces::srv::LinearMotion>::SharedPtr client_;

  void result_callback_(
    rclcpp::Client<example_ros2_interfaces::srv::LinearMotion>::SharedFuture
      result_future) {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "收到移动结果：%f", response->pose);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*产生一个的节点*/
  auto node = std::make_shared<ExampleInterfacesControl>("example_interfaces_control_01");
  /* 运行节点，并检测退出信号*/
  node->send_request(5.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
