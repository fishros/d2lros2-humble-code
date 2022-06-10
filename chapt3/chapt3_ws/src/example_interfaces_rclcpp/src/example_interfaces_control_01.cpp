#include "example_ros2_interfaces/msg/robot_status.hpp"
#include "example_ros2_interfaces/srv/move_robot.hpp"
#include "rclcpp/rclcpp.hpp"


class ExampleInterfacesControl : public rclcpp::Node {
 public:
  explicit ExampleInterfacesControl(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    /*创建move_robot客户端*/
    client_ = this->create_client<example_ros2_interfaces::srv::MoveRobot>(
        "move_robot");
    /*订阅机器人状态话题*/
    robot_status_subscribe_ =
        this->create_subscription<example_ros2_interfaces::msg::RobotStatus>(
            "robot_status", 10,
            std::bind(&ExampleInterfacesControl::robot_status_callback_, this,
                      std::placeholders::_1));
  }

  /**
   * @brief 发送移动机器人请求函数
   * 步骤：1.等待服务上线
   *      2.构造发送请求
   *
   * @param distance
   */
  void move_robot(float distance) {
    RCLCPP_INFO(this->get_logger(), "请求让机器人移动%f", distance);

    /*等待服务端上线*/
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      //等待时检测rclcpp的状态
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request =
        std::make_shared<example_ros2_interfaces::srv::MoveRobot::Request>();
    request->distance = distance;

    // 发送异步请求，然后等待返回，返回时调用回调函数
    client_->async_send_request(
        request, std::bind(&ExampleInterfacesControl::result_callback_, this,
                           std::placeholders::_1));
  };

 private:
  // 声明客户端
  rclcpp::Client<example_ros2_interfaces::srv::MoveRobot>::SharedPtr client_;
  rclcpp::Subscription<example_ros2_interfaces::msg::RobotStatus>::SharedPtr
      robot_status_subscribe_;
  /* 机器人移动结果回调函数 */
  void result_callback_(
      rclcpp::Client<example_ros2_interfaces::srv::MoveRobot>::SharedFuture
          result_future) {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "收到移动结果：%f", response->pose);
  }

  /**
   * @brief 机器人状态话题接收回调函数
   *
   * @param msg
   */
  void robot_status_callback_(
      const example_ros2_interfaces::msg::RobotStatus::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到状态数据位置：%f 状态：%d", msg->pose,
                msg->status);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleInterfacesControl>(
      "example_interfaces_control_01");
  node->move_robot(5.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
