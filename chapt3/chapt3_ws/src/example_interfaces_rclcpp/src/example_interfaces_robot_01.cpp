#include "example_ros2_interfaces/srv/move_robot.hpp"
#include "example_ros2_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

/*
* 测试指令：ros2 service call /move_robot example_ros2_interfaces/srv/MoveRobot "{distance: 5}"
*/

class Robot {
public:
  Robot() = default;
  ~Robot() = default;
  float move_distance(float distance) {
    status = example_ros2_interfaces::msg::RobotStatus::STATUS_MOVEING;
    target_pose_ += distance;
    while (fabs(target_pose_ - current_pose_) > 0.5) {
      float step = distance / fabs(distance) * 0.1;
      current_pose_  += step;
      std::cout<<"移动了："<<step<<"当前位置："<<current_pose_<<std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    status = example_ros2_interfaces::msg::RobotStatus::STATUS_STOP;
    return current_pose_;
  }

  float get_current_pose()
  {
    return current_pose_;
  }

  int get_status()
  {
    return status;
  }

private:
  // 声明当前位置
  float current_pose_ = 0.0;
  // 目标距离
  float target_pose_ = 0.0;
  int status = example_ros2_interfaces::msg::RobotStatus::STATUS_STOP;
};



class ExampleInterfacesRobot : public rclcpp::Node {
public:
  ExampleInterfacesRobot(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    // 创建一个订阅者订阅话题
    move_robot_server_ = this->create_service<example_ros2_interfaces::srv::MoveRobot>(
      "move_robot", std::bind(&ExampleInterfacesRobot::handle_move_robot, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ExampleInterfacesRobot::timer_callback, this));
    robot_status_publisher_ = this->create_publisher<example_ros2_interfaces::msg::RobotStatus>("robot_status",10);
  }

private:
  Robot robot;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<example_ros2_interfaces::srv::MoveRobot>::SharedPtr move_robot_server_;
  // 声明话题发布者
  rclcpp::Publisher<example_ros2_interfaces::msg::RobotStatus>::SharedPtr robot_status_publisher_;
  void timer_callback()
  {
      // 创建消息
      example_ros2_interfaces::msg::RobotStatus message;
      message.status = robot.get_status();
      message.pose = robot.get_current_pose();
      RCLCPP_INFO(this->get_logger(), "Publishing: %f", robot.get_current_pose());
      // 发布消息
      robot_status_publisher_->publish(message);
  };

  // 收到话题数据的回调函数
  void handle_move_robot(const std::shared_ptr<example_ros2_interfaces::srv::MoveRobot::Request> request,
                          std::shared_ptr<example_ros2_interfaces::srv::MoveRobot::Response> response) {
      RCLCPP_INFO(this->get_logger(), "收到请求移动距离：%f，当前位置：%f", request->distance, robot.get_current_pose());
      robot.move_distance(request->distance);
      response->pose = robot.get_current_pose();
  };
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleInterfacesRobot>("example_interfaces_robot_01");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}