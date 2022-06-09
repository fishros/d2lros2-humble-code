
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
public:
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  Robot() = default;
  ~Robot() = default;
  /**
   * @brief 移动指定的距离
   *
   * @param distance
   * @return float
   */
  float move_distance(float distance) {
    status_ = MoveRobot::Feedback::STATUS_MOVEING;
    target_pose_ += distance;
    /* 当目标距离和当前距离大于0.01则持续向目标移动 */
    while (fabs(target_pose_ - current_pose_) > 0.01) {
      /* 每一步移动当前到目标距离的1/10*/
      float step = distance / fabs(distance) * fabs(target_pose_ - current_pose_) * 0.1;
      current_pose_ += step;
      std::cout << "移动了：" << step << "当前位置：" << current_pose_ << std::endl;
      /*当前线程休眠500ms*/
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    status_ = MoveRobot::Feedback::STATUS_STOP;
    return current_pose_;
  }
  /**
   * @brief Get the current pose
   *
   * @return float
   */
  float get_current_pose() { return current_pose_; }

  void stop_move() {}

  /**
   * @brief Get the status
   *
   * @return int
   *  1 example_ros2_interfaces::msg::RobotStatus::STATUS_MOVEING
   *  2 example_ros2_interfaces::msg::RobotStatus::STATUS_STOP
   */
  int get_status() { return status_; }

private:
  float current_pose_ = 0.0;             /*声明当前位置*/
  float target_pose_ = 0.0;              /*目标距离*/
  std::atomic<bool> cancel_flag_{false}; /*取消标志*/
  int status_ = MoveRobot::Feedback::STATUS_STOP;
};

// 创建一个ActionServer类
class ActionRobotCpp : public rclcpp::Node {
public:
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  explicit ActionRobotCpp(std::string name) : Node(name) {
    using namespace std::placeholders;

    this->action_server_ =
      rclcpp_action::create_server<MoveRobot>(this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
                                              this->get_node_waitables_interface(), "move_robot", std::bind(&ActionRobotCpp::handle_goal, this, _1, _2),
                                              std::bind(&ActionRobotCpp::handle_cancel, this, _1), std::bind(&ActionRobotCpp::handle_accepted, this, _1));
  }

private:
  Robot robot;
  rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MoveRobot::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with distance %f", goal->distance);
    (void)uuid;
    if (fabs(goal->distance > 100)) {
      RCLCPP_WARN(this->get_logger(), "目标距离太远了，本机器人表示拒绝！");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_WARN(this->get_logger(), "目标距离我可以走到，本机器人接受，准备出发！");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    /*认可取消执行，让机器人停下来*/
    robot.stop_move();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveRobot::Feedback>();
    float& pose = feedback->pose;
    auto result = std::make_shared<MoveRobot::Result>();

    pose = 0.1f;
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish Feedback");

    
    // 处理取消
    if (goal_handle->is_canceling()) {
      result->pose = 0.0f;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
      return;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->pose = 0.0;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionRobotCpp::execute, this, _1), goal_handle}.detach();
  }
};  // class ActionRobotCpp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<ActionRobotCpp>("action_robot_cpp");
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}