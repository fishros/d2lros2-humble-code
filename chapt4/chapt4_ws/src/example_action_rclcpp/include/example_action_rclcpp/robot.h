/*
copyright
*/
#ifndef EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#define EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#include "rclcpp/rclcpp.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
 public:
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  Robot() = default;
  ~Robot() = default;
  float move_step(); /*移动一小步，请间隔500ms调用一次*/
  bool set_goal(float distance); /*移动一段距离*/
  float get_current_pose();
  int get_status();
  bool close_goal(); /*是否接近目标*/
  void stop_move();  /*停止移动*/

 private:
  float current_pose_ = 0.0;             /*声明当前位置*/
  float target_pose_ = 0.0;              /*目标距离*/
  float move_distance_ = 0.0;            /*目标距离*/
  std::atomic<bool> cancel_flag_{false}; /*取消标志*/
  int status_ = MoveRobot::Feedback::STATUS_STOP;
};

#endif  // EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_