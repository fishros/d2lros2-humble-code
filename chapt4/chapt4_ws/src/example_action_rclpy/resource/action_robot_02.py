#!/usr/bin/env python3
from robot_control_interfaces.action import MoveRobot

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from example_action_rclpy.robot import Robot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ActionRobot02(Node):
    """机器人端Action服务"""

    def __init__(self):
        super().__init__('action_robot_02')
        self.robot_ = Robot()
        
        self.action_server_ = ActionServer(
            self, MoveRobot, 'move_robot', self.execute_callback,callback_group=MutuallyExclusiveCallbackGroup())


    def execute_callback(self, goal_handle: ServerGoalHandle):
        """执行回调函数,若采用默认handle_goal函数则会自动调用"""
        self.get_logger().info('执行移动机器人')
        feedback_msg = MoveRobot.Feedback()
        self.robot_.set_goal(goal_handle.request.distance)

        rate = self.create_rate(2)
        while rclpy.ok() and not self.robot_.close_goal():
            # move
            self.robot_.move_step()
            # feedback
            feedback_msg.pose = self.robot_.get_current_pose()
            feedback_msg.status = self.robot_.get_status()
            goal_handle.publish_feedback(feedback_msg)
            # cancel check
            if goal_handle.is_cancel_requested:
                result = MoveRobot.Result()
                result.pose = self.robot_.get_current_pose()
                return result
            rate.sleep()

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.pose = self.robot_.get_current_pose()
        return result

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    action_robot_02 = ActionRobot02()
    executor = MultiThreadedExecutor()
    executor.add_node(action_robot_02)
    executor.spin()
    # rclpy.spin(action_robot_02)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
