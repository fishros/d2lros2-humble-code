
from robot_control_interfaces.action import MoveRobot

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class ActionControl02(Node):
    """Action客户端"""

    def __init__(self):
        super().__init__('action_control_02')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')
        self.send_goal_timer_ = self.create_timer(1, self.send_goal)

    def send_goal(self):
        """发送目标"""
        self.send_goal_timer_.cancel()
        goal_msg = MoveRobot.Goal()
        goal_msg.distance = 5.0
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                                                                     feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """收到目标处理结果"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """获取结果反馈"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.pose}')

    def feedback_callback(self, feedback_msg):
        """获取回调反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.pose}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    action_robot_02 = ActionControl02()
    rclpy.spin(action_robot_02)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
