from robot_control_interfaces.action import MoveRobot
import math


class Robot():
    """机器人类，模拟一个机器人"""

    def __init__(self) -> None:
        self.current_pose_ = 0.0
        self.target_pose_ = 0.0
        self.move_distance_ = 0.0
        self.status_ = MoveRobot.Feedback

    def get_status(self):
        """获取状态"""
        return self.status_

    def get_current_pose(self):
        """获取当前位置"""
        return self.current_pose_

    def close_goal(self):
        """接近目标"""
        return math.fabs(self.target_pose_ - self.current_pose_) < 0.01

    def stop_move(self):
        """停止移动"""
        self.status_ = MoveRobot.Feedback.STATUS_STOP

    def move_step(self):
        """移动一小步"""
        direct = self.move_distance_ / math.fabs(self.move_distance_)
        step = direct * math.fabs(self.target_pose_ - self.current_pose_) * 0.1
        self.current_pose_ += step  # 移动一步
        print(f"移动了：{step}当前位置：{self.current_pose_}")
        return self.current_pose_

    def set_goal(self, distance):
        """设置目标"""
        self.move_distance_ = distance
        self.target_pose_ += distance  # 更新目标位置

        if self.close_goal():
            self.stop_move()
            return False

        self.status_ = MoveRobot.Feedback.STATUS_MOVEING  # 更新状态为移动
        return True
