from robot_control_interfaces.action import MoveRobot
import math


class Robot():
    def __init__(self) -> None:
        self.current_pose_ = 0.0
        self.target_pose_ = 0.0
        self.status_ = MoveRobot.Feedback

    def get_status(self):
        return self.status_

    def get_current_pose(self):
        return self.current_pose_

    def move_distance(self, distance):
        self.status_ = RobotStatus.STATUS_MOVEING  # 更新状态为移动、
        self.target_pose_ += distance  # 更新目标位置

        while math.fabs(self.target_pose_ - self.current_pose_) > 0.01:
            step = distance / math.fabs(distance) * math.fabs(
                self.target_pose_ - self.current_pose_) * 0.1  # 计算一步移动距离
            self.current_pose_ += step  # 移动一步
            print(f"移动了：{step}当前位置：{self.current_pose_}")
            sleep(0.5)  # 休息0.5s
        self.status_ = RobotStatus.STATUS_STOP  # 更新状态为停止
        return self.current_pose_
