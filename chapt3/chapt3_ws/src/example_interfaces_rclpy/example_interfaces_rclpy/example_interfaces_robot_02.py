#!/usr/bin/env python3
from time import sleep
import rclpy
from rclpy.node import Node
# 导入接口
from example_ros2_interfaces.msg import RobotStatus
from example_ros2_interfaces.srv import MoveRobot
import math


class Robot():
    def __init__(self) -> None:
        self.current_pose_ = 0.0
        self.target_pose_ = 0.0
        self.status_ = RobotStatus.STATUS_STOP

    def get_status(self):
        return self.status_

    def get_current_pose(self):
        return self.current_pose_

    def move_distance(self,distance):
        self.status_ = RobotStatus.STATUS_MOVEING # 更新状态为移动、
        self.target_pose_ += distance # 更新目标位置

        while math.fabs(self.target_pose_ - self.current_pose_) > 0.01:
            step = distance / math.fabs(distance) * math.fabs(self.target_pose_ - self.current_pose_) * 0.1 # 计算一步移动距离
            self.current_pose_  += step # 移动一步
            print(f"移动了：{step}当前位置：{self.current_pose_}")
            sleep(0.5) #休息0.5s
        self.status_ = RobotStatus.STATUS_STOP # 更新状态为停止
        return self.current_pose_
        

class ExampleInterfacesRobot02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.robot = Robot()
        self.move_robot_server_ = self.create_service(MoveRobot,"move_robot", self.handle_move_robot) 
        self.robot_status_publisher_ = self.create_publisher(RobotStatus,"robot_status", 10) 
        self.publisher_timer_ = self.create_timer(0.5, self.publisher_timer_callback)
    

    def publisher_timer_callback(self):
        """
        定时器回调发布数据函数
        """
        msg = RobotStatus() #构造消息
        msg.status = self.robot.get_status()
        msg.pose = self.robot.get_current_pose()
        self.robot_status_publisher_.publish(msg) # 发布消息
        self.get_logger().info(f'发布了当前的状态：{msg.status} 位置：{msg.pose}')

    def handle_move_robot(self,request, response):
        self.robot.move_distance(request.distance)
        response.pose = self.robot.get_current_pose()
        return response

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ExampleInterfacesRobot02("example_interfaces_robot_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
