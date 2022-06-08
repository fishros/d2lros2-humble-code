#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


# 参数有关的几个函数
# https://docs.ros2.org/latest/api/rclpy/search.html?q=parameter
# 声明
# declare_parameter	        声明和初始化一个参数
# declare_parameters	    声明和初始化一堆参数
# 获取
# describe_parameter(name)  通过参数名字获取参数的描述
# get_parameter	            通过参数名字获取一个参数
# get_parameters	        通过多个参数名字获取多个参数
# 设置
# set_parameters	        设置一组参数的值
# 
# has_parameter             参数是否被声明
# rcutils/logging.h
#   RCUTILS_LOG_SEVERITY_UNSET = 0,  ///< The unset log level
#   RCUTILS_LOG_SEVERITY_DEBUG = 10,  ///< The debug log level
#   RCUTILS_LOG_SEVERITY_INFO = 20,  ///< The info log level
#   RCUTILS_LOG_SEVERITY_WARN = 30,  ///< The warn log level
#   RCUTILS_LOG_SEVERITY_ERROR = 40,  ///< The error log level
#   RCUTILS_LOG_SEVERITY_FATAL = 50,  ///< The fatal log level
# ros2 run example_parameters_rclpy parameters_basic --ros-args -p "rcl_log_level:=50"

class ParametersBasicNode(Node):
    """
    创建一个ParametersBasicNode节点，并在初始化时输出一个话
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        # 声明参数
        self.declare_parameter('rcl_log_level', 0) 
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().info(f"设置日至级别为 {log_level} !")
        self.get_logger().set_level(log_level)
        self.get_logger().info(f"已经设置日至级别为 {log_level} !")
        # 定时修改
        self.timer = self.create_timer(0.5,self.timer_callback)

    def timer_callback(self):
        self.get_logger().debug(f"进入日志回调设置函数!")
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().info(f"设置日至级别为 {log_level} !")
        self.get_logger().set_level(log_level)
        self.get_logger().info(f"已经设置日至级别为 {log_level} !")


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ParametersBasicNode("parameters_basic")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
