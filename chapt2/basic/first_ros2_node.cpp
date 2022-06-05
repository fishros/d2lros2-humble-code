// 包含rclcpp头文件，如果Vscode显示红色的波浪线也没关系
// 我们只是把VsCode当记事本而已，谁会在意记事本对代码的看法呢，不是吗？
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 调用rclcpp的初始化函数
    rclcpp::init(argc, argv);
    // 调用rclcpp的循环运行我们创建的first_node节点
    rclcpp::spin(std::make_shared<rclcpp::Node>("first_node"));
    return 0;
}