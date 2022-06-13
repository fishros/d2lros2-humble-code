#include <chrono>

#include "rclcpp/rclcpp.hpp"
/*
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
*/
class ParametersBasicNode : public rclcpp::Node {
 public:
  // 构造函数,有一个参数为节点名称
  explicit ParametersBasicNode(std::string name) : Node(name) {
    // 打印一句
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    this->declare_parameter("rcl_log_level", 0);
    this->get_parameter("rcl_log_level", log_level);

    RCLCPP_INFO(this->get_logger(), "设置%s节点日志级别 %d .", name.c_str(),
                log_level);
    this->get_logger().set_level((rclcpp::Logger::Level)log_level);
    RCLCPP_INFO(this->get_logger(), "设置%s节点日志级别完成 %d .", name.c_str(),
                log_level);

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ParametersBasicNode::timer_callback, this));
  }

 private:
  int log_level;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    RCLCPP_DEBUG(this->get_logger(), "设置%s节点日志级别 %d .",
                 this->get_name(), log_level);
    RCLCPP_INFO(this->get_logger(), "设置%s节点日志级别 %d .", this->get_name(),
                log_level);

    this->get_logger().set_level((rclcpp::Logger::Level)log_level);
    RCLCPP_DEBUG(this->get_logger(), "我是DEBUG级别的日志，我被打印出来了!");
    RCLCPP_INFO(this->get_logger(), "我是INFO级别的日志，我被打印出来了!");
    RCLCPP_WARN(this->get_logger(), "我是WARN级别的日志，我被打印出来了!");
    RCLCPP_ERROR(this->get_logger(), "我是ERROR级别的日志，我被打印出来了!");
    RCLCPP_FATAL(this->get_logger(), "我是FATAL级别的日志，我被打印出来了!");
    RCLCPP_INFO(this->get_logger(), "设置%s节点日志级别完成 %d .",
                this->get_name(), log_level);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*产生一个的节点*/
  auto node = std::make_shared<ParametersBasicNode>("parameters_basic");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
