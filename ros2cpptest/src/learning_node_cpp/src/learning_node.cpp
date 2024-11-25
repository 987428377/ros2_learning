#include "rclcpp/rclcpp.hpp"
#include <iostream>

class HelloWorldNode : public rclcpp::Node
{
public:
  HelloWorldNode()
    : Node("node_helloworld")
  {
    // 创建一个定时器，每0.5秒触发一次回调函数
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&HelloWorldNode::publishHelloWorld, this));
  }

private:
  void publishHelloWorld()
  {
    RCLCPP_INFO(get_logger(), "Hello World");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "1111" << std::endl;
  rclcpp::spin(std::make_shared<HelloWorldNode>());
  rclcpp::shutdown();
  return 0;
}
