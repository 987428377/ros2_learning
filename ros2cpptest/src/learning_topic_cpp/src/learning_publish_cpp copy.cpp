#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "learning_interface/msg/object_position.hpp"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("topic_helloworld_pub") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);    
    timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&PublisherNode::timer_callback, this));
    
    
    //自定义消息
    publisher1_ = create_publisher<learning_interface::msg::ObjectPosition>("object_position", 10);    
    // 初始化 ObjectPosition 消息
    object_position_msg_.x = 0;
    timer_1 = create_wall_timer(std::chrono::milliseconds(500), [this]() {
          object_position_msg_.x++;
          publisher1_->publish(object_position_msg_);
        });   
  }

private:
  void timer_callback()
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "Hello World";
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  
  rclcpp::Publisher<learning_interface::msg::ObjectPosition>::SharedPtr publisher1_;
  rclcpp::TimerBase::SharedPtr timer_1;
  learning_interface::msg::ObjectPosition object_position_msg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
