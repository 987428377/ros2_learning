#include "rclcpp/rclcpp.hpp"                 // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"
#include "learning_interface/msg/object_position.hpp"           // ROS2标准定义的String消息

/**
 * 创建一个订阅者节点
 */
class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("topic_helloworld_sub") {
        // 创建订阅者对象（消息类型、话题名、队列长度、订阅者回调函数）
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10, std::bind(&SubscriberNode::listenerCallback, this, std::placeholders::_1));
        subscription1_ = this->create_subscription<learning_interface::msg::ObjectPosition>(
        "object_position", 10, std::bind(&SubscriberNode::listenerCallback1, this, std::placeholders::_1));
    
    }

private:
    void listenerCallback(const std_msgs::msg::String::SharedPtr msg) {
        // 回调函数，执行收到话题消息后对数据的处理
        RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void listenerCallback1(const learning_interface::msg::ObjectPosition::SharedPtr msg) {
        // 回调函数，执行收到话题消息后对数据的处理
        RCLCPP_INFO(get_logger(), "Received Object Position: x = %f", msg->x);
    }

    rclcpp::Subscription<learning_interface::msg::ObjectPosition>::SharedPtr subscription1_;   
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);                   // ROS2 C++接口初始化
    rclcpp::spin(std::make_shared<SubscriberNode>());  // 创建节点对象并进入主循环
    rclcpp::shutdown();                         // 关闭ROS2 C++接口
    return 0;
}

