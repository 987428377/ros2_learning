#include "rclcpp/rclcpp.hpp"                 // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "learning_interface/msg/object_position.hpp"           // ROS2标准定义的String消息
#include <iomanip>

/**
 * 创建一个订阅者节点
 */
class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("topic_helloworld_sub") {
        // 创建订阅者对象（消息类型、话题名、队列长度、订阅者回调函数）
        subscription_ = this->create_subscription<std_msgs::msg::Header>(
            "chatter", 10, std::bind(&SubscriberNode::listenerCallback, this, std::placeholders::_1));
        subscription1_ = this->create_subscription<learning_interface::msg::ObjectPosition>(
        "object_position", 10, std::bind(&SubscriberNode::listenerCallback1, this, std::placeholders::_1));
    
    }

private:
    void listenerCallback(const std_msgs::msg::Header::SharedPtr msg) {
        // 回调函数，执行收到话题消息后对数据的处理
        auto received_time = msg->stamp;
         std::string received_time_str = convert_stamp_to_string(received_time);
        RCLCPP_INFO(this->get_logger(), "Received timestamp: %s", received_time_str.c_str());

        auto current_time = this->now();

        // 转换当前时间为年月日时分秒格式
        std::string current_time_str = convert_stamp_to_string(current_time);
        RCLCPP_INFO(this->get_logger(), "Current time: %s", current_time_str.c_str());

        // 比较时间差
        auto time_diff = current_time - received_time;
        RCLCPP_INFO(this->get_logger(), "Time difference: %f seconds", time_diff.seconds()); 

    }

    void listenerCallback1(const learning_interface::msg::ObjectPosition::SharedPtr msg) {
        // 回调函数，执行收到话题消息后对数据的处理
        // RCLCPP_INFO(get_logger(), "Received Object Position: x = %f", msg->x);
    }

     std::string convert_stamp_to_string(const rclcpp::Time &stamp)
    {
        // 获取时间戳的秒数
        double timestamp_seconds = stamp.seconds();

        // 转换为 std::chrono::system_clock::time_point
        std::chrono::system_clock::time_point time_point = std::chrono::system_clock::from_time_t(static_cast<std::time_t>(timestamp_seconds));

        // 转换为系统时间（tm结构）
        std::time_t time = std::chrono::system_clock::to_time_t(time_point);
        std::tm *tm_info = std::localtime(&time);

        // 使用 std::ostringstream 格式化输出年月日时分秒
        std::ostringstream oss;
        oss << std::put_time(tm_info, "%Y-%m-%d %H:%M:%S");

        return oss.str();
    }

    rclcpp::Subscription<learning_interface::msg::ObjectPosition>::SharedPtr subscription1_;   
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);                   // ROS2 C++接口初始化
    rclcpp::spin(std::make_shared<SubscriberNode>());  // 创建节点对象并进入主循环
    rclcpp::shutdown();                         // 关闭ROS2 C++接口
    return 0;
}

