#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <ctime>

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // Get the clock from the node
        auto clock = this->get_clock();

        // Use the clock for timing operations
        auto current_time = clock->now();
        rclcpp::Time start_time = clock->now();
        std::time_t time_now = current_time.seconds(); // 获取当前时间的秒数
        // std::tm *tm_time = std::gmtime(&time_now);
        // char buffer[30];
        // std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_time);
        // RCLCPP_INFO(this->get_logger(), "Current time: %s", buffer);

        std::tm *tm_time = std::localtime(&time_now); // 转换为本地时间
        char buffer[30];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_time);
        
        RCLCPP_INFO(this->get_logger(), "Current time: %s", buffer);

        auto current_time2 = this->now();
        std::time_t time_now2 = current_time2.seconds(); // 获取当前时间的秒数
        std::tm *tm_time2 = std::localtime(&time_now2); // 转换为本地时间
        char buffer2[30];
        std::strftime(buffer2, sizeof(buffer2), "%Y-%m-%d %H:%M:%S", tm_time2);
        
        RCLCPP_INFO(this->get_logger(), "Current2 time: %s", buffer2);








    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
