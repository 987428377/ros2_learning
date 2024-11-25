#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "std_msgs/msg/string.hpp"

class MinimalSubscriberWithTopicStatistics : public rclcpp::Node
{
public:
  MinimalSubscriberWithTopicStatistics()
  : Node("minimal_subscriber_with_topic_statistics")
  {
    // manually enable topic statistics via options
    //我们现在添加了配置订阅的选项，以使用 rclcpp::SubscriptionOptions() 选项结构启用主题统计。 还可以配置统计数据收集/发布期间和用于发布统计数据的主题等字段。
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    // configure the collection window and publish period (default 1s)
    // 配置收集窗口和发布周期（默认为1秒）收集统计数据并发布统计消息的周期（默认1s）
   
    options.topic_stats_options.publish_period = std::chrono::seconds(10);

    // configure the topic name (default '/statistics')发布统计数据时使用的主题（默认/statistics）
    // options.topic_stats_options.publish_topic = "/topic_statistics"

    auto callback = [this](std_msgs::msg::String::SharedPtr msg) {
        this->topic_callback(msg);
      };

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, callback, options);
  }

private:
  void topic_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
  rclcpp::shutdown();
  return 0;
}