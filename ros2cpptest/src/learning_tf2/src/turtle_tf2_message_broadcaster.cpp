#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"

class PointPublisher : public rclcpp::Node
{
public:
    PointPublisher()
    : Node("turtle_tf2_message_broadcaster"), turtle_spawning_service_ready_(false), turtle_spawned_(false), turtle_pose_cansubscribe_(false)
    {
        // Create a client to spawn a turtle
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        // Set up a timer to periodically call the on_timer function
        //创建一个定时器，用于定期调用某个函数。定时器每隔 1 秒触发一次。
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PointPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        if (turtle_spawning_service_ready_) {
            if (turtle_spawned_) {
                turtle_pose_cansubscribe_ = true;
            } else {
                //方法三
                //wait_for 是 std::shared_future 类的一个成员函数，用于等待异步操作的结果在指定时间内准备好。在这里，它等待 1 秒钟。
                // if (result_.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
                //     auto result = result_.get();
                //     RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", result->name.c_str());
                //     turtle_spawned_ = true;
                // } else {
                //     RCLCPP_INFO(this->get_logger(), "Spawn is not finished");
                // }

                // //方法二
                //auto future = client_->async_send_request(request_);
                //rclcpp::spin_until_future_complete 函数会阻塞当前线程
                auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_);
                if (result == rclcpp::FutureReturnCode::SUCCESS) {
                    auto result = result_.get();
                    RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", result->name.c_str());
                    turtle_spawned_ = true;
                } else {
                    RCLCPP_ERROR(get_logger(), "Failed to call service add_two_ints");
                }

            }
        } else {
            //这个调用用于检查服务是否在指定的时间内可用。在这里，它会等待 1 秒钟
            if (spawner_->wait_for_service(std::chrono::seconds(1))) {
                //创建一个新的请求对象，用于向服务发送请求。
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->name = "turtle3";
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                //发送异步请求到服务端。
                result_ = spawner_->async_send_request(request);
                turtle_spawning_service_ready_ = true;
            } else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }

        if (turtle_pose_cansubscribe_) {
            //查 vel_pub_ 指针是否为空，意味着检查是否已创建过名为 vel_pub_ 的发布者。如果 vel_pub_ 为空，表示尚未创建发布者。
            if (!vel_pub_) {
                vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle3/cmd_vel", 10);
            }
            if (!sub_) {
                sub_ = this->create_subscription<turtlesim::msg::Pose>(
                    "turtle3/pose", 10,
                    std::bind(&PointPublisher::handle_turtle_pose, this, std::placeholders::_1));
            }
            if (!pub_) {
                pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("turtle3/turtle_point_stamped", 10);
            }
        }
    }

    void handle_turtle_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        auto vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = 1.0;
        vel_msg.angular.z = 1.0;
        vel_pub_->publish(vel_msg);

        auto ps = geometry_msgs::msg::PointStamped();
        ps.header.stamp = this->get_clock()->now();
        ps.header.frame_id = "world";
        ps.point.x = msg->x;
        ps.point.y = msg->y;
        ps.point.z = 0.0;
        pub_->publish(ps);
    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    std::shared_future<std::shared_ptr<turtlesim::srv::Spawn::Response>> result_;
    bool turtle_spawning_service_ready_;
    bool turtle_spawned_;
    bool turtle_pose_cansubscribe_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointPublisher>());
    rclcpp::shutdown();
    return 0;
}
