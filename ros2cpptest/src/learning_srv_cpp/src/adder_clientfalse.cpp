#include "rclcpp/rclcpp.hpp"
#include "msgs/srv/add_two_ints.hpp"  // 包含自定义的服务接口头文件
#include <future>  // 添加头文件以使用 std::future
#include <chrono>

class AdderClient : public rclcpp::Node {
public:
    AdderClient(int argc, char* argv[]) : Node("service_adder_client") {
        if (argc != 3) {
            RCLCPP_ERROR(get_logger(), "Usage: adder_client <int_value_a> <int_value_b>");
            rclcpp::shutdown();
            return;
        }

        client_ = create_client<msgs::srv::AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }
        request_ = std::make_shared<msgs::srv::AddTwoInts::Request>();
        request_->a = std::stoi(std::string(argv[1]));
        request_->b = std::stoi(std::string(argv[2]));

        // 异步发送请求
        future_ = std::async(std::launch::async, [this]() {
            auto response = client_->async_send_request(request_);
            return response.get();
        });
    }

private:
    void response_callback(rclcpp::Client<msgs::srv::AddTwoInts>::SharedFuture future) {
        if (future.valid()) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Sum: %d", response->sum);
            } catch (const std::exception& e) {
                RCLCPP_INFO(get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_INFO(get_logger(), "Future is invalid.");
        }
    }

    rclcpp::Client<msgs::srv::AddTwoInts>::SharedPtr client_;
    std::future<msgs::srv::AddTwoInts::Response::SharedPtr> future_;
    msgs::srv::AddTwoInts::Request::SharedPtr request_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);  // ROS2 C++接口初始化
    rclcpp::spin(std::make_shared<AdderClient>(argc, argv));  // 创建ROS2节点对象并进入主循环
    rclcpp::shutdown();  // 关闭ROS2 C++接口
    return 0;
}
