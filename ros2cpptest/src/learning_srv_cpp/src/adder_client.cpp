#include "rclcpp/rclcpp.hpp"
#include "learning_interface/srv/add_two_ints.hpp"  // 包含自定义的服务接口头文件

class AdderClient : public rclcpp::Node {
public:
    AdderClient(const std::vector<std::string>& args) : Node("service_adder_client") {
        // if (args.size() != 2) {
        //     RCLCPP_ERROR(get_logger(), "Usage: adder_client <int_value_a> <int_value_b>");
        //     rclcpp::shutdown();
        //     return;
        // }

        client_ = create_client<learning_interface::srv::AddTwoInts>("add_two_ints");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }
        request_ = std::make_shared<learning_interface::srv::AddTwoInts::Request>();
        request_->a = std::stoi(args[0]);
        request_->b = std::stoi(args[1]);

        // //方法一
        // using ServiceResponseFuture = rclcpp::Client<learning_interface::srv::AddTwoInts>::SharedFuture;
        // auto response_received_callback = [this](ServiceResponseFuture future) {
        //     auto response = future.get();
        //     RCLCPP_INFO(this->get_logger(), "Sum: %d", response->sum);
        //     rclcpp::shutdown();
        // };
        // auto future_result = client_->async_send_request(request_, response_received_callback);

        // //方法二
        // auto future = client_->async_send_request(request_);
        // //rclcpp::spin_until_future_complete 函数会阻塞当前线程
        // auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        // if (result == rclcpp::FutureReturnCode::SUCCESS) {
        //     RCLCPP_INFO(get_logger(), "Sum: %ld", future.get()->sum);
        // } else {
        //     RCLCPP_ERROR(get_logger(), "Failed to call service add_two_ints");
        // }

        //方法三  有问题，但是turtle_tf2_message_broadcaster没问题，感觉是服务端的问题
        //等待结果，最多 1 秒钟
        // std::shared_future<std::shared_ptr<learning_interface::srv::AddTwoInts::Response>> future = client_->async_send_request(request_);
        // if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
        //     try {
        //         auto response = future.get();  // 获取服务调用的结果
        //         RCLCPP_INFO(get_logger(), "Sum: %ld", response->sum);
        //     } catch (const std::exception &e) {
        //         // 捕获可能的异常并记录错误
        //         RCLCPP_ERROR(get_logger(), "Exception occurred: %s", e.what());
        //     }
        // } else {
        //     RCLCPP_ERROR(get_logger(), "Service call timed out");
        // }
        
    }

private:
    void response_callback(learning_interface::srv::AddTwoInts::Response::SharedPtr response) {
        RCLCPP_INFO(get_logger(), "Sum: %d", response->sum);
    }

    rclcpp::Client<learning_interface::srv::AddTwoInts>::SharedPtr client_;

    learning_interface::srv::AddTwoInts::Request::SharedPtr request_;
    learning_interface::srv::AddTwoInts::Response::SharedPtr future_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);  // ROS2 C++接口初始化
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: adder_client <int_value_a> <int_value_b>");
        return 1;
    }
    std::vector<std::string> args(argv + 1, argv + argc);
    rclcpp::spin(std::make_shared<AdderClient>(args));  // 创建ROS2节点对象并进入主循环
    rclcpp::shutdown();  // 关闭ROS2 C++接口
    return 0;
}

