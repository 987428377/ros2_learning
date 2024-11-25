#include "rclcpp/rclcpp.hpp"
#include "learning_interface/srv/add_two_ints.hpp"

class AdderServerNode : public rclcpp::Node {
public:
    AdderServerNode() : Node("adder_server") {
        server_ = create_service<learning_interface::srv::AddTwoInts>(
            "add_two_ints", std::bind(&AdderServerNode::adderCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Service 'add_two_ints' is ready.");
    }

private:
    void adderCallback(
        const learning_interface::srv::AddTwoInts::Request::SharedPtr request,
        const learning_interface::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(get_logger(), "Incoming request\na: %ld b: %ld", request->a, request->b);
        RCLCPP_INFO(get_logger(), "Sending response: %ld", response->sum);
    }

    rclcpp::Service<learning_interface::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdderServerNode>());
    rclcpp::shutdown();
    return 0;
}