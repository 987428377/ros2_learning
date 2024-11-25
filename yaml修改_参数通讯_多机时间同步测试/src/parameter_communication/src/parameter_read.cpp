#include "rclcpp/rclcpp.hpp"
//#include <chrono>
#include <iostream>
//#include "std_msgs/msg/string.hpp"

class parameter_read : public rclcpp::Node
{
   public:
         // 构造函数,有一个参数为节点名称
         parameter_read(std::string name) : Node(name) 
         {
             RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
             // 创建客户端
             //client_ = this->create_client<custom_interface_pkg::srv::CustomService>("add_and_sub_srv");
             timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&parameter_read::read_parameter,this));
             //timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&ServiceClient01::send_request2,this));
             //针对重载的回调函数，绑定函数要用std::bind(static_cast<返回类型 (类名::*)(参数类型)>(&类名::重载函数名), /*其他参数*/);格式
             //timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(static_cast<void (ServiceClient01::*)() >(&ServiceClient01::send_request), this));
             //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this](rclcpp::TimerBase & timer) {send_request(3, 6 ,9);})

         }
    private:

         void read_parameter()
         {
             int num1;
             float num2;
             bool num3;
             this->get_parameter("parameter_int", num1);    //ZFLnode：中文标点
             this->get_parameter("parameter_float", num2);
             this->get_parameter("parameter_bool", num3);
             RCLCPP_INFO(this->get_logger(), "整形参数是%d",num1);
             RCLCPP_INFO(this->get_logger(),"浮点型参数是%f", num2);
             RCLCPP_INFO(this->get_logger(),"bool型参数是%s", num3 ? "true" : "false");
         };


   private:
          // 声名定时器指针
          rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) 
{
   rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
   auto node = std::make_shared<parameter_read>("read_parameter");
 
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
