#include "rclcpp/rclcpp.hpp"
#include "custom_interface_pkg/srv/custom_service.hpp"
class parameter_write : public rclcpp::Node 
{
   public:
        parameter_write(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
            //timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&parameter_write::write_parameter,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() -> void {  
                                                                                        this->write_parameter(); }  
        );  
        }


private:
      //ZFLmodify 使用成员变量
      //void write_parameter(int num1,float num2,bool num3) 
      void write_parameter()
        {
             
             this->set_parameter(rclcpp::Parameter("parameter_int", num1));
             this->set_parameter(rclcpp::Parameter("parameter_float", num2));
             this->set_parameter(rclcpp::Parameter("parameter_bool", num3));

             RCLCPP_INFO(this->get_logger(), "整形参数是%d",num1);
             RCLCPP_INFO(this->get_logger(),"浮点型参数是%f", num2);
             RCLCPP_INFO(this->get_logger(),"bool型参数是%s", num3 ? "true" : "false");
             num1++;
        };
private:
        // 声名定时器指针
          rclcpp::TimerBase::SharedPtr timer_;
          int num1 = 1;
          float num2 = 2.0;
          bool num3 = true;


};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared <parameter_write>("write_parameter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
