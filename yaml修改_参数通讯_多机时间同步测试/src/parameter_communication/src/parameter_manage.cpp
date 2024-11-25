#include "rclcpp/rclcpp.hpp"

class parameter_manage : public rclcpp::Node 
{
   public:
        parameter_manage(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
            //声明多个参数
            this->declare_parameter("parameter_int",1);
            this->declare_parameter("parameter_float",3.14);
            this->declare_parameter("parameter_bool",true);


            //zfladd  
            int int_param;  
            float float_param;  
            bool bool_param;  
            this->get_parameter("parameter_int", int_param);  
            this->get_parameter("parameter_float", float_param);  
            this->get_parameter("parameter_bool", bool_param);  
  
            custom_parameter_obj.num1 = int_param;  
            custom_parameter_obj.num2 = float_param;  
        }
    private:
        struct custom_parameter
        {
            int num1;
            float num2;
        };
        custom_parameter custom_parameter_obj;


};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared <parameter_manage>("parameter_manage");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}