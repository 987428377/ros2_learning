
// //普通节点
// #include "rclcpp/rclcpp.hpp"
// namespace palomino
// {
//     class VincentDriver : public rclcpp::Node
//     {
//         public:
//             VincentDriver() : Node("composable_node"){
//                 std::cout << "test !" <<std::endl;
//             };
            
//     };
// }
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<palomino::VincentDriver>());
//     rclcpp::shutdown();
//     return 0;
// }


#include "rclcpp/rclcpp.hpp"

namespace palomino
{
    class VincentDriver : public rclcpp::Node
    {
        public:
            //您可能需要对类定义进行的唯一更改是确保该类的构造函数采用 NodeOptions 参数。
            VincentDriver(const rclcpp::NodeOptions & options) : Node("composable_node", options){
                std::cout << "test !" <<std::endl;
            }        
    };
}

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver)



