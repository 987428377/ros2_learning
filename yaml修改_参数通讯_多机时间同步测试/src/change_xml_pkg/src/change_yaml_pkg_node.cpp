#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>  // 用于处理YAML文件
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>


class MyNode : public rclcpp::Node
{
public:
    MyNode()
        : Node("change_xml_pkg_node")
    {
        this->declare_parameter("global_map_path",rclcpp::ParameterValue(std::string("")));
        this->declare_parameter("test",rclcpp::ParameterValue(std::string("")));
         this->declare_parameter("config_path",rclcpp::ParameterValue(std::string("")));

        try{
            std::string global_map_path = this->get_parameter("global_map_path").as_string();
            std::string test = this->get_parameter("test").as_string();
            RCLCPP_INFO(this->get_logger(), "global_map_path: %s", global_map_path.c_str());
            RCLCPP_INFO(this->get_logger(), "test: %s", test.c_str());
        }catch(const std::exception& ex){
            std::cerr << "Failed to Read ROS parameter! " << ex.what() << std::endl;
            return ;
        }
        // 获取功能包路径
        // std::string package_name = "change_yaml_pkg";  // 替换为你的功能包名称
        // pkg_dir = ament_index_cpp::get_package_share_directory(package_name);
        // RCLCPP_INFO(this->get_logger(), "Package Share Path: %s", pkg_dir.c_str());
        // 获取 YAML 配置文件路径
        config_file_path;
        if (this->get_parameter("config_path", config_file_path)) {
            RCLCPP_INFO(this->get_logger(), "Config file path: %s", config_file_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get config file path parameter.");
        }

        //测试 也可以通过 ros2 param set /change_yaml_pkg_node global_map_path test_path 修改
        set_parameter(rclcpp::Parameter("global_map_path", "new/path/to/map"));
    }

    ~MyNode()
    {
        save_parameters_to_yaml();
    }
private:
    std::string config_file_path;

    // 保存修改后的参数值到 YAML 配置文件
    void save_parameters_to_yaml()
    {
        // 获取参数值
        std::string global_map_path_new = this->get_parameter("global_map_path").as_string();
        RCLCPP_INFO(this->get_logger(), "global_map_path: %s", global_map_path_new.c_str());
        
        // 使用 YAML-CPP 更新 YAML 文件
        try {
            std::string node_name = this->get_name();
            RCLCPP_INFO(this->get_logger(), "The name of this node is: %s", node_name.c_str());
            YAML::Node config = YAML::LoadFile(config_file_path);
            // config["change_yaml_pkg_node"]["ros__parameters"]["global_map_path"] = global_map_path_new;
            if (config[node_name]["ros__parameters"]) {
                config[node_name]["ros__parameters"]["global_map_path"] = global_map_path_new;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid YAML structure: cannot find 'ros__parameters'.");
                return;
            }

            // 保存修改后的内容到文件
            std::ofstream fout(config_file_path);
            fout << config;
            fout.close();

            RCLCPP_INFO(this->get_logger(), "Successfully saved new global_map_path to config file: %s", global_map_path_new.c_str());
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save parameters to YAML file: %s", ex.what());
        }
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}