cmake_minimum_required(VERSION 3.8)
project(learning_composable_node)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Composable Node 
#首先，将 rclcpp_components 添加为 CMakeLists.txt 中的依赖项：
find_package(rclcpp_components REQUIRED)


# Add your executable
# add_executable(vincent_driver src/vincent_driver.cpp)
# Composable Node 
#其次，我们将用具有新目标名称的 add_library 替换 add_executable。
#add_library(vincent_driver_component src/vincent_driver.cpp) #静态库
add_library(vincent_driver_component SHARED src/vincent_driver.cpp)


# ament_target_dependencies(vincent_driver rclcpp )
# Composable Node 
#第三，替换使用旧目标的其他构建命令以作用于新目标。 IE。
ament_target_dependencies(vincent_driver_component rclcpp rclcpp_components)


# Composable Node 
#第四，添加一个新命令来声明您的组件。
rclcpp_components_register_node(
    vincent_driver_component
    #指定你的组件类的插件名称。这个名称是你在 RCLCPP_COMPONENTS_REGISTER_NODE 宏中定义的插件名称，它用于在运行时加载组件。插件名称是以 "package_name::ClassName" 的形式给出的。
    #你的代码中，你使用了 RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver)，因此 PLUGIN 的值应为 "palomino::VincentDriver"。
    PLUGIN "palomino::VincentDriver"
    #指定将用于运行组件的可执行文件的名称。这个名称是你在 CMake 构建过程中指定的可执行文件名称。
    EXECUTABLE vincent_driver
)


# Include directories, if needed
# target_include_directories(learning_node_cpp PUBLIC include)



# Install the executable
# install(TARGETS vincent_driver
#     DESTINATION lib/${PROJECT_NAME}
# )

# Composable Node 
#第五，也是最后，更改 CMake 中在旧目标上运行的所有安装命令以安装库版本。例如，不要将任一目标安装到 lib/${PROJECT_NAME} 中。替换为库安装。
ament_export_targets(export_vincent_driver_component) #这行指令创建了一个名为 export_vincent_driver_component 的目标导出。这允许其他 ROS 2 包通过 find_package() 来使用这个组件。
install(TARGETS vincent_driver_component
        EXPORT export_vincent_driver_component
        ARCHIVE DESTINATION lib #安装归档库（如静态库）到 lib 目录。
        LIBRARY DESTINATION lib #安装共享库到 lib 目录。
        RUNTIME DESTINATION bin #安装可执行文件到 bin 目录。
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
