cmake_minimum_required(VERSION 3.5)

project(composition_)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(learning_interface REQUIRED)

include_directories(include)


#这个 CMake 命令将 node_plugins 变量初始化为空字符串。
#这通常是为了准备后续的操作，比如收集和注册节点插件信息。在实际的构建过程中，你可能会动态地向这个变量中添加需要注册的插件或库的名称。
set(node_plugins "")


#话题
add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")


#NodeLikeListener:
# 提供了更大的灵活性，因为它允许开发者在组件中使用任意对象结构，并且不必直接继承 rclcpp::Node。
# Listener:
# 实现较简单，直接继承 rclcpp::Node，适用于简单的节点实现。

#方法一
#这个组件直接继承自 rclcpp::Node。所有的 ROS 2 功能（如创建订阅）直接由基类 rclcpp::Node 提供。
#继承自 rclcpp::Node 的好处是可以直接使用 Node 类提供的接口，而不需要额外的实现 NodeBaseInterface。
add_library(listener_component SHARED
  src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(listener_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(listener_component "composition::Listener")
set(node_plugins "${node_plugins}composition::Listener;$<TARGET_FILE:listener_component>\n")

#方法二
#这个组件不直接继承自 rclcpp::Node。它使用了一个 rclcpp::Node 对象作为内部成员（node_），通过这个对象来处理消息和创建订阅。
#需要实现 get_node_base_interface() 方法来返回 rclcpp::NodeBaseInterface，这是为了符合 ROS 2 组件的要求。
add_library(node_like_listener_component SHARED
  src/node_like_listener_component.cpp)
target_compile_definitions(node_like_listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(node_like_listener_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(node_like_listener_component "composition::NodeLikeListener")
set(node_plugins "${node_plugins}composition::NodeLikeListener;$<TARGET_FILE:node_like_listener_component>\n")












# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

#服务
add_library(client_component SHARED
  src/client_component.cpp)
target_compile_definitions(client_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(client_component
  "learning_interface"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(client_component "composition::Client")
set(node_plugins "${node_plugins}composition::Client;$<TARGET_FILE:client_component>\n")


add_library(server_component SHARED
  src/server_component.cpp)
target_compile_definitions(server_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "learning_interface"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(server_component "composition::Server")
set(node_plugins "${node_plugins}composition::Server;$<TARGET_FILE:server_component>\n")





#具有硬编码节点的编译时组合
#该演示表明，可以重用相同的共享库来编译运行多个组件的单个可执行文件，而无需使用 ROS 接口。
#可执行文件包含上面的所有四个组件：发送者和接收者以及服务器和客户端，它们被硬编码在主函数中。
add_executable(manual_composition
  src/manual_composition.cpp)
target_link_libraries(manual_composition
  talker_component
  listener_component
  server_component
  client_component)
ament_target_dependencies(manual_composition
  "rclcpp")


#该演示通过创建通用容器进程并显式传递要加载的库而不使用 ROS 接口，提供了运行时组合的替代方案。
#该过程将打开每个库并在库中创建每个“rclcpp::Node”类的一个实例
add_executable(dlopen_composition
  src/dlopen_composition.cpp)
ament_target_dependencies(dlopen_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")


set(libs
  talker_component
  listener_component
  server_component
  client_component
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()

add_executable(linktime_composition
  src/linktime_composition.cpp)
target_link_libraries(linktime_composition ${libs})
  ament_target_dependencies(linktime_composition
    "class_loader"
    "rclcpp"
    "rclcpp_components")






install(TARGETS
  node_like_listener_component
  talker_component
  listener_component
  server_component
  client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(TARGETS
  #manual_composition所有节点在编译时就确定好了，不支持在运行时动态添加或卸载节点。
  #直接使用 std::make_shared 创建了 Talker, Listener, Server, 和 Client 四个节点的实例。这些节点是在编译时静态确定的。
  #适合节点固定、需求简单的场景。例如，一个基本的 ROS 2 节点启动程序，所有功能在编译时就已经确定。
  manual_composition 
  dlopen_composition
  #linktime_composition节点创建: 使用 class_loader::ClassLoader 动态加载共享库，并在运行时创建节点实例。这样可以在运行时根据需要加载不同的库和节点。
  #允许在运行时动态加载和卸载节点。需要通过指定的库路径动态加载节点类。
  #适合需要在运行时动态配置节点的场景。例如，插件式的系统，在运行时可以加载或卸载不同的功能模块。
  linktime_composition
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()