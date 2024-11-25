cmake_minimum_required(VERSION 3.8)
project(learning_custom_memory_allocator)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rmw REQUIRED)
find_package(tlsf_cpp REQUIRED)

# include_directories(include)
add_executable(learning_custom_memory_allocator src/allocator_tutorial.cpp)
ament_target_dependencies(learning_custom_memory_allocator rclcpp std_msgs)


install(TARGETS
  learning_custom_memory_allocator
  DESTINATION lib/${PROJECT_NAME})





# ######第二个可执行文件
# # add_library 命令用于创建一个新库。tlsf_cpp 是库的名称。INTERFACE 表示这是一个接口库（Interface Library）。接口库本身不编译任何源代码，它仅用于传播编译选项、包含目录和链接库。
# add_library(tlsf_cpp INTERFACE)
# # 指定包含目录 INTERFACE 表示这些目录是传递给依赖于此库的其他目标的。
# target_include_directories(tlsf_cpp INTERFACE
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" #示在构建时，包含目录为当前源目录下的 include 目录。这用于确保在开发时库的源代码可以被正确找到。
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>") #表示在安装时，包含目录为 include/${PROJECT_NAME}。这用于在安装到系统或其他位置时指定正确的包含路径。
# # 指定链接库 
#   target_link_libraries(tlsf_cpp INTERFACE
#   tlsf)

# add_executable(tlsf_allocator_example
#   src/tlsf.cpp)
# target_link_libraries(tlsf_allocator_example
#   rclcpp::rclcpp
#   ${std_msgs_TARGETS}
#   )

# install(TARGETS tlsf_allocator_example
#   DESTINATION bin)





ament_package()

