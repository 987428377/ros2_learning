cmake_minimum_required(VERSION 3.8)
project(learning_topic_cpp)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(learning_interface REQUIRED)




add_executable(learning_publish_cpp src/learning_publish_cpp.cpp) 
add_executable(learning_subscribe_cpp src/learning_subscribe_cpp.cpp) 

ament_target_dependencies(
 learning_publish_cpp 
 rclcpp std_msgs
 learning_interface)
ament_target_dependencies(
 learning_subscribe_cpp 
 rclcpp std_msgs 
 learning_interface) 

install(TARGETS learning_publish_cpp
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS learning_subscribe_cpp
  DESTINATION lib/${PROJECT_NAME})  
  
  
ament_package()
