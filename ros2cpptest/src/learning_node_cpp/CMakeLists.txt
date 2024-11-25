cmake_minimum_required(VERSION 3.8)
project(learning_node_cpp)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add your executable
add_executable(learning_node_cpp src/learning_node.cpp) 
#add_executable(test1 src/test1.cpp) # Assuming your source file is in a "src" folder

#ament_target_dependencies(test1 rclcpp std_msgs)
ament_target_dependencies(learning_node_cpp rclcpp )
# Include directories, if needed
# target_include_directories(learning_node_cpp PUBLIC include)



# Install the executable
#install(TARGETS test1
#  DESTINATION lib/${PROJECT_NAME})
install(TARGETS learning_node_cpp
  DESTINATION lib/${PROJECT_NAME})



ament_package()
