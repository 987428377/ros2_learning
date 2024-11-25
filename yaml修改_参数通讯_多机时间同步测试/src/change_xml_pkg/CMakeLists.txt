cmake_minimum_required(VERSION 3.10)
project(change_yaml_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif() 

# Default to C++14              
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif() 

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(ament_index_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp)

add_executable(${PROJECT_NAME}_node src/change_yaml_pkg_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp ament_index_cpp)

target_link_libraries(${PROJECT_NAME}_node yaml-cpp)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY  
        launch
        config 
    DESTINATION share/${PROJECT_NAME})

ament_package()   



