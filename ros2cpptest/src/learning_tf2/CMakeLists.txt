cmake_minimum_required(VERSION 3.8)
project(learning_tf2)
set(CMAKE_CXX_STANDARD 14)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(backward_ros REQUIRED)

find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#处理不同ros发行版之间的差异
if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()


find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)


add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
   static_turtle_tf2_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)
install(TARGETS
   static_turtle_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})





add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
    turtle_tf2_broadcaster
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)
install(TARGETS
    turtle_tf2_broadcaster
    DESTINATION lib/${PROJECT_NAME})





add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
    ament_target_dependencies(
        turtle_tf2_listener
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
        turtlesim
    )
install(TARGETS
    turtle_tf2_listener
    DESTINATION lib/${PROJECT_NAME})





add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
    ament_target_dependencies(
        fixed_frame_tf2_broadcaster
        geometry_msgs
        rclcpp
        tf2_ros
)
install(TARGETS
    fixed_frame_tf2_broadcaster
    DESTINATION lib/${PROJECT_NAME})






add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
    ament_target_dependencies(
        dynamic_frame_tf2_broadcaster
        geometry_msgs
        rclcpp
        tf2_ros
    )
install(TARGETS
    dynamic_frame_tf2_broadcaster
    DESTINATION lib/${PROJECT_NAME})



add_executable(turtle_tf2_listener_debug src/turtle_tf2_listener_debug.cpp)
    ament_target_dependencies(
        turtle_tf2_listener_debug
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
        turtlesim
    )
install(TARGETS
turtle_tf2_listener_debug
    DESTINATION lib/${PROJECT_NAME})


#C++版本的message_broadcaster
add_executable(turtle_tf2_message_filter src/turtle_tf2_message_filter.cpp)
ament_target_dependencies(
    turtle_tf2_message_filter
    geometry_msgs
    message_filters
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    )
if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(turtle_tf2_message_filter PUBLIC -DTF2_CPP_HEADERS)
endif()

install(TARGETS
  turtle_tf2_message_filter
  DESTINATION lib/${PROJECT_NAME})






install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})



    

ament_package()