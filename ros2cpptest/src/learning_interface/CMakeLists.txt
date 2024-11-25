cmake_minimum_required(VERSION 3.8)
project(learning_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)







set(msg_files
  "msg/ObjectPosition.msg"
  "srv/AddTwoInts.srv"
  "srv/GetObjectPosition.srv"
  "action/MoveCircle.action"
)
rosidl_generate_interfaces(
  msgs 
    ${msg_files}
  DEPENDENCIES
    std_msgs
  LIBRARY_NAME ${PROJECT_NAME})  


ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)





ament_package()
