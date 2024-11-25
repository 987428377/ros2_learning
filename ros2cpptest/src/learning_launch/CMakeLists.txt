cmake_minimum_required(VERSION 3.8)
project(learning_launch)
# set(CMAKE_CXX_STANDARD 17)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(learning_interface REQUIRED)


#调试工具
# find_package(backward_ros REQUIRED)

# add_executable(adder_server src/adder_server.cpp)
# add_executable(adder_client src/adder_client.cpp)

# target_include_directories(adder_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
  
# target_include_directories(adder_client PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
  
#target_compile_features(adder_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ament_target_dependencies(
#   adder_server rclcpp std_msgs learning_interface
#   # "rclcpp"
#   # "std_msgs"
#   # "learning_interface"
# )
# ament_target_dependencies(
#   adder_client rclcpp std_msgs learning_interface
#   # "rclcpp"
#   # "std_msgs"
#   # "learning_interface"
# )

# target_link_libraries(adder_server ${CUSTOM_LIBRARIES} ) 

# install(TARGETS adder_server
#   DESTINATION lib/${PROJECT_NAME})
# install(TARGETS adder_client
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
install(
  DIRECTORY  
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
