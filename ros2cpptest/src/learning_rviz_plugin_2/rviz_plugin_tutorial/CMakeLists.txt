# 为了生成正确的 Qt 文件，我们需要：
#   打开 CMAKE_AUTOMOC
#   通过使用其中包含 Q_OBJECT 的每个标头调用 qt5_wrap_cpp 来包装标头。
#   将 MOC_FILES 与我们的其他 cpp 文件一起包含在库中。
#   请注意，如果您wrap头文件，则在运行时尝试加载插件时可能会收到一条错误消息，大致如下：


# v4
# cmake_minimum_required(VERSION 3.5)
# project(rviz_plugin_tutorial)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rviz_common REQUIRED)
# find_package(rviz_plugin_tutorial_msgs REQUIRED)
# find_package(rviz_rendering REQUIRED)

# set(CMAKE_AUTOMOC ON)
# qt5_wrap_cpp(MOC_FILES
#   include/rviz_plugin_tutorial/point_display.hpp
# )

# add_library(point_display src/point_display.cpp ${MOC_FILES})
# target_include_directories(point_display PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(point_display
#   pluginlib
#   rviz_common
#   rviz_plugin_tutorial_msgs
#   rviz_rendering
# )
# install(TARGETS point_display
#         EXPORT export_rviz_plugin_tutorial
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )
# install(DIRECTORY include/
#         DESTINATION include
# )
# install(FILES rviz_common_plugins.xml
#         DESTINATION share/${PROJECT_NAME}
# )
# ament_export_include_directories(include)
# ament_export_targets(export_rviz_plugin_tutorial)
# pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()


# V5
cmake_minimum_required(VERSION 3.10)
project(rviz_plugin_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_plugin_tutorial_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/rviz_plugin_tutorial/point_display.hpp
)

add_library(point_display src/point_display.cpp ${MOC_FILES})
target_include_directories(point_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(point_display
  pluginlib
  rviz_common
  rviz_plugin_tutorial_msgs
  rviz_rendering
)
install(TARGETS point_display
        EXPORT export_rviz_plugin_tutorial
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/Point2D.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_plugin_tutorial)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()