cmake_minimum_required(VERSION 3.5)
project(rviz_camera_stream)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TODO: might not be needed
add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(yaml_cpp_vendor REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rcpputils)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_AUTOMOC ON)
#set(rviz_camera_stream_headers_to_moc
#set(${PROJECT_NAME}_mocs
#  include/${PROJECT_NAME}/camera_display.hpp
#)

#foreach(header "${rviz_camera_stream_headers_to_moc}")
#  qt5_wrap_cpp(rviz_camera_stream_moc_files "${header}")
#endforeach()

qt5_wrap_cpp(${PROJECT_NAME}_mocs include/${PROJECT_NAME}/camera_display.hpp)

#set(${PROJECT_NAME}_source_files
#  src/camera_display.cpp
#)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_mocs}
#  ${${PROJECT_NAME}_source_files}
  src/camera_display.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_CAMERA_STREAM_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rcpputils
  rclcpp
  sensor_msgs
  image_transport
  rviz_common
  rviz_rendering
  rviz_default_plugins
  yaml_cpp_vendor
  pluginlib
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rcpputils
  rclcpp
  image_transport
  sensor_msgs
  rviz_common
  rviz_rendering
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# TODO: fix this
#install(
#        DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
#        DESTINATION "share/${PROJECT_NAME}"
#)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # uncomment the line when a copyright and license is not present in all source files
#  #set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # uncomment the line when this package is not in a git repo
#  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()