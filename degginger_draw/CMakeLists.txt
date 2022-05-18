cmake_minimum_required(VERSION 3.5)
project(degginger_draw)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)
# set(CMAKE_AUTOUIC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(touch_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies touch_interfaces rclcpp touch_interfaces image_geometry sensor_msgs tf2_ros tf2_geometry_msgs OpenCV)
set(srcs
  include/degginger_draw/frame.h
  src/main.cpp
  src/frame.cpp
  src/bezier.cpp
)

add_executable(draw ${srcs})
target_compile_definitions(draw PRIVATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
target_link_libraries(draw Qt5::Widgets)
ament_target_dependencies(draw ${dependencies})
target_include_directories(draw PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS draw
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
