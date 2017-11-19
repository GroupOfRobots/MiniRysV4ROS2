cmake_minimum_required(VERSION 3.5)

### Package name
project(rys_motors_controller)

### Compile options
include(../cmake_modules/compile_options.cmake)

### Package settings
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rys_interfaces REQUIRED)

set(HEADER_FILES
	src/MotorsControllerNode.hpp
	src/MotorsController.hpp
)
set(SOURCE_FILES
	src/MotorsControllerNode.cpp
	src/MotorsController.cpp
)
set(AMENT_DEPENDENCIES
	rclcpp
	sensor_msgs
	std_msgs
	rys_interfaces
)

### Package definitions, install targets etc
include(../cmake_modules/package_targets.cmake)
