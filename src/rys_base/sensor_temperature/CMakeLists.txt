cmake_minimum_required(VERSION 3.5)

### Package name
project(rys_sensor_temperature)

### Compile options
include(../cmake_modules/compile_options.cmake)

### Package settings
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(HEADER_FILES
	src/TemperatureNode.hpp
)
set(SOURCE_FILES
	src/TemperatureNode.cpp
)
set(AMENT_DEPENDENCIES
	rclcpp
	std_msgs
)

### Package definitions, install targets etc
include(../cmake_modules/package_targets.cmake)
