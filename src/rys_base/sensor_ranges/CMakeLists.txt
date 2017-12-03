cmake_minimum_required(VERSION 3.5)

### Package name
project(rys_sensor_ranges)

### Compile options
include(../cmake_modules/compile_options.cmake)

### Package settings
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rys_interfaces REQUIRED)

set(HEADER_FILES
	src/RangesNode.hpp
	src/VL53L0X.hpp
	src/VL53L0X_defines.hpp
	src/I2Cdev.hpp
)
set(SOURCE_FILES
	src/RangesNode.cpp
	src/VL53L0X.cpp
	src/I2Cdev.cpp
)
set(AMENT_DEPENDENCIES
	rclcpp
	std_msgs
	rys_interfaces
)

### Package definitions, install targets etc
include(../cmake_modules/package_targets.cmake)
