cmake_minimum_required(VERSION 3.5)

### Package name
project(rys_sensor_imu)

### Compile options
include(../cmake_modules/compile_options.cmake)

### Package settings
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(HEADER_FILES
	src/IMU.hpp
	src/IMUNode.hpp
	src/MPU6050.hpp
	src/I2Cdev.hpp
	src/helper_3dmath.hpp
)
set(SOURCE_FILES
	src/IMU.cpp
	src/IMUNode.cpp
	src/MPU6050.cpp
	src/I2Cdev.cpp
)
set(AMENT_DEPENDENCIES
	rclcpp
	sensor_msgs
	std_msgs
)

### Package definitions, install targets etc
include(../cmake_modules/package_targets.cmake)
