cmake_minimum_required(VERSION 3.5)

project(rys_sensor_imu)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

### ament_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_generate_code()
ament_auto_add_executable(rys_sensor_imu
	src/rys_sensor_imu.cpp
	src/I2Cdev.cpp
	src/IMU.cpp
	src/MPU6050.cpp
)
ament_auto_package()
