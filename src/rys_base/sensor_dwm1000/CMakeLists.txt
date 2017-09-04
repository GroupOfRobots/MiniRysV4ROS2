cmake_minimum_required(VERSION 3.5)

project(rys_sensor_dwm1000)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

# Add warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Add definition for DW1000 API
add_definitions(-D_XOPEN_SOURCE=500)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(rys_sensor_dwm1000
	src/main.cpp
	src/DWM.cpp
	src/DWMNode.cpp
	src/dwm_platform.c
	src/deca_api/deca_device.c
	src/deca_api/deca_params_init.c
)
ament_target_dependencies(rys_sensor_dwm1000
	rclcpp
	std_msgs
	pthread
	m
)

install(TARGETS
	rys_sensor_dwm1000
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
