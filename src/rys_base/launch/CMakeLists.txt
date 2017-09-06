cmake_minimum_required(VERSION 3.5)

project(rys_launch)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

# Add warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rys_interfaces REQUIRED)
find_package(rys_motors_controller REQUIRED)
find_package(rys_sensor_dwm1000 REQUIRED)

# Motors controller runner executable
add_executable(main
	src/main.cpp
)
ament_target_dependencies(main
	rclcpp
	std_msgs
	rys_interfaces
	rys_motors_controller
	rys_sensor_dwm1000
)

# Install
install(TARGETS
	main
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
