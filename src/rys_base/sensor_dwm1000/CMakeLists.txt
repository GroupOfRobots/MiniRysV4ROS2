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

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# DWM Node 'library'
add_library(rys_sensor_dwm1000_node SHARED
	src/DWM.cpp
	src/DWMNode.cpp
	src/dwm_platform.c
	src/deca_api/deca_device.c
	src/deca_api/deca_params_init.c
)
target_compile_definitions(rys_sensor_dwm1000_node
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(rys_sensor_dwm1000_node
	rclcpp
	std_msgs
	pthread
	m
)
ament_export_libraries(rys_sensor_dwm1000_node)

# DWM runner executable
add_executable(main src/main.cpp)
target_link_libraries(main rys_sensor_dwm1000_node)
ament_target_dependencies(main rclcpp)

# Install
install(TARGETS
	rys_sensor_dwm1000_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
install(TARGETS
	main
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
