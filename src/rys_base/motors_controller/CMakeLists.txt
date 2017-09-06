cmake_minimum_required(VERSION 3.5)

project(rys_motors_controller)

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

# Motors controller node 'library'
add_library(rys_motors_controller_node SHARED
	src/MotorsController.cpp
	src/MotorsControllerNode.cpp
)
target_compile_definitions(rys_motors_controller_node
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(rys_motors_controller_node
	rclcpp
	std_msgs
	rys_interfaces
)
ament_export_libraries(rys_motors_controller_node)

# Motors controller runner executable
add_executable(main src/main.cpp)
target_link_libraries(main rys_motors_controller_node)
ament_target_dependencies(main rclcpp)

# Install
install(TARGETS
	rys_motors_controller_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
install(TARGETS
	main
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
