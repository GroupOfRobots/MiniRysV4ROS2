cmake_minimum_required(VERSION 3.5)

### Package name
project(rys_sensor_dwm1000)

### Compile options
include(../cmake_modules/compile_options.cmake)

# Add definition for DW1000 API
add_definitions(-D_XOPEN_SOURCE=500)

### Package settings
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# Node class filename
set(HEADER_FILES
	src/DWM.hpp
	src/DWMNode.hpp
	src/dwm_platform.h
)
set(DECA_HEADER_FILES
	src/deca_api/deca_device_api.h
	src/deca_api/deca_param_types.h
	src/deca_api/deca_regs.h
	src/deca_api/deca_types.h
	src/deca_api/deca_version.h
)
set(SOURCE_FILES
	src/DWM.cpp
	src/DWMNode.cpp
	src/dwm_platform.c
	src/deca_api/deca_device.c
	src/deca_api/deca_params_init.c
)
set(AMENT_DEPENDENCIES
	rclcpp
	std_msgs
	pthread
	m
)

FILE(GLOB ${PROJECT_NAME}_deca_headers ${HEADER_FILES})
install(FILES ${${PROJECT_NAME}_deca_headers}
	DESTINATION include/${PROJECT_NAME}/deca_api
)

### Package definitions, install targets etc
include(../cmake_modules/package_targets.cmake)
