cmake_minimum_required(VERSION 3.5)

project(rys_motors_controller)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

### ament_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_generate_code()
ament_auto_add_executable(rys_motors_controller
	src/rys_motors_controller.cpp
	src/Controller.cpp
	src/Motors.cpp
)
ament_auto_package()
