cmake_minimum_required(VERSION 3.5)

project(rys_interfaces)

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
	"msg/ImuRollRotation.msg"
	"msg/ImuYawPitchRoll.msg"
	"msg/LocationDistances.msg"
	"msg/Ranges.msg"
	"msg/Steering.msg"
)
set(srv_files
	"srv/GetRegulatorSettings.srv"
	"srv/SetRegulatorSettings.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	DEPENDENCIES builtin_interfaces std_msgs
	ADD_LINTER_TESTS
)

ament_package()
