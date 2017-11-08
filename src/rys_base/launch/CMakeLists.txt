cmake_minimum_required(VERSION 3.5)

project(rys_launch)

include(../cmake_modules/compile_options.cmake)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rys_interfaces REQUIRED)
find_package(rys_motors_controller REQUIRED)
find_package(rys_sensor_battery REQUIRED)
find_package(rys_sensor_dwm1000 REQUIRED)
find_package(rys_sensor_imu REQUIRED)
find_package(rys_sensor_temperature REQUIRED)
find_package(rys_sensor_vl53l0x REQUIRED)

add_executable(main
	src/main.cpp
)
ament_target_dependencies(main
	rclcpp
	std_msgs
	rys_interfaces
	rys_motors_controller
	rys_sensor_battery
	rys_sensor_dwm1000
	rys_sensor_imu
	rys_sensor_temperature
	rys_sensor_vl53l0x
)

add_executable(multithread
	src/multithread.cpp
)
ament_target_dependencies(multithread
	rclcpp
	std_msgs
	rys_interfaces
	rys_motors_controller
	rys_sensor_battery
	rys_sensor_dwm1000
	rys_sensor_imu
	rys_sensor_temperature
	rys_sensor_vl53l0x
)

# Install
install(TARGETS
	main
	multithread
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
