### Package targets (common; pseudo-ament_auto)
# ROS Node 'library'
add_library(${PROJECT_NAME}_node SHARED ${SOURCE_FILES})
target_compile_definitions(${PROJECT_NAME}_node PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}_node ${AMENT_DEPENDENCIES})
ament_export_libraries(${PROJECT_NAME}_node)

# Header files
FILE(GLOB ${PROJECT_NAME}_headers ${HEADER_FILES})

# Single runner executable
add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_node)
ament_target_dependencies(main rclcpp)

## Install targets
# node class object into lib/
install(TARGETS
	${PROJECT_NAME}_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
# header files into include/
install(FILES ${${PROJECT_NAME}_headers}
	DESTINATION include/${PROJECT_NAME}
)
# runner file into lib/ (for `ros2 run`)
install(TARGETS
	main
	DESTINATION lib/${PROJECT_NAME}
)

## Create ament package
ament_package()
