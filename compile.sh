#!/usr/bin/env sh

#ARGS="--symlink-install"
ARGS=""
COMMON_PKGS="rys_interfaces"
ROBOT_PKGS="rys_motors_controller rys_sensor_imu rys_sensor_ranges rys_sensor_dwm1000 rys_sensor_battery rys_sensor_temperature rys_launch"
REMOTE_PKGS="rys_remote"
MOTOR_PKGS="rys_motors_controller"
BUILD_CMD="ament build"

# Check whether ament is available
command -v ament > /dev/null 2>&1;
if [ $? -ne 0 ] ; then
	echo "Ament not found. Install ROS2 first (try 'ros2_install')." >&2
	exit 1
fi

startTime=`date +%s.%N`
if [ -z $1 ] ; then
	${BUILD_CMD} ${ARGS}
elif [ $1 = 'robot' ] ; then
	${BUILD_CMD} ${ARGS} --only-packages ${ROBOT_PKGS} ${COMMON_PKGS}
elif [ $1 = 'remote' ] ; then
	${BUILD_CMD} ${ARGS} --only-packages ${REMOTE_PKGS} ${COMMON_PKGS}
elif [ $1 = 'motor' ] ; then
    ${BUILD_CMD} ${ARGS} --only-packages ${MOTOR_PKGS} ${COMMON_PKGS}
else
	echo "Invalid argument. Possible values: [none], 'robot', 'remote', 'motor'"
	exit 1
fi
endTime=`date +%s.%N`
runTime=$(echo "${endTime} - ${startTime}" | bc)
echo -e "\nExecution time: ${runTime}"
