#!/usr/bin/env sh

#ARGS="--symlink-install"
ARGS=""
COMMON_PKGS="rys_interfaces"
ROBOT_PKGS="rys_motors_controller "
ROBOT_PKGS="${ROBOT_PKGS} rys_sensor_imu rys_sensor_sonars rys_sensor_vl53l0x "
CONTROL_PKGS="rys_display_sensors "
CONTROL_PKGS="${CONTROL_PKGS} rys_remote "

# Check whether ament is available
command -v ament > /dev/null 2>&1;
if [ $? -ne 0 ] ; then
	echo "Ament not found. Install ROS2 first (try 'ros2_install')." >&2
	exit 1
fi

if [ -z $1 ] ; then
	ament build ${ARGS}
	exit
fi

if [ $1 = 'robot' ] ; then
	ament build ${ARGS} --only-packages ${ROBOT_PKGS} ${COMMON_PKGS}
	exit
fi

if [ $1 = 'control' ] ; then
	ament build ${ARGS} --only-packages ${CONTROL_PKGS} ${COMMON_PKGS}
	exit
fi

echo "Invalid argument. Possible values: [none], 'robot', 'control'"
exit 1
