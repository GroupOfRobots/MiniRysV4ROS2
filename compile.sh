#!/usr/bin/env sh

ARGS="--symlink-install"
COMMON_PKGS="rys_messages"
ROBOT_PKGS="rys_motors_controller "
ROBOT_PKGS="${ROBOT_PKGS} rys_sensor_imu "
CONTROL_PKGS="rys_display_sensors "
CONTROL_PKGS="${CONTROL_PKGS} rys_remote "

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
