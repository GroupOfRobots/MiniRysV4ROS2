import sys
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rys_messages.msg import ImuRoll

def sensorsReadingCallback(msg):
	print('Received: %f' % (msg.roll))

def main(args = None):
	if args is None:
		args = sys.argv

	rclpy.init(args)

	node = rclpy.create_node('rys_display_sensors')
	sub = node.create_subscription(ImuRoll, 'rys_imu', sensorsReadingCallback, qos_profile_sensor_data)
	# prevent unused variable warning
	assert sub

	while rclpy.ok():
		# Timeout of 1 due to RCLPY memory leak bug (https://github.com/ros2/rclpy/issues/74, there is already a PR that fixes it, PR#79)
		rclpy.spin_once(node, 1.0)

if __name__ == '__main__':
	main()
