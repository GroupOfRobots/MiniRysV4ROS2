import sys
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rys_messages.msg import ImuYawPitchRoll

def sensorsReadingCallback(msg):
	print('Received: %f' % (msg.roll))

def main(args = None):
	if args is None:
		args = sys.argv

	rclpy.init(args)

	node = rclpy.create_node('rys_display_sensors')
	sub = node.create_subscription(ImuYawPitchRoll, 'rys_imu', sensorsReadingCallback, qos_profile_sensor_data)
	# prevent unused warning
	assert sub

	while rclpy.ok():
		rclpy.spin_once(node)

if __name__ == '__main__':
	main()
