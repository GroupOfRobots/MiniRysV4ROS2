import rclpy
from math import pi
from rys_messages.msg import Steering, ImuRoll, Sonars
from std_msgs.msg import Empty
from PyQt5.QtCore import QThread, pyqtSignal

class RosBridge(QThread):
	"""docstring for RosBridge"""

	rollChanged = pyqtSignal(float)
	sonarChanged = pyqtSignal(int, int, int)

	def __init__(self, node, parent = None, rate = 10):
		super(RosBridge, self).__init__(parent)
		self.node = node
		self.rate = rate
		self.exitFlag = False

		self.throttle = 0.0
		self.rotation = 0.0
		self.previousRoll = 0.0

	def setSteering(self, throttle, rotation):
		self.throttle = float(throttle)
		self.rotation = float(rotation)

	def calibrateImu(self):
		msg = Empty()
		self.publisherCalibrateImu.publish(msg)

	def stopExecution(self):
		self.exitFlag = True

	def imuSubscriptionCallback(self, message):
		roll = message.roll * 180 / pi
		if roll is self.previousRoll:
			return
		self.previousRoll = roll
		self.rollChanged.emit(roll)

	def sonarsSubscriptionCallback(self, message):
		self.sonarChanged.emit(message.front, message.back, message.top)

	def timerCallback(self):
		msg = Steering()
		msg.throttle = self.throttle
		msg.rotation = self.rotation
		self.publisherSteering.publish(msg)

	def run(self):
		self.publisherSteering = self.node.create_publisher(Steering, 'rys_steering')
		self.publisherCalibrateImu = self.node.create_publisher(Empty, 'rys_imu_calibrate')

		subscriptionImu = self.node.create_subscription(ImuRoll, 'rys_imu', self.imuSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		subscriptionSonars = self.node.create_subscription(Sonars, 'rys_sonars', self.sonarsSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		# prevent unused variable warning
		assert subscriptionImu, subscriptionSonars

		sleepTime = 1.0 / self.rate
		self.node.create_timer(sleepTime, self.timerCallback)

		while rclpy.ok() and not self.exitFlag:
			# Timeout of 1 due to RCLPY memory leak bug (https://github.com/ros2/rclpy/issues/74, there is already a PR that fixes it, PR#79)
			rclpy.spin_once(self.node, 1.0)
