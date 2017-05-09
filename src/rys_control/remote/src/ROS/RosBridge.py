import rclpy
from math import pi
from time import sleep
from rys_messages.msg import Steering, ImuRoll, Sonars
from std_msgs import msg
from PyQt5.QtCore import QThread, pyqtSignal

class RosBridge(QThread):
	"""docstring for RosBridge"""

	rollChanged = pyqtSignal(float)
	sonarChanged = pyqtSignal(int, int, int)

	def __init__(self, node, parent = None, steeringMessageRate = 10, enableMessageRate = 0.5):
		super(RosBridge, self).__init__(parent)
		self.node = node
		self.enableMessageRate = enableMessageRate
		self.steeringMessageRate = steeringMessageRate
		self.enabled = False
		self.exitFlag = False

		self.throttle = 0.0
		self.rotation = 0.0
		self.previousRoll = 0.0

		# Create ROS publishers
		self.publisherEnable = self.node.create_publisher(msg.Bool, 'rys_enable')
		self.publisherSteering = self.node.create_publisher(Steering, 'rys_steering')
		self.publisherCalibrateImu = self.node.create_publisher(msg.Empty, 'rys_imu_calibrate')

		# Create ROS subscribers
		subscriptionImu = self.node.create_subscription(ImuRoll, 'rys_imu', self.imuSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		subscriptionSonars = self.node.create_subscription(Sonars, 'rys_sonars', self.sonarsSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		# Prevent unused variable warning
		assert subscriptionImu, subscriptionSonars

	""" Public methods """

	def setEnabled(self, enabled):
		self.enabled = enabled
		self.enableTimerCallback()

	def setSteering(self, throttle, rotation):
		self.throttle = float(throttle)
		self.rotation = float(rotation)

	def calibrateImu(self):
		message = msg.Empty()
		self.publisherCalibrateImu.publish(message)

	def stopExecution(self):
		self.exitFlag = True

	""" Subscription callbacks """

	def imuSubscriptionCallback(self, message):
		roll = message.roll * 180 / pi
		if roll is self.previousRoll:
			return
		self.previousRoll = roll
		self.rollChanged.emit(roll)

	def sonarsSubscriptionCallback(self, message):
		self.sonarChanged.emit(message.front, message.back, message.top)

	""" Publisher timer callbacks """

	def steeringTimerCallback(self):
		if self.enabled is False:
			return

		message = Steering()
		message.throttle = self.throttle
		message.rotation = self.rotation
		self.publisherSteering.publish(message)

	def enableTimerCallback(self):
		message = msg.Bool()
		message.data = self.enabled
		self.publisherEnable.publish(message)

	""" Thread 'run' method """

	def run(self):
		# Create publisher timers
		enableTimerTime = 1.0 / self.enableMessageRate
		enableTimer = self.node.create_timer(enableTimerTime, self.enableTimerCallback)

		steeringTimerTime = 1.0 / self.steeringMessageRate
		steeringTimer = self.node.create_timer(steeringTimerTime, self.steeringTimerCallback)

		# Start main thread loop
		while rclpy.ok() and not self.exitFlag:
			# Timeout of 1 due to RCLPY memory leak bug (https://github.com/ros2/rclpy/issues/74, there is already a PR that fixes it, PR#79)
			rclpy.spin_once(self.node, 1.0)
			sleep(0.01)

		# Cleanup: disable motors
		self.setEnabled(False)
		# Cleanup: destroy timers (threads!)
		self.node.destroy_timer(enableTimer)
		self.node.destroy_timer(steeringTimer)
