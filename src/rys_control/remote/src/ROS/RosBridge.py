import rclpy
from math import pi
from time import sleep
from rys_messages import msg as RysMsgs
from std_msgs import msg as RosMsgs
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
		self.publisherEnable = self.node.create_publisher(RosMsgs.Bool, 'rys_enable')
		self.publisherSteering = self.node.create_publisher(RysMsgs.Steering, 'rys_steering')
		self.publisherCalibrateImu = self.node.create_publisher(RosMsgs.Empty, 'rys_imu_calibrate')
		self.publisherSetPIDs = self.node.create_publisher(RosMsgs.Empty, 'rys_set_pids')

		# Create ROS subscribers
		subscriptionImu = self.node.create_subscription(RysMsgs.ImuRoll, 'rys_imu', self.imuSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		subscriptionSonars = self.node.create_subscription(RysMsgs.Sonars, 'rys_sonars', self.sonarsSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
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
		message = RosMsgs.Empty()
		self.publisherCalibrateImu.publish(message)

	def setPIDs(self, speedP, speedI, speedD, angleP, angleI, angleD):
		message = RysMsgs.PIDSettings()
		message.speed_p = speedP
		message.speed_i = speedI
		message.speed_d = speedD
		message.angle_p = angleP
		message.angle_i = angleI
		message.angle_d = angleD
		self.publisherSetPIDs.publish(message)

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

		message = RysMsgs.Steering()
		message.throttle = self.throttle
		message.rotation = self.rotation
		self.publisherSteering.publish(message)

	def enableTimerCallback(self):
		message = RosMsgs.Bool()
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
			sleep(0.02)

		# Cleanup: disable motors
		self.setEnabled(False)
		# Cleanup: destroy timers (threads!)
		self.node.destroy_timer(enableTimer)
		self.node.destroy_timer(steeringTimer)
