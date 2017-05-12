import rclpy
from math import pi
from time import sleep
from rys_interfaces import msg as RysMsgs
from rys_interfaces import srv as RysSrvs
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
		self.publisherEnable = self.node.create_publisher(RosMsgs.Bool, 'rys_control_enable')
		self.publisherSteering = self.node.create_publisher(RysMsgs.Steering, 'rys_control_steering')
		self.publisherCalibrateImu = self.node.create_publisher(RosMsgs.Empty, 'rys_control_imu_calibrate')

		# Create ROS service clients
		self.clientSetRegulatorSettings = self.node.create_client(RysSrvs.SetRegulatorSettings, 'rys_set_regulator_settings')
		self.clientGetRegulatorSettings = self.node.create_client(RysSrvs.GetRegulatorSettings, 'rys_get_regulator_settings')

		# Create ROS subscribers
		subscriptionImu = self.node.create_subscription(RysMsgs.ImuRoll, 'rys_sensor_imu_roll', self.imuSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		subscriptionSonars = self.node.create_subscription(RysMsgs.Sonars, 'rys_sensor_sonars', self.sonarsSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
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

	def setRegulatorParameters(self, parameters):
		request = RysSrvs.SetRegulatorSettings.Request()
		request.speed_kp = parameters['speedKp']
		request.speed_ki = parameters['speedKi']
		request.speed_kd = parameters['speedKd']
		request.angle_kp = parameters['angleKp']
		request.angle_ki = parameters['angleKi']
		request.angle_kd = parameters['angleKd']

		request.speed_filter_factor = parameters['speedFilterFactor']
		request.angle_filter_factor = parameters['rollFilterFactor']
		request.angular_velocity_factor = parameters['angularVelocityFactor']

		request.speed_regulator_enabled = parameters['speedRegulatorEnabled']

		self.clientSetRegulatorSettings.call(request)
		# disable until async way is found
		# self.clientSetRegulatorSettings.wait_for_future()
		# response = self.clientSetRegulatorSettings.response
		# return (response.success, response.error_text)

	def getRegulatorParameters(self):
		return None

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
