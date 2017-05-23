import rclpy
from math import pi
from time import sleep
from rys_interfaces import msg as RysMsgs
from rys_interfaces import srv as RysSrvs
from std_msgs import msg as RosMsgs
from PyQt5.QtCore import QThread, pyqtSignal

class RosBridge(QThread):
	"""docstring for RosBridge"""

	imuChanged = pyqtSignal(float, float)
	sonarChanged = pyqtSignal(int, int, int)
	regulatorSettingsSetDone = pyqtSignal(bool, str)
	regulatorSettingsGetDone = pyqtSignal(object)

	def __init__(self, node, parent = None, steeringMessageRate = 10, enableMessageRate = 0.5):
		super(RosBridge, self).__init__(parent)
		self.node = node
		self.enableMessageRate = enableMessageRate
		self.steeringMessageRate = steeringMessageRate
		self.enabled = False
		self.balancingEnabled = False
		self.exitFlag = False

		self.throttle = 0.0
		self.rotation = 0.0
		self.previousRoll = 0.0
		self.previousRotationX = 0.0

		# Create ROS publishers
		self.publisherEnable = self.node.create_publisher(RosMsgs.Bool, 'rys_control_enable')
		self.publisherBalancingEnabled = self.node.create_publisher(RosMsgs.Bool, 'rys_control_balancing_enabled')
		self.publisherSteering = self.node.create_publisher(RysMsgs.Steering, 'rys_control_steering')
		self.publisherCalibrateImu = self.node.create_publisher(RosMsgs.Empty, 'rys_control_imu_calibrate')

		# Create ROS service clients
		self.clientSetRegulatorSettings = self.node.create_client(RysSrvs.SetRegulatorSettings, 'rys_set_regulator_settings')
		self.clientGetRegulatorSettings = self.node.create_client(RysSrvs.GetRegulatorSettings, 'rys_get_regulator_settings')

		# Create ROS subscribers
		subscriptionImu = self.node.create_subscription(RysMsgs.ImuRollRotation, 'rys_sensor_imu_roll', self.imuSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		subscriptionSonars = self.node.create_subscription(RysMsgs.Sonars, 'rys_sensor_sonars', self.sonarsSubscriptionCallback, rclpy.qos.qos_profile_sensor_data)
		# Prevent unused variable warning
		assert subscriptionImu, subscriptionSonars

	""" Public methods """

	def setEnabled(self, enabled):
		self.enabled = enabled
		self.enableTimerCallback()

	def setBalancingEnabled(self, balancingEnabled):
		message = RosMsgs.Bool()
		message.data = balancingEnabled
		self.publisherBalancingEnabled.publish(message)

	def setSteering(self, throttle, rotation):
		self.throttle = float(throttle)
		self.rotation = float(rotation)

	def calibrateImu(self):
		message = RosMsgs.Empty()
		self.publisherCalibrateImu.publish(message)

	def stopExecution(self):
		self.exitFlag = True

	""" Subscription callbacks """

	def imuSubscriptionCallback(self, message):
		roll = message.roll * 180 / pi
		rotationX = message.rotation_x
		if roll is self.previousRoll and rotationX is self.previousRotationX:
			return
		self.previousRoll = roll
		self.previousRotationX = rotationX

		self.imuChanged.emit(roll, rotationX)

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

	""" QT signal handlers """

	def regulatorSettingsSetRequested(self, parameters):
		request = RysSrvs.SetRegulatorSettings.Request()

		request.speed_filter_factor = parameters['speedFilterFactor']
		request.angle_filter_factor = parameters['rollFilterFactor']
		request.lqr_enabled = parameters['lqrEnabled']
		request.pid_speed_regulator_enabled = parameters['pidSpeedRegulatorEnabled']
		request.pid_speed_kp = parameters['pidSpeedKp']
		request.pid_speed_ki = parameters['pidSpeedKi']
		request.pid_speed_kd = parameters['pidSpeedKd']
		request.pid_angle_kp = parameters['pidAngleKp']
		request.pid_angle_ki = parameters['pidAngleKi']
		request.pid_angle_kd = parameters['pidAngleKd']
		request.lqr_linear_velocity_k = parameters['lqrLinearVelocityK']
		request.lqr_angular_velocity_k = parameters['lqrAngularVelocityK']
		request.lqr_angle_k = parameters['lqrAngleK']

		self.clientSetRegulatorSettings.call(request)
		# Locks the UI - disabled until further investigation
		#
		# while rclpy.ok():
		# 	response = self.clientSetRegulatorSettings.response
		# 	if response is not None:
		# 		self.regulatorSettingsSetDone(response.success, response.error_text)
		# 		break
		# 	rclpy.spin_once(self.node, 1.0)
		# 	sleep(0.1)

	def regulatorSettingsGetRequested(self):
		# TODO
		pass

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

			response = self.clientSetRegulatorSettings.response
			if response is not None:
				self.regulatorSettingsSetDone.emit(response.success, response.error_text)

			sleep(0.02)

		# Cleanup: disable motors
		self.setEnabled(False)
		# Cleanup: destroy timers (threads!)
		self.node.destroy_timer(enableTimer)
		self.node.destroy_timer(steeringTimer)
